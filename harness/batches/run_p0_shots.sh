#!/usr/bin/env bash
# run_p0_shots.sh <tag前缀> <轮数>
# 每轮：正常跑（RVIZ=true 开 rviz2、不开 gazebo），同时
#   * 起 costmap_residue_probe 采 /local_costmap 两层 + 偏航角速度（P0-A 取证）；
#   * 每 5 s 截一次 rviz2 窗口（xwd→ffmpeg 转 png），存 <轮目录>/shots/。
set -u
cd /home/kbf/stemm_exploration_sim_ws || exit 9
PREFIX="$1"; N="$2"
PROG="/tmp/${PREFIX}_progress.log"; : > "$PROG"
WS=/home/kbf/stemm_exploration_sim_ws
START="${START:-1}"
for i in $(seq "$START" "$(( START + N - 1 ))") ; do
  # 起栈前清场：上一轮若异常退出会留下 gzserver / /dev/shm 里的 fastrtps 段，
  # 让新一轮 gzserver 迟迟起不来 → sensor preflight 8 s 超时 → 整轮直接作废。
  # 2026-10-26 修复"批次切换时轮次拿不到轮目录/起栈失败"（连续 3 轮复现）：
  #   ① 收掉上一批残留（整栈按可执行路径枚举，比 pkill 单进程可靠）
  #   ② 删掉上一轮被杀后可能仍被孤儿持有的 flock 文件
  #   ③ 轮询等 gzserver 真正归零（最多 30 s），再清 fastrtps 共享内存段
  bash harness/stack_kill.sh term >/dev/null 2>&1
  pkill -x gzserver 2>/dev/null; pkill -x gzclient 2>/dev/null
  rm -f /tmp/stemm_round.lock 2>/dev/null
  for _ in $(seq 1 30); do [ "$(pgrep -cx gzserver)" = "0" ] && break; sleep 1; done
  rm -f /dev/shm/fastrtps_* 2>/dev/null
  sleep 2
  echo "  清场完成 gzserver=$(pgrep -cx gzserver) lock=$( [ -e /tmp/stemm_round.lock ] && echo 有 || echo 无 )" >> "$PROG"
  TAG="${PREFIX}r$i"
  echo "START $TAG $(date +%H:%M:%S)" >> "$PROG"
  # 轮次在后台起，方便同时挂探针与截图
  # ROUND_MARGIN：探索结束后等"返航/收尾"的余量（真实秒）。用户口径：返航失败只记录不解决，
  # 所以余量给小一点，避免每轮都吃满 300 s 把单轮拖到 14 分钟。两批必须一致。
  RVIZ=true GUI=false VISION=1 POSE_OVERRIDE="0.0 0.0 0.0" ROUND_MARGIN="${ROUND_MARGIN:-90}" RECOVERY_BT="${RECOVERY_BT:-none}" \
    timeout -k 10 $(( 180 + ${ROUND_MARGIN:-90} + 300 )) bash harness/round.sh "$TAG" 180 \
    >/tmp/${TAG}.log 2>&1 9>&- &
  RPID=$!
  # 等轮目录出现
  D=""; for _ in $(seq 1 150); do D=$(ls -dt logs/round_*_"$TAG" 2>/dev/null | head -1); [ -n "$D" ] && break; sleep 1; done
  if [ -z "$D" ]; then echo "  ✗ 没等到轮目录（150 s）" >> "$PROG"; kill -9 $RPID 2>/dev/null; pkill -9 -f "harness/round[.]sh $TAG" 2>/dev/null; sleep 3; continue; fi
  mkdir -p "$D/shots"
  # 探针（等栈起来再挂，最多等 90 s）
  (
    # 关键：本脚本首行是 `set -u`，而 ROS 的 setup.bash 里有未绑定变量，
    # `set -u` 下 source 会直接 "未绑定的变量" 让**整个子 shell 静默退出**
    # （stderr 又被重定向掉）——这正是此前 p1/p2/p3/base 各轮探针一根采样都没有的根因。
    set +u
    source /opt/ros/humble/setup.bash >/dev/null 2>&1
    source /home/kbf/stemm_sim_ws/install/setup.bash >/dev/null 2>&1
    source /home/kbf/stemm_explore_ws/install/setup.bash >/dev/null 2>&1
    source /home/kbf/stemm_exploration_sim_ws/install/setup.bash >/dev/null 2>&1
    export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
    # 门控用"timeline.jsonl 出现"（arbiter 起来后才写），不要用 ros2 topic list：
    # daemon 不稳时它每次能挂满 6 s 超时，把整轮等完（实测探针一次都没跑起来）。
    for _ in $(seq 1 120); do [ -s "$D/timeline.jsonl" ] && break; sleep 1; done
    sleep 5
    echo "probe start $(date +%H:%M:%S) timeline=$(wc -l < "$D/timeline.jsonl" 2>/dev/null)"
    timeout 400 python3 harness/costmap_residue_probe.py --out "$D/costmap"
  ) > "$D/costmap_probe.log" 2>&1 &
  PPID_PROBE=$!
  # 截图循环：每 5 s 一张 rviz2 窗口
  ( for k in $(seq 1 120); do
      WID=$(xwininfo -root -tree 2>/dev/null | grep -iE '"[^"]*[Rr][Vv]iz' | head -1 | awk '{print $1}')
      if [ -n "$WID" ]; then
        G=$(xwininfo -id "$WID" 2>/dev/null | grep -E "Width:|Height:|Absolute upper-left X:|Absolute upper-left Y:")
        W=$(printf '%s' "$G" | awk '/Width:/{print $2}'); H=$(printf '%s' "$G" | awk '/Height:/{print $2}')
        X=$(printf '%s' "$G" | awk '/Absolute upper-left X:/{print $4}'); Y=$(printf '%s' "$G" | awk '/Absolute upper-left Y:/{print $4}')
        if [ -n "$W" ] && [ -n "$H" ]; then
          ffmpeg -loglevel error -f x11grab -video_size "${W}x${H}" -i ":0.0+${X},${Y}" -frames:v 1 \
            -y "$D/shots/rviz_$(date +%H%M%S).png" >/dev/null 2>&1
        fi
      fi
      sleep 5
    done ) &
  SPID=$!
  wait $RPID
  kill -9 $PPID_PROBE $SPID 2>/dev/null
  echo "DONE $TAG $(date +%H:%M:%S) dir=$D shots=$(ls "$D/shots" 2>/dev/null | wc -l)" >> "$PROG"
  # 起栈期失败（gzserver 没起来 → preflight 超时）不算实验数据：删掉重来一次
  # 注意：不能匹配 "gzserver 已退出" —— 那是 round.sh 正常的收尾打印（踩过一次，
  # 把跑完整的轮次误判成起栈失败又重跑了一遍，白花 5 分钟）。只认"真起不来"的特征。
  if grep -qE "sensor preflight timed out|Core sensor preflight failed|nav2 生命周期未在 guard 超时内激活" "$D/launch.log" 2>/dev/null \
     || ! [ -s "$D/timeline.jsonl" ] || ! [ -s "$D/digest.md" ]; then
    echo "  ↻ $TAG 起栈失败，重跑一次" >> "$PROG"
    rm -rf "$D"
    pkill -x gzserver 2>/dev/null; rm -f /dev/shm/fastrtps_* 2>/dev/null; sleep 5
    RVIZ=true GUI=false VISION=1 POSE_OVERRIDE="0.0 0.0 0.0" ROUND_MARGIN="${ROUND_MARGIN:-90}" RECOVERY_BT="${RECOVERY_BT:-none}" \
      timeout -k 10 $(( 180 + ${ROUND_MARGIN:-90} + 300 )) bash harness/round.sh "$TAG" 180 \
      >/tmp/${TAG}.retry.log 2>&1 9>&- &
    wait $!
    echo "  ✓ $TAG 重跑结束 $(date +%H:%M:%S) dir=$(ls -dt logs/round_*_"$TAG" 2>/dev/null | head -1)" >> "$PROG"
  fi
done
echo "ALL DONE $(date +%H:%M:%S)" >> "$PROG"
