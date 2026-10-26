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
for i in $(seq 1 "$N"); do
  TAG="${PREFIX}r$i"
  echo "START $TAG $(date +%H:%M:%S)" >> "$PROG"
  # 轮次在后台起，方便同时挂探针与截图
  RVIZ=true GUI=false VISION=1 POSE_OVERRIDE="0.0 0.0 0.0" \
    setsid bash harness/round.sh "$TAG" 120 >/tmp/${TAG}.log 2>&1 &
  RPID=$!
  # 等轮目录出现
  D=""; for _ in $(seq 1 60); do D=$(ls -dt logs/round_*_"$TAG" 2>/dev/null | head -1); [ -n "$D" ] && break; sleep 1; done
  if [ -z "$D" ]; then echo "  ✗ 没等到轮目录" >> "$PROG"; kill -9 $RPID 2>/dev/null; continue; fi
  mkdir -p "$D/shots"
  # 探针（等栈起来再挂，最多等 90 s）
  (
    source /opt/ros/humble/setup.bash >/dev/null 2>&1
    source /home/kbf/stemm_sim_ws/install/setup.bash >/dev/null 2>&1
    source /home/kbf/stemm_explore_ws/install/setup.bash >/dev/null 2>&1
    source /home/kbf/stemm_exploration_sim_ws/install/setup.bash >/dev/null 2>&1
    export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
    # 不要用 ros2 topic list 做门控：daemon 不稳时它每次能挂满 6 s 超时，
    # 45 次迭代就把整轮等完了（实测探针一次都没跑起来）。固定等 45 s 直接挂探针。
    sleep 45
    timeout 400 python3 harness/costmap_residue_probe.py --out "$D/costmap" > "$D/costmap_probe.log" 2>&1
  ) &
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
done
echo "ALL DONE $(date +%H:%M:%S)" >> "$PROG"
