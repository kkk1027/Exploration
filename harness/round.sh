#!/usr/bin/env bash
# round.sh —— 探索建图单轮运行器（确定性启动/收尾 + 六项验收取证）
# 用法: round.sh <标签> <探索秒数> [启动后等待秒数(仅空跑用)]
# 产物: runs/<时间戳>-<标签>.jsonl 摘要 + logs/round_<时间戳>_<标签>/launch.log
set +u
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy ROS_LOCALHOST_ONLY
source /home/kbf/stemm_sim_ws/domain_guard.sh || exit 9
source /opt/ros/humble/setup.bash
source /home/kbf/stemm_sim_ws/install/setup.bash
source /home/kbf/stemm_explore_ws/install/setup.bash
source /home/kbf/stemm_exploration_sim_ws/install/setup.bash

TAG="${1:-run}"; EXPLORE="${2:-120.0}"; HOLD="${3:-0}"
BASE=/home/kbf/stemm_exploration_sim_ws
TS=$(date +%Y%m%d-%H%M%S)
OUT="$BASE/logs/round_${TS}_${TAG}"; mkdir -p "$OUT"
J="$BASE/runs/${TS}-${TAG}.jsonl"
say() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$OUT/harness.log"; }
rec() { printf '{"ts":"%s","tag":"%s","%s"}\n' "$(date -Iseconds)" "$TAG" "$1" >> "$J"; }

# ---- 1) 启动前断言：域 42 零节点 + 无 gzserver ----
proc_residue() { pgrep -af "waypoint_follower|controller_server|planner_server|bt_navigator|behavior_server|smoother_server|cartographer_node|occupancy_grid|stemm_|robot_state_publisher|ekf_node|imu_bias_shim|scan_deadzone_relay|async_slam_toolbox" 2>/dev/null | grep -E "/(lib/|install/|opt/ros)" | grep -v "harness/round.sh" | awk '{print $1}' | tr '\n' ' '; }
NODES=$(ros2 node list 2>/dev/null | wc -l)
GZ=$(pgrep -cx gzserver)
PRES=$(proc_residue)
say "启动前：ROS 节点数=$NODES（含DDS僵尸，仅记录）  gzserver=$GZ  进程残留=[${PRES}]"
rec "\"check\":\"pre_launch\",\"nodes\":$NODES,\"gzserver\":$GZ,\"residue\":\"${PRES}\""
if [ -n "$PRES" ] || [ "$GZ" -gt 0 ]; then
  say "存在残留，先清理"
  pgrep -af "waypoint_follower|controller_server|planner_server|bt_navigator|behavior_server|smoother_server|cartographer_node|occupancy_grid|stemm_|ros2 launch|robot_state_publisher|ekf_node|imu_bias_shim|scan_deadzone_relay|sim_clock_resampler|async_slam_toolbox" 2>/dev/null \
    | grep -E "/(lib/|install/|opt/ros|nodes/|scripts/)" | grep -v "harness/round.sh" \
    | awk '{print $1}' | while read pid; do kill -TERM "$pid" 2>/dev/null; done
  pkill -x gzserver 2>/dev/null; sleep 4
  NODES=$(ros2 node list 2>/dev/null | wc -l); GZ=$(pgrep -cx gzserver); PRES=$(proc_residue)
  say "清理后：ROS 节点数=$NODES  gzserver=$GZ  进程残留=[${PRES}]"
  rec "\"check\":\"post_cleanup\",\"nodes\":$NODES,\"gzserver\":$GZ,\"residue\":\"${PRES}\""
  { [ -n "$PRES" ] || [ "$GZ" -gt 0 ]; } && { say "清理不彻底，放弃本轮"; rec '"verdict":"FAIL_preclean"'; exit 10; }
fi

rm -f /home/kbf/stemm_sim_ws/maps/explore_latest.pgm /home/kbf/stemm_sim_ws/maps/explore_latest.yaml
T_START=$(date +%s)

# 随机起点（5 个候选，房间原点附近的安全偏移位姿）
POSES=("0.0 0.0 0.0" "0.4 0.3 1.5708" "-0.3 0.4 3.1416" "0.3 -0.4 -1.5708" "-0.4 -0.3 0.7854")
POSE=${POSES[$((RANDOM % ${#POSES[@]}))]}
set -- $POSE; SX=$1; SY=$2; SYAW=$3
say "本轮随机起点：x=$SX y=$SY yaw=$SYAW"
rec "\"check\":\"pose\",\"x\":$SX,\"y\":$SY,\"yaw\":$SYAW"

# ---- 2) 启动（记录真实 PGID） ----
setsid bash -c "echo \$\$ > '$OUT/pgid'; exec ros2 launch stemm_cartographer_exploration_sim sim_explore.launch.py explore_sec:=$EXPLORE spawn_x:=$SX spawn_y:=$SY spawn_yaw:=$SYAW" \
  > "$OUT/launch.log" 2>&1 < /dev/null &
sleep 3; PGID=$(cat "$OUT/pgid" 2>/dev/null || echo "")
say "已启动：PGID=$PGID  日志 $OUT/launch.log"
rec "\"check\":\"launch\",\"pgid\":\"$PGID\""

if [ "$HOLD" -gt 0 ]; then
  sleep "$HOLD"
  say "空跑 $HOLD s 结束，执行 teardown 验证"
else
  # 全流程模式：等会话自然收尾（会话结束 / 地图落盘 / 超时上限）
  MAXW=$(python3 -c "print(int(float('$EXPLORE') + 300))")
  say "全流程模式：最多等待 ${MAXW}s（探索 ${EXPLORE}s + 返航/存图余量）"
  waited=0
  while [ "$waited" -lt "$MAXW" ]; do
    sleep 10; waited=$((waited+10))
    if grep -q "会话结束" "$OUT/launch.log" 2>/dev/null; then say "检测到会话结束（${waited}s）"; break; fi
    MF=/home/kbf/stemm_sim_ws/maps/explore_latest.pgm
    if [ -f "$MF" ] && [ "$(stat -c %Y "$MF")" -ge "$T_START" ]; then say "地图已落盘（${waited}s）"; break; fi
    pgrep -x gzserver >/dev/null || { say "gzserver 已退出（${waited}s）"; break; }
  done
fi

# ---- 3) teardown（按 PGID） ----
teardown() {
  [ -n "$PGID" ] && kill -TERM -- "-$PGID" 2>/dev/null
  for i in $(seq 1 20); do pgrep -x gzserver >/dev/null || break; sleep 1; done
  pkill -x gzserver 2>/dev/null
  for i in $(seq 1 10); do [ "$(ros2 node list 2>/dev/null | wc -l)" -le 1 ] && break; sleep 1; done
}
teardown
NODES=$(ros2 node list 2>/dev/null | wc -l); GZ=$(pgrep -cx gzserver)
RES=$(pgrep -af "cartographer_node|sim_session_driver|controller_server|planner_server|bt_navigator|imu_bias_shim|scan_deadzone_relay" 2>/dev/null | grep -E "/(lib/|install/|opt/ros)" | grep -v "harness/round.sh" | awk '{print $1}' | tr '\n' ' ')
say "收尾后：ROS 节点数=$NODES  gzserver=$GZ  残留=[${RES}]"
rec "\"check\":\"post_teardown\",\"nodes\":$NODES,\"gzserver\":$GZ,\"residue\":\"${RES}\""
[ "$GZ" = "0" ] && [ -z "$RES" ] && rec '"verdict":"PASS_clean"' || rec '"verdict":"FAIL_residue"'
echo "JSONL: $J"
