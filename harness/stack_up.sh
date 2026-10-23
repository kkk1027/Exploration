#!/usr/bin/env bash
# stack_up.sh —— 起一套仿真栈并**保持运行**（调试用；正式验收仍走 round.sh / check_vision_layer.sh）
# 用法: harness/stack_up.sh <vision 0|1> [spawn_x] [spawn_y] [yaw]
set +u
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy ROS_LOCALHOST_ONLY
source /home/kbf/stemm_sim_ws/domain_guard.sh || exit 9
source /opt/ros/humble/setup.bash >/dev/null
source /home/kbf/stemm_sim_ws/install/setup.bash >/dev/null
source /home/kbf/stemm_explore_ws/install/setup.bash >/dev/null
source /home/kbf/stemm_exploration_sim_ws/install/setup.bash >/dev/null
V="${1:-1}"; SX="${2:-0.0}"; SY="${3:-0.0}"; YAW="${4:-0.0}"
BASE=/home/kbf/stemm_exploration_sim_ws
OUT="$BASE/logs/live_v${V}_$(date +%Y%m%d-%H%M%S)"; mkdir -p "$OUT"
echo "$OUT" > /tmp/live_stack_dir
if [ "$V" = "1" ]; then python3 harness/vision_nav_params.py --apply > "$OUT/params_apply.log" 2>&1
else python3 harness/vision_nav_params.py --restore > "$OUT/params_apply.log" 2>&1 || true; fi
setsid bash -c "echo \$\$ > '$OUT/pgid'; exec ros2 launch stemm_cartographer_exploration_sim sim_explore.launch.py \
  vision:=$V rviz:=false escape_supervisor:=false auto_explore:=false explore_sec:=1800.0 \
  spawn_x:=$SX spawn_y:=$SY spawn_yaw:=$YAW" > "$OUT/launch.log" 2>&1 < /dev/null &
sleep 3
for i in $(seq 1 60); do
  grep -q "ready_guard_timing" "$OUT/launch.log" 2>/dev/null && { echo "READY after $((i*4))s"; break; }
  sleep 4
done
grep -E "Using plugin" "$OUT/launch.log" | sed 's/.*Using plugin/  Using plugin/'
echo "OUT=$OUT"
