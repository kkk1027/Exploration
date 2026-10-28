#!/usr/bin/env bash
# diag_tf_axis.sh —— 起 nav_only（车静止）→ 等 amcl active → 跑 tf_probe 量"三种 TF 取法"。
# 目的：确认视觉层失败的到底是"帧不存在"还是"时间轴对不上"。
set -u
cd /home/kbf/stemm_exploration_sim_ws || exit 9
OUT="logs/diag_axis_$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT"

set +u
source /home/kbf/stemm_sim_ws/domain_guard.sh >/dev/null 2>&1
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source /home/kbf/stemm_sim_ws/install/setup.bash >/dev/null 2>&1
source /home/kbf/stemm_explore_ws/install/setup.bash >/dev/null 2>&1
source /home/kbf/stemm_exploration_sim_ws/install/setup.bash >/dev/null 2>&1
export ROS_LOCALHOST_ONLY=1
set -u

exec 9>/tmp/stemm_round.lock
flock -n 9 || { echo "已有轮次在跑"; exit 3; }
bash harness/preflight_dds.sh -q || { echo "环境不健康"; exit 4; }

cleanup() {
  [ -n "${LPID:-}" ] && kill -TERM -- "-$LPID" 2>/dev/null
  sleep 3
  [ -n "${LPID:-}" ] && kill -KILL -- "-$LPID" 2>/dev/null
  bash harness/stack_kill.sh kill >/dev/null 2>&1
  python3 harness/vision_nav_params.py --restore >/dev/null 2>&1
  echo "[axis] 收尾 gz=$(pgrep -cx gzserver 2>/dev/null || true)"
}
trap cleanup EXIT INT TERM

setsid ros2 launch stemm_cartographer_exploration_sim nav_only.launch.py \
  map_yaml:=/home/kbf/stemm_sim_ws/maps/explore_latest.yaml \
  spawn_x:=0.0 spawn_y:=0.0 spawn_yaw:=0.0 \
  vision:=true vision_mode:=accum vision_tf_stamp:=true \
  gui:=false rviz:=false > "$OUT/launch.log" 2>&1 &
LPID=$!
for _ in $(seq 1 90); do
  st=$(timeout 8 ros2 lifecycle get /amcl 2>/dev/null | head -1)
  case "$st" in active*) break ;; esac
  sleep 2
done
sleep 6
echo "=== 车静止 12 s 后探测（此时 amcl 早已 active）==="
timeout 60 python3 harness/tf_probe.py map camera_depth_optical_frame 3 2>&1 | tee "$OUT/probe.txt"
echo "=== amcl 的关键参数 ==="
for p in update_min_d update_min_a transform_tolerance tf_broadcast laser_max_range; do
  echo "$p = $(timeout 8 ros2 param get /amcl $p 2>&1 | tail -1)"
done | tee -a "$OUT/probe.txt"
echo "=== 视觉层结论行 ==="
sed 's/\x1b\[[0-9;]*m//g' "$OUT/launch.log" | grep -a '累积使能' | tail -2 | tee -a "$OUT/probe.txt"
echo "[axis] 输出 $OUT"
