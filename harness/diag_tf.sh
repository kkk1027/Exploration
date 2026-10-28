#!/usr/bin/env bash
# diag_tf.sh —— nav_only 里"map 帧到底存不存在"的最小取证（只读探针，不改代码）。
# 用法：bash harness/diag_tf.sh [秒]
set -u
cd /home/kbf/stemm_exploration_sim_ws || exit 9
SECS="${1:-40}"
OUT="logs/diag_tf_$(date +%Y%m%d-%H%M%S)"
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
  echo "[diag] 收尾 gz=$(pgrep -cx gzserver 2>/dev/null || true)"
}
trap cleanup EXIT INT TERM

setsid ros2 launch stemm_cartographer_exploration_sim nav_only.launch.py \
  map_yaml:=/home/kbf/stemm_sim_ws/maps/explore_latest.yaml \
  spawn_x:=0.0 spawn_y:=0.0 spawn_yaw:=0.0 \
  vision:=true vision_mode:=accum vision_tf_stamp:=true vision_tf_debug:=true \
  gui:=false rviz:=false > "$OUT/launch.log" 2>&1 &
LPID=$!

# 等 amcl active
for _ in $(seq 1 90); do
  st=$(timeout 8 ros2 lifecycle get /amcl 2>/dev/null | head -1)
  case "$st" in active*) break ;; esac
  sleep 2
done
sleep 12

{
  echo "===== amcl 参数 ====="
  for p in tf_broadcast global_frame_id odom_frame_id base_frame_id set_initial_pose; do
    echo "$p = $(timeout 8 ros2 param get /amcl $p 2>&1 | tail -1)"
  done
  echo "===== /tf 上出现过的 parent->child（5 s 采样）====="
  timeout 6 ros2 topic echo /tf --once 2>/dev/null | grep -E 'frame_id|child_frame_id' | head -20
  echo "===== /tf_static ====="
  timeout 6 ros2 topic echo /tf_static --once 2>/dev/null | grep -E 'frame_id|child_frame_id' | head -20
  echo "===== tf2_echo map odom_combined（5 s）====="
  timeout 6 ros2 run tf2_ros tf2_echo map odom_combined 2>&1 | head -6
  echo "===== tf2_echo map camera_depth_optical_frame（5 s）====="
  timeout 6 ros2 run tf2_ros tf2_echo map camera_depth_optical_frame 2>&1 | head -6
  echo "===== 话题 ====="
  for t in /scan /camera/depth/points /vision/live_cloud /vision/obstacle_grid; do
    echo "hz $t => $(timeout 8 ros2 topic hz $t 2>/dev/null | grep -m1 'average rate' || echo 无)"
  done
  echo "===== 视觉节点自己的 buffer 帧表（tf_debug）====="
  sed 's/\x1b\[[0-9;]*m//g' "$OUT/launch.log" | grep -a -A25 'tf_debug\] lookup' | tail -30
  echo "===== 视觉节点 TF 计数 ====="
  sed 's/\x1b\[[0-9;]*m//g' "$OUT/launch.log" | grep -a '累积使能' | tail -2
} 2>&1 | tee "$OUT/diag.txt"

sleep "$SECS" 2>/dev/null || true
echo "[diag] 输出 $OUT"
