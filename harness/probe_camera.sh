#!/usr/bin/env bash
# probe_camera.sh —— 只起机器人层（stack:=false），验证深度相机话题/TF/点云是否真实可用
# 用法: harness/probe_camera.sh [等待秒数]         默认 45 s
set +u
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy ROS_LOCALHOST_ONLY
source /home/kbf/stemm_sim_ws/domain_guard.sh || exit 9
source /opt/ros/humble/setup.bash
source /home/kbf/stemm_sim_ws/install/setup.bash
source /home/kbf/stemm_explore_ws/install/setup.bash
source /home/kbf/stemm_exploration_sim_ws/install/setup.bash
WAIT="${1:-45}"
BASE=/home/kbf/stemm_exploration_sim_ws
OUT="$BASE/logs/probe_camera_$(date +%Y%m%d-%H%M%S)"; mkdir -p "$OUT"
echo "[probe] 机器人层启动，等待 ${WAIT}s …"
setsid bash -c "exec ros2 launch stemm_cartographer_exploration_sim sim_explore.launch.py \
  stack:=false rviz:=false escape_supervisor:=false" > "$OUT/launch.log" 2>&1 < /dev/null &
PG=$!
sleep "$WAIT"
{
  echo "=== 话题（camera 相关）==="
  timeout 20 ros2 topic list 2>/dev/null | grep -iE "camera|depth|vision" || echo "(无 camera 话题)"
  echo
  echo "=== 图像/点云帧率 ==="
  for t in /camera/depth/image_raw /camera/color/image_raw /camera/depth/points; do
    printf '%-32s ' "$t"
    timeout 12 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate" || echo "无数据"
  done
  echo
  echo "=== 点云 header.frame_id ==="
  timeout 15 ros2 topic echo --once /camera/depth/points --field header 2>/dev/null | head -6 || echo "无点云"
  echo
  echo "=== 点云规模/字段 ==="
  timeout 15 ros2 topic echo --once /camera/depth/points --field width 2>/dev/null | head -2
  timeout 15 ros2 topic echo --once /camera/depth/points --field height 2>/dev/null | head -2
  echo
  echo "=== TF: base_footprint -> camera_depth_optical_frame ==="
  timeout 12 ros2 run tf2_ros tf2_echo base_footprint camera_depth_optical_frame 2>/dev/null | head -12 || echo "TF 不可达"
  echo
  echo "=== TF: odom_combined -> camera_depth_optical_frame ==="
  timeout 12 ros2 run tf2_ros tf2_echo odom_combined camera_depth_optical_frame 2>/dev/null | head -8 || echo "TF 不可达"
} 2>&1 | tee "$OUT/probe.txt"
echo "[probe] 收尾"
kill -TERM -- "-$PG" 2>/dev/null; sleep 2; pkill -x gzserver 2>/dev/null; sleep 3
kill -9 -- "-$PG" 2>/dev/null; pkill -9 -x gzserver 2>/dev/null
sleep 1; pkill -9 -f "wheeltec_two_wheel_model/(scan_deadzone_relay|imu_bias_shim)" 2>/dev/null
echo "[probe] 残留检查：gzserver=$(pgrep -cx gzserver)  launch=$(pgrep -fc 'sim_explore.launch.py')"
echo "[probe] 产物：$OUT/probe.txt"
