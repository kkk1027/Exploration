#!/usr/bin/env bash
# start_explore_gui.sh —— 拉起"完整探索建图"（Gazebo 仅后台 gzserver，不启界面；带 rviz2）
set +u
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy ROS_LOCALHOST_ONLY
source /home/kbf/stemm_sim_ws/domain_guard.sh || exit 9
source /opt/ros/humble/setup.bash
source /home/kbf/stemm_sim_ws/install/setup.bash
source /home/kbf/stemm_explore_ws/install/setup.bash
source /home/kbf/stemm_exploration_sim_ws/install/setup.bash
export DISPLAY=${DISPLAY:-:0}
export QT_QPA_PLATFORM=xcb
OUT=/home/kbf/stemm_exploration_sim_ws/logs/live_$(date +%H%M%S); mkdir -p "$OUT"
echo "日志 $OUT/launch.log"
setsid nohup ros2 launch stemm_cartographer_exploration_sim sim_explore.launch.py \
  rviz:=true explore_sec:=${1:-180.0} > "$OUT/launch.log" 2>&1 < /dev/null &
echo "$OUT" > /tmp/live_dir

# ③ 演示收尾：地图落盘后自动关 rviz2（避免 GUI 负载影响后续测试）
(
  for i in $(seq 1 90); do
    sleep 5
    if [ -f /home/kbf/stemm_sim_ws/maps/explore_latest.pgm ]; then
      sleep 5; pkill -x rviz2 2>/dev/null && echo "[demo] rviz2 已自动关闭"; break
    fi
  done
) &
