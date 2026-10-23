#!/usr/bin/env bash
# check_local_layer.sh —— 取证"局部体素层真的在标记"（全局层已由 vision_check.py 取证）
# 做法：在 cafe_table 前 1.3 m 起栈（世界 2.2,1.37；map 原点=起点，桌面在 map(1.34,0)），
#       直接把 /local_costmap/costmap 与 /vision/obstacle_grid 做 ASCII 对照。
set +u
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy ROS_LOCALHOST_ONLY
source /home/kbf/stemm_sim_ws/domain_guard.sh >/dev/null 2>&1
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source /home/kbf/stemm_sim_ws/install/setup.bash >/dev/null 2>&1
source /home/kbf/stemm_explore_ws/install/setup.bash >/dev/null 2>&1
source /home/kbf/stemm_exploration_sim_ws/install/setup.bash >/dev/null 2>&1
OUT=/home/kbf/stemm_exploration_sim_ws/logs/local_layer_$(date +%Y%m%d-%H%M%S); mkdir -p "$OUT"
# 起点放在桌子前 1.3 m（世界 2.2,1.37），保证 cafe_table 落在 6x6 m 局部窗口内
bash harness/stack_up.sh 1 2.2 1.37 0.0 > "$OUT/stack.log" 2>&1
grep -E "READY|Using plugin" "$OUT/stack.log" | sed 's/.*Using plugin/  Using plugin/'
echo "=== 等视觉层喂进局部窗口（相机 5 Hz，局部 5 Hz）==="
sleep 15
echo "=== 局部代价地图切片（中心 = cafe_table_0 在 map 坐标系 (1.34,0)）==="
python3 harness/costmap_dump.py --topic /local_costmap/costmap --x 1.34 --y 0.0 --half 1.2 --wait 30 \
  | tee "$OUT/local_dump.txt"
echo "=== 局部层是否订阅了 /vision/live_cloud ==="
timeout 15 ros2 topic info /vision/live_cloud -v 2>/dev/null | grep -A3 "Subscription count" | head -6 | tee -a "$OUT/local_dump.txt"
bash harness/stack_down.sh > "$OUT/stack_down.log" 2>&1
echo "产物: $OUT"
