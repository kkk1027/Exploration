#!/usr/bin/env bash
# check_vision_layer.sh —— 方案A 集成自检：注入视觉图层 → 起栈 → 交叉验证 → 收尾撤销
# 用法: harness/check_vision_layer.sh [标签]
set +u
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy ROS_LOCALHOST_ONLY
source /home/kbf/stemm_sim_ws/domain_guard.sh || exit 9
source /opt/ros/humble/setup.bash
source /home/kbf/stemm_sim_ws/install/setup.bash
source /home/kbf/stemm_explore_ws/install/setup.bash
source /home/kbf/stemm_exploration_sim_ws/install/setup.bash
TAG="${1:-vchk}"
BASE=/home/kbf/stemm_exploration_sim_ws
OUT="$BASE/logs/vision_${TAG}_$(date +%Y%m%d-%H%M%S)"; mkdir -p "$OUT"
say() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$OUT/harness.log"; }

# 启动前残留断言（上一轮被强杀的子进程会占着 /scan 发布者，preflight 会判"2 个发布者"整轮打断）
residue() {
  ps -eo pid,args | grep -E "wheeltec_two_wheel_model/(scan_deadzone_relay|imu_bias_shim)|cartographer_exploration_sim/(collision_probe|session_driver)|gzserver" \
    | grep -vE "grep|check_vision_layer" | awk '{print $1}' | tr '\n' ' '
}
R=$(residue); [ -n "$R" ] && { say "清理残留: $R"; for p in $R; do kill -TERM "$p" 2>/dev/null; done; sleep 3;
  for p in $R; do kill -9 "$p" 2>/dev/null; done; sleep 1; }
say "启动前残留=[$(residue)]"

say "1) 注入视觉图层"
python3 harness/vision_nav_params.py --apply 2>&1 | tee -a "$OUT/harness.log"
[ "${PIPESTATUS[0]}" = "0" ] || { say "注入失败"; exit 11; }
trap 'python3 /home/kbf/stemm_exploration_sim_ws/harness/vision_nav_params.py --restore >/dev/null 2>&1' EXIT INT TERM

NAVF=$(readlink -f "$(ros2 pkg prefix stemm_cartographer_exploration)/share/stemm_cartographer_exploration/config/stemm_safe_nav2_params.yaml")
say "补丁文件: $NAVF"
say "启动前: md5=$(md5sum "$NAVF" | awk '{print $1}')  哨兵块=$(grep -c 'vision_fusion_patch:begin' "$NAVF")"
grep -n "vision_voxel_layer\|vision_obstacle_layer" "$NAVF" | head -4 | tee -a "$OUT/harness.log"

say "2) 起栈（vision:=true，stack_delay=65）"
setsid bash -c "echo \$\$ > '$OUT/pgid'; exec ros2 launch stemm_cartographer_exploration_sim sim_explore.launch.py \
  vision:=true rviz:=false escape_supervisor:=false explore_sec:=30.0 \
  spawn_x:=0.0 spawn_y:=0.0 spawn_yaw:=0.0" > "$OUT/launch.log" 2>&1 < /dev/null &
sleep 3; PGID=$(cat "$OUT/pgid" 2>/dev/null); say "PGID=$PGID"

say "3) 等栈就绪（最多 240 s）"
for i in $(seq 1 60); do
  grep -q "ready_guard_timing" "$OUT/launch.log" 2>/dev/null && { say "栈就绪（$((i*4))s）"; break; }
  sleep 4
done
say "就绪后复核: md5=$(md5sum "$NAVF" | awk '{print $1}')  哨兵块=$(grep -c 'vision_fusion_patch:begin' "$NAVF")"
say "nav2 实际加载的图层："
grep -E "Using plugin" "$OUT/launch.log" | sed 's/.*Using plugin/  Using plugin/' | tee -a "$OUT/harness.log"

say "4) 再等 30 s 让视觉累积（相机 5 Hz → 5 cm 栅格）"
sleep 30
grep -E "视觉障碍：" "$OUT/launch.log" | tail -3 | tee -a "$OUT/harness.log"

say "5) 交叉验证：视觉障碍是否进入全局代价地图"
RVIZ= python3 harness/vision_check.py --timeout 60 --json "$OUT/vision_check.json" 2>&1 | tee -a "$OUT/harness.log"
RC=${PIPESTATUS[0]}
say "vision_check rc=$RC"

say "6) 收尾"
[ -n "$PGID" ] && kill -TERM -- "-$PGID" 2>/dev/null
for i in $(seq 1 20); do pgrep -x gzserver >/dev/null || break; sleep 1; done
pkill -x gzserver 2>/dev/null; sleep 2
[ -n "$PGID" ] && kill -9 -- "-$PGID" 2>/dev/null
echo "残留：$(residue)"
python3 harness/vision_nav_params.py --restore 2>&1 | tee -a "$OUT/harness.log"
say "残留：gzserver=$(pgrep -cx gzserver)  产物 $OUT"
exit "$RC"
