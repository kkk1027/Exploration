#!/usr/bin/env bash
# diag_local_residue.sh <tag> [圈数]
# 局部代价地图残留实验台：nav_only（初始位姿=地图原点）→ 基线 → 连转 N 圈（每圈停 settle）
# → 转回原朝向 → 停 hold 秒，逐秒记录 raw_lethal / 膨胀面积 / 比值。
# 视觉口径：VISION_GATE / VISION_DECAY 可注入，用来确认"改动1/2 是否影响局部层"。
set -u
cd /home/kbf/stemm_exploration_sim_ws || exit 9
TAG="${1:-lr}"
TURNS="${2:-3}"
OUT="logs/localres_$(date +%Y%m%d-%H%M%S)_${TAG}"
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
flock -n 9 || { echo "[local] 已有轮次在跑"; exit 3; }
bash harness/preflight_dds.sh -q || { echo "[local] 环境不健康"; exit 4; }

cleanup() {
  [ -n "${LPID:-}" ] && kill -TERM -- "-$LPID" 2>/dev/null
  sleep 3
  [ -n "${LPID:-}" ] && kill -KILL -- "-$LPID" 2>/dev/null
  bash harness/stack_kill.sh kill >/dev/null 2>&1
  python3 harness/vision_nav_params.py --restore >/dev/null 2>&1
  echo "[local] 收尾 gz=$(pgrep -cx gzserver 2>/dev/null || true)"
}
trap cleanup EXIT INT TERM

setsid ros2 launch stemm_cartographer_exploration_sim nav_only.launch.py \
  map_yaml:=/home/kbf/stemm_sim_ws/maps/explore_latest.yaml \
  spawn_x:=0.0 spawn_y:=0.0 spawn_yaw:=0.0 \
  vision:=${VISION:-true} vision_mode:=accum vision_tf_stamp:=true \
  vision_omega_gate:=${VISION_GATE:-0.0} \
  vision_accum_decay_sec:=${VISION_DECAY:-120.0} \
  vision_low_accum_decay_sec:=${VISION_LOW_DECAY:-0.0} \
  gui:=false rviz:=false > "$OUT/launch.log" 2>&1 &
LPID=$!
for _ in $(seq 1 90); do
  st=$(timeout 8 ros2 lifecycle get /amcl 2>/dev/null | head -1)
  case "$st" in active*) break ;; esac
  sleep 2
done
for _ in $(seq 1 40); do
  timeout 6 ros2 topic hz /local_costmap/costmap 2>/dev/null | grep -q 'average rate' && break
  sleep 2
done
sleep 6
echo "[local] 局部代价地图关键参数："
for p in use_sim_time update_frequency rolling_window width height; do
  echo "  /local_costmap/local_costmap $p = $(timeout 8 ros2 param get /local_costmap/local_costmap $p 2>&1 | tail -1)"
done | tee "$OUT/params.txt"

timeout 900 python3 harness/local_residue_probe.py \
  --turns "$TURNS" --settle 10 --hold 40 --period 10 \
  --out "$OUT/residue.jsonl" 2>&1 | tee "$OUT/probe.log"

echo "=== nav2 视觉体素层的告警（若有）==="
sed 's/\x1b\[[0-9;]*m//g' "$OUT/launch.log" | grep -aiE 'vision_voxel|VoxelLayer|Clearing|obstacle_layer.*drop|Message Filter' | tail -8 | tee -a "$OUT/probe.log"
echo "[local] 输出 $OUT"
