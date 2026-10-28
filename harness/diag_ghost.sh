#!/usr/bin/env bash
# diag_ghost.sh <tag> <tf_stamp:true|false> [hold_sec]
#
# 视觉层"鬼影暂留"实验台：
#   起 nav_only（初始位姿 = 地图原点，硬性口径）→ 车静止采基线 →
#   原地转 360° → 停车观察 hold_sec 秒 → 量化孤立残留是否留下并滞留。
#
# 为什么要原地转：鬼影的成因是"用当前姿态去变换若干帧之前的点云"，
# 静止时不产生错位；旋转才有 ω·Δt 的系统性错位，多帧叠加=一圈漂移鬼影。
#
# 判据（看 logs/ghost_*/ghost.jsonl）：
#   marks       视觉累积层标记格总数
#   iso_frac    其中"距静态图最近占据格 > --iso-m"的比例（鬼影候选）
#   new         相对旋转前基线新增的格数
#   暂留的证据 = 旋转后 new 大幅上升且 hold 期间基本不回落。
set -u
cd /home/kbf/stemm_exploration_sim_ws || exit 9
TAG="${1:-ghost}"
TSV="${2:-true}"
HOLD="${3:-90}"
OUT="logs/ghost_$(date +%Y%m%d-%H%M%S)_${TAG}_stamp${TSV}"
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
flock -n 9 || { echo "[ghost] 已有轮次在跑"; exit 3; }
bash harness/preflight_dds.sh -q || { echo "[ghost] 环境不健康"; exit 4; }

cleanup() {
  echo "[ghost] 收尾"
  [ -n "${LPID:-}" ] && kill -TERM -- "-$LPID" 2>/dev/null
  sleep 3
  [ -n "${LPID:-}" ] && kill -KILL -- "-$LPID" 2>/dev/null
  bash harness/stack_kill.sh kill >/dev/null 2>&1
  python3 harness/vision_nav_params.py --restore >/dev/null 2>&1
  echo "[ghost] 收尾后 gz=$(pgrep -cx gzserver 2>/dev/null || true)"
}
trap cleanup EXIT INT TERM

# ★ 初始位姿固定在地图原点（用户口径 2026-10-28）
setsid ros2 launch stemm_cartographer_exploration_sim nav_only.launch.py \
  map_yaml:=/home/kbf/stemm_sim_ws/maps/explore_latest.yaml \
  spawn_x:=0.0 spawn_y:=0.0 spawn_yaw:=0.0 \
  vision:=true vision_mode:=accum vision_tf_stamp:=$TSV \
  vision_omega_gate:=${OMEGA_GATE:-0.0} vision_accum_decay_sec:=${ACCUM_DECAY:-120.0} \
  vision_low_accum_decay_sec:=${LOW_DECAY:-0.0} \
  gui:=false rviz:=false > "$OUT/launch.log" 2>&1 &
LPID=$!

for _ in $(seq 1 90); do
  st=$(timeout 8 ros2 lifecycle get /amcl 2>/dev/null | head -1)
  case "$st" in active*) break ;; esac
  sleep 2
done
for _ in $(seq 1 60); do
  timeout 6 ros2 topic hz /vision/obstacle_grid 2>/dev/null | grep -q 'average rate' && break
  sleep 2
done
sleep 8

echo "[ghost] 开始探测（tf_stamp=$TSV, hold=${HOLD}s, omega_gate=${OMEGA_GATE:-0.0}, accum_decay=${ACCUM_DECAY:-120.0}, low_decay=${LOW_DECAY:-0.0}）"
timeout 600 python3 harness/ghost_probe.py \
  --rotate-deg 360 --rate 1.2 --hold-sec "$HOLD" --period 10 --return-turn \
  --back-hold-sec "${BACK_HOLD:-40}" \
  --iso-m 0.30 --wait 90 --cmd-topic "${CMD_TOPIC:-/cmd_vel_nav}" --out "$OUT/ghost.jsonl" 2>&1 | tee "$OUT/probe.log"

sed 's/\x1b\[[0-9;]*m//g' "$OUT/launch.log" | grep -a '累积使能' | tail -3 | tee "$OUT/vision_stats.txt"
grep -ac 'tf_stamp_fallback\|tf_stamp_ok' "$OUT/launch.log" >/dev/null 2>&1 || true
echo "[ghost] 输出 $OUT"
