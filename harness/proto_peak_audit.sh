#!/usr/bin/env bash
# proto_peak_audit.sh <tag>
#
# 目的：把"虚假 lethal"的验证从**停稳时刻**扩到**旋转峰值时刻**，并同时覆盖
# **全局代价地图**（视觉累积层）。这是结项前最后一块证据缺口：
#   * 位姿滞后造成的错位标记只在旋转中产生 → 峰值时刻才可能看到鬼影；
#   * 用户的原始抱怨也包含全局"视觉累积栅格"。
#
# 做法（只读 + 只发 /cmd_vel）：
#   起栈（原始口径 + C1）→ 等 local_costmap active（硬闸门）
#   → 后台跑 local_residue_probe 连转 3 圈（提供旋转与停稳）
#   → 同时跑 costmap_lethal_audit（local 与 global 各一次，逐采样落盘 cells）
#   → 停稳后再各测一次
#   产物：cells_local_*.json / cells_global_*.json / audit_*.log / summary
#   后续用 GT 位图对**峰值那一帧**做逐格判定（见 classify 步骤）。
set -u
cd /home/kbf/stemm_exploration_sim_ws || exit 9
TAG="${1:-peak}"
OUT="logs/peak_$(date +%Y%m%d-%H%M%S)_${TAG}"
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
flock -n 9 || { echo "[peak] 已有轮次在跑"; exit 3; }
bash harness/preflight_dds.sh -q || { echo "[peak] 环境不健康"; exit 4; }

cleanup() {
  [ -n "${LPID:-}" ] && kill -TERM -- "-$LPID" 2>/dev/null
  sleep 3
  [ -n "${LPID:-}" ] && kill -KILL -- "-$LPID" 2>/dev/null
  bash harness/stack_kill.sh kill >/dev/null 2>&1
  python3 harness/vision_nav_params.py --restore >/dev/null 2>&1
  echo "[peak] 收尾 gz=$(pgrep -cx gzserver 2>/dev/null || true)"
}
trap cleanup EXIT INT TERM

setsid ros2 launch stemm_cartographer_exploration_sim nav_only.launch.py \
  map_yaml:=/home/kbf/stemm_sim_ws/maps/explore_latest.yaml \
  spawn_x:=0.0 spawn_y:=0.0 spawn_yaw:=0.0 \
  vision:=true vision_mode:=accum vision_tf_stamp:=true \
  gui:=false rviz:=false > "$OUT/launch.log" 2>&1 &
LPID=$!

LC_OK=0
for _ in $(seq 1 150); do
  st=$(timeout 8 ros2 lifecycle get /local_costmap/local_costmap 2>/dev/null | head -1)
  if [ "${st#active}" != "$st" ] && timeout 6 ros2 topic hz /local_costmap/costmap 2>/dev/null | grep -q 'average rate'; then
    LC_OK=1; break
  fi
  sleep 2
done
[ "$LC_OK" = "1" ] || { echo "[peak] ✗ local_costmap 未就绪（${st:-?}）"; exit 5; }
sleep 6

echo "[peak] 后台开始旋转（3 圈 + 停稳）"
timeout 900 python3 harness/local_residue_probe.py --turns 3 --settle 10 --hold 40 \
  --period 10 --base-n 2 --out "$OUT/residue.jsonl" > "$OUT/residue.log" 2>&1 &
ROT=$!

echo "[peak] 并行审计 local（每 3 s 一采样，逐采样落盘）"
timeout 300 python3 harness/costmap_lethal_audit.py --costmap local --samples 24 --period 3 \
  --out "$OUT/audit_local.jsonl" --dump-prefix "$OUT/cells_local" 2>&1 | tee "$OUT/audit_local.log"

wait $ROT 2>/dev/null

echo "[peak] 停稳后：global 审计"
timeout 200 python3 harness/costmap_lethal_audit.py --costmap global --samples 2 --period 5 \
  --out "$OUT/audit_global.jsonl" --dump-prefix "$OUT/cells_global" 2>&1 | tee "$OUT/audit_global.log"

echo "[peak] 输出 $OUT"
ls -1 "$OUT" | head -20
