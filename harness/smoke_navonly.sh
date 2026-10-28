#!/usr/bin/env bash
# smoke_navonly.sh <tag> [秒数]  —— nav_only 冒烟（最小可信闭环检查）
#
# 设计原则（来自 2026-10-28 事故）：
#   * flock 单实例：绝不允许两个实例同时起栈（事故就是两个实例互相 pkill gzserver）
#   * 起栈前过 preflight 闸门：环境不干净就拒绝起栈，不许"重试掩盖根因"
#   * setsid 起 launch ⇒ 它自成进程组，收尾时按进程组 TERM/KILL，不留孤儿
#   * trap 兜底：脚本被 Ctrl-C / 被 kill / 超时，都走同一条清理路径
#   * 只报有证据的量：spawn 成功、相机/视觉话题频率、lifecycle 状态、接触计数
#
# 判据（全绿才算冒烟通过）：
#   1. launch.log 出现 "Successfully spawned entity"
#   2. /camera/depth/points 有数（hz 采样非空）
#   3. /vision/live_cloud 有数
#   4. amcl / controller_server 到达 active
#   5. collision_probe 接触计数 0（车不动）
set -u
cd /home/kbf/stemm_exploration_sim_ws || exit 9
TAG="${1:-navsmoke}"
SECS="${2:-90}"
MAP="${MAP:-/home/kbf/stemm_sim_ws/maps/explore_latest.yaml}"
# ★ 2026-10-28 用户口径：初始位姿必须在**地图原点**（地图由世界原点起图生成）
SX="${SX:-0.0}"; SY="${SY:-0.0}"; SYAW="${SYAW:-0.0}"
RVIZ_FLAG="${RVIZ_FLAG:-true}"

# ROS 环境（注意：setup.bash 有未绑定变量，必须在 set +u 下 source）
set +u
source /home/kbf/stemm_sim_ws/domain_guard.sh >/dev/null 2>&1
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source /home/kbf/stemm_sim_ws/install/setup.bash >/dev/null 2>&1
source /home/kbf/stemm_explore_ws/install/setup.bash >/dev/null 2>&1
source /home/kbf/stemm_exploration_sim_ws/install/setup.bash >/dev/null 2>&1
export ROS_LOCALHOST_ONLY=1
set -u
echo "[smoke] env DOMAIN=$ROS_DOMAIN_ID LOCALHOST=$ROS_LOCALHOST_ONLY"

exec 9>/tmp/stemm_round.lock
if ! flock -n 9; then echo "[smoke] ✗ 已有轮次在跑（/tmp/stemm_round.lock 被占）"; exit 3; fi

if ! bash harness/preflight_dds.sh -q; then
  echo "[smoke] ✗ 环境不健康，拒绝起栈（先 bash harness/stack_reset.sh）"
  exit 4
fi

OUT="logs/smoke_$(date +%Y%m%d-%H%M%S)_${TAG}"
mkdir -p "$OUT"
LOG="$OUT/launch.log"
echo "[smoke] 输出目录 $OUT"

cleanup() {
  echo "[smoke] 收尾：按进程组 TERM → KILL"
  [ -n "${LPID:-}" ] && kill -TERM -- "-$LPID" 2>/dev/null
  sleep 4
  [ -n "${LPID:-}" ] && kill -KILL -- "-$LPID" 2>/dev/null
  sleep 2
  bash harness/stack_kill.sh kill >/dev/null 2>&1
  python3 harness/vision_nav_params.py --restore >/dev/null 2>&1
  echo "[smoke] 收尾后 gzserver=$(pgrep -cx gzserver 2>/dev/null || true) 悬挂=$(for p in /proc/[0-9]*/maps; do grep -q 'fastrtps.*(deleted)' $p 2>/dev/null && echo x; done | wc -l)"
}
trap cleanup EXIT INT TERM

setsid ros2 launch stemm_cartographer_exploration_sim nav_only.launch.py \
  map_yaml:="$MAP" spawn_x:=$SX spawn_y:=$SY spawn_yaw:=$SYAW \
  vision:=true vision_mode:=accum vision_tf_stamp:=true \
  gui:=false rviz:=$RVIZ_FLAG plan_audit_log:="$OUT/plan_audit.jsonl" \
  contact_log:="$OUT/contacts.jsonl" > "$LOG" 2>&1 &
LPID=$!
echo "$LPID" > "$OUT/launch.pgid"
echo "[smoke] launch pid=$LPID 日志=$LOG"

# 等 spawn（最多 120 s）
for _ in $(seq 1 120); do
  grep -q 'Successfully spawned entity' "$LOG" 2>/dev/null && break
  kill -0 "$LPID" 2>/dev/null || { echo "[smoke] ✗ launch 已退出"; break; }
  sleep 1
done
if grep -q 'Successfully spawned entity' "$LOG"; then
  echo "[smoke] ✓ spawn 成功（$(grep -c 'Successfully spawned entity' "$LOG") 次）"
else
  echo "[smoke] ✗ spawn 未成功；日志尾部："
  grep -aE 'Waiting for entity|Waiting for master|Failed|Exception' "$LOG" | tail -5
fi

say() { echo "[smoke] $*"; }

# 等 nav2 生命周期
for _ in $(seq 1 90); do
  st=$(timeout 8 ros2 lifecycle get /amcl 2>/dev/null | head -1)
  case "$st" in active*) break ;; esac
  sleep 2
done
say "amcl=$(timeout 8 ros2 lifecycle get /amcl 2>/dev/null | head -1)"
say "controller_server=$(timeout 8 ros2 lifecycle get /controller_server 2>/dev/null | head -1)"
say "map_server=$(timeout 8 ros2 lifecycle get /map_server 2>/dev/null | head -1)"

# 话题频率（各采 8 s）
for t in /camera/depth/points /vision/live_cloud /vision/obstacle_grid /scan /clock; do
  r=$(timeout 10 ros2 topic hz "$t" 2>/dev/null | grep -m1 'average rate' || true)
  say "hz $t ⇒ ${r:-无数据}"
done

# ★ 2026-10-28：视觉层端到端闭环（视觉格是否真的进了全局代价地图）
say "--- vision_check（视觉障碍 vs 全局代价地图）---"
timeout 90 python3 harness/vision_check.py --timeout 60 --json "$OUT/vision_check.json" 2>&1 | tail -12 | tee "$OUT/vision_check.txt"
say "vision_check rc=$?"
sed 's/\x1b\[[0-9;]*m//g' "$LOG" | grep -a '累积使能' | tail -1 | tee -a "$OUT/vision_check.txt"

sleep $(( SECS > 0 ? SECS : 0 )) 2>/dev/null || true

# 汇总
{
  echo "== $TAG $(date '+%F %T') =="
  echo "spawn_ok=$(grep -c 'Successfully spawned entity' "$LOG")"
  echo "contacts=$( [ -f "$OUT/contacts.jsonl" ] && grep -c . "$OUT/contacts.jsonl" || echo 0)"
  grep -a 'vision_obstacle_grid.*累积\|已发布 robot_description' "$LOG" | tail -2
} | tee "$OUT/summary.txt"
echo "[smoke] DONE $OUT"
