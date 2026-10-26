#!/usr/bin/env bash
# nav_sim.sh —— **不用 Gazebo** 的 nav2 测试台（用户 2026-10-26 指示）。
#
# 组成：fake_base（/cmd_vel 积分 + 离线地图 raycast 出 /scan）+ nav2（车上那份参数）
#       + map_server + nav_plan_audit（每条 /plan 量净宽，穿窄处可 veto）+ RViz
#
# 用法：
#   bash harness/nav_sim.sh                       # 默认地图/参数（调优后）
#   MAP=$HOME/stemm_sim_ws/maps/explore_latest.yaml START="1.0 5.1 -1.5708" bash harness/nav_sim.sh
#   NAV_ARGS_OFF=1 bash harness/nav_sim.sh        # 参数不注入（原厂对照）
#   VETO=false bash harness/nav_sim.sh            # 只报警不拦目标
set +u
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy
WS=/home/kbf/stemm_exploration_sim_ws
if [ "${SIM_REMOTE:-0}" != "1" ]; then export ROS_LOCALHOST_ONLY=1; fi
source /home/kbf/stemm_sim_ws/domain_guard.sh || exit 9
source /opt/ros/humble/setup.bash
source /home/kbf/stemm_explore_ws/install/setup.bash
source "$WS/install/setup.bash"
timeout 10 ros2 daemon stop >/dev/null 2>&1 || true

MAP="${MAP:-$HOME/stemm_sim_ws/maps/explore_latest.yaml}"
START="${START:-0.0 0.0 0.0}"
set -- $START
SX="$1"; SY="$2"; SYAW="$3"
LOG="$WS/logs/nav_sim_$(date +%H%M%S)"; mkdir -p "$LOG"
PASS_W="${PASS_W:-1.05}"

cleanup() {
  python3 "$WS/harness/nav_params.py" --restore >/dev/null 2>&1
  python3 "$WS/harness/vision_nav_params.py" --restore >/dev/null 2>&1
  echo "[nav_sim] 已撤销注入（nav2 参数/视觉层）"
}
trap cleanup EXIT INT TERM

if [ "${NAV_ARGS_OFF:-0}" != "1" ]; then
  ARGS=(--vx-max "${VX_MAX:-0.35}")
  [ -n "${VX_MIN:-}" ] && ARGS+=(--vx-min "$VX_MIN")
  if [ "${COST_INFL:-1}" = "1" ]; then
    ARGS+=(--infl-radius "${GLOBE_R:-1.00}" --infl-scaling "${GLOBE_S:-1.2}" --pad "${PAD:-0.10}")
  fi
  if [ "${PLANNER_PATCH:-1}" = "1" ]; then
    ARGS+=(--allow-unknown "${ALLOW_UNKNOWN:-false}" --motion "${MOTION_MODEL:-REEDS_SHEPP}"
           --reverse-penalty "${REV_PENALTY:-3.0}")
  fi
  python3 harness/nav_params.py --apply "${ARGS[@]}" || exit 4
else
  echo "[nav_sim] NAV_ARGS_OFF=1：nav2 参数保持原厂（对照）"
fi

echo "[nav_sim] 地图=$MAP  起点=($SX,$SY,$SYAW)  判据 pass_width=${PASS_W}  日志=$LOG"
ros2 launch stemm_cartographer_exploration_sim nav_sim.launch.py \
  map_yaml:="$MAP" \
  start_x:=$(awk -v v="$SX" 'BEGIN{printf "%.3f", v}') \
  start_y:=$(awk -v v="$SY" 'BEGIN{printf "%.3f", v}') \
  start_yaw:=$(awk -v v="$SYAW" 'BEGIN{printf "%.4f", v}') \
  pass_width:=$(awk -v v="$PASS_W" 'BEGIN{printf "%.4f", v}') \
  plan_veto:="${VETO:-true}" \
  plan_audit_log:="$LOG/plan_audit.jsonl" 2>&1 | tee "$LOG/launch.log"
