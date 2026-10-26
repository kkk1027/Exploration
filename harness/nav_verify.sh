#!/usr/bin/env bash
# nav_verify.sh —— 只拉【Gazebo + 机器人 + 车端 nav2 栈(auto_explore=false) + RViz + 计划体检】
# 不跑探索建图（用户 2026-10-26 指示：代价地图/规划问题与探索关系不大，别为它重跑整轮）。
#
# 用法：
#   bash harness/nav_verify.sh                      # 用当前调优参数（膨胀/惩罚 + vx_max）
#   COST_INFL=0 bash harness/nav_verify.sh          # 不改代价地图参数（对照）
#   VISION=0   bash harness/nav_verify.sh           # 不起视觉障碍层（纯雷达）
#   PASS_W=1.05 GAZEBO_GUI=true RVIZ=true bash harness/nav_verify.sh
#
# 进入后你可以：
#   1) RViz 里用 "2D Goal Pose" 随手给一个目标（建议给"明显有宽路可选"的点）；
#   2) 看终端 nav_plan_audit 打印：`[plan#N] … 最小净宽 X m … → ✗ 穿窄处/✓ 通过`；
#   3) RViz 里 /plan_audit/markers：红点=计划压过的窄处，黄/橙球=最窄点；
#   4) 想让规划器换一条路，就把目标稍微挪一点再给一次，比较两次的最小净宽。
set +u
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy
WS=/home/kbf/stemm_exploration_sim_ws
if [ "${SIM_REMOTE:-0}" != "1" ]; then export ROS_LOCALHOST_ONLY=1; fi
source /home/kbf/stemm_sim_ws/domain_guard.sh || exit 9
source /opt/ros/humble/setup.bash
source /home/kbf/stemm_sim_ws/install/setup.bash
source /home/kbf/stemm_explore_ws/install/setup.bash
source "$WS/install/setup.bash"
# 遗留 daemon 会让 ros2 CLI 看不见回环节点（详见 docs/P9 第 3 节）
timeout 10 ros2 daemon stop >/dev/null 2>&1 || true

WORLD="${WORLD:-/home/kbf/stemm_sim_ws/install/wheeltec_two_wheel_model/share/wheeltec_two_wheel_model/world/aws_small_house.world}"
PASS_W="${PASS_W:-1.05}"
COST_INFL="${COST_INFL:-1}"
VISION="${VISION:-1}"
GAZEBO_GUI="${GAZEBO_GUI:-true}"
RVIZ="${RVIZ:-true}"
LOG="$WS/logs/nav_verify_$(date +%H%M%S)"
mkdir -p "$LOG"

cleanup() {
  python3 "$WS/harness/nav_params.py" --restore >/dev/null 2>&1
  python3 "$WS/harness/vision_nav_params.py" --restore >/dev/null 2>&1
  echo "[nav_verify] 已撤销全部注入（膨胀/速度/视觉层/规划器）"
}
trap cleanup EXIT INT TERM

# 一次性生成整份 nav2 参数（唯一注入器，逐字节可回退）
NAV_ARGS=(--vx-max "${VX_MAX:-0.35}")
[ -n "${VX_MIN:-}" ] && NAV_ARGS+=(--vx-min "$VX_MIN")
if [ "${COST_INFL:-1}" = "1" ]; then
  NAV_ARGS+=(--infl-radius "${GLOBE_R:-1.00}" --infl-scaling "${GLOBE_S:-1.2}" --pad "${PAD:-0.10}")
fi
if [ "${PLANNER_PATCH:-1}" = "1" ]; then
  NAV_ARGS+=(--allow-unknown "${ALLOW_UNKNOWN:-false}" --motion "${MOTION_MODEL:-REEDS_SHEPP}"
             --reverse-penalty "${REV_PENALTY:-3.0}")
fi
python3 harness/nav_params.py --apply "${NAV_ARGS[@]}" || exit 4
if [ "$VISION" = "1" ]; then
  python3 harness/vision_nav_params.py --apply || exit 4
fi
# 视觉累积格衰减（0=不衰减）
VDEC="${VISION_DECAY:-20.0}"

echo "[nav_verify] 世界=$WORLD  判据 pass_width=${PASS_W} m  视觉层=$VISION  膨胀注入=$COST_INFL  计划否决=${PLAN_VETO:-true}"
echo "[nav_verify] 日志目录：$LOG"
ros2 launch stemm_cartographer_exploration_sim nav_verify.launch.py \
  world:="$WORLD" gazebo_gui:="$GAZEBO_GUI" rviz:="$RVIZ" \
  pass_width:="$(awk -v v="$PASS_W" 'BEGIN{printf "%.4f", v}')" plan_veto:="${PLAN_VETO:-true}" \
  plan_audit_log:="$LOG/plan_audit.jsonl" 2>&1 | tee "$LOG/launch.log"
