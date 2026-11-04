#!/usr/bin/env bash
# stack_kill.sh —— 干净地杀掉整套仿真栈（含 setsid 出来的 ros2 launch 及其子节点）。
#
# 为什么单列一个脚本：2026-10-26 手杀 gzserver 时，`ros2 launch`（setsid 后另起进程组）
# 和它 24 个子节点还活着 —— 结果 cartographer 没了 /scan 与 /clock，TF 停止刷新，
# RViz 里就表现为"TF 断了"。这里按"可执行路径/进程名"枚举全部 PID 再杀，
# 不依赖进程组，也不会误杀调用者（显式排除 $$ 与 $PPID）。
set -u
MODE="${1:-term}"        # term=先 TERM 再 KILL；kill=直接 KILL

# ★★ 2026-10-28 域隔离（P0 误伤修复）：
#   本机同时存在**域 0 的真实机器人栈**（/opt/ros/humble/lib/nav2_*、ydlidar、
#   fishbot_bringup 等）。原脚本按可执行路径枚举，会把它们一起杀掉。
#   现在只处理 environ 里 ROS_DOMAIN_ID=42（仿真域）的进程 —— 与本仓库
#   domain_guard.sh 的口径一致（生产车=0，仿真=42）。
SIM_DOMAIN_EXPECT="${SIM_DOMAIN_EXPECT:-42}"
is_ours() {
  local pid="$1"
  [ -r "/proc/$pid/environ" ] || return 1
  tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -qx "ROS_DOMAIN_ID=${SIM_DOMAIN_EXPECT}"
}
list_pids() {
  {
    pgrep -f 'stemm_sim_ws/install'            2>/dev/null
    pgrep -f 'stemm_explore_ws/install'        2>/dev/null
    pgrep -f 'stemm_exploration_sim_ws/install' 2>/dev/null
    pgrep -f '/opt/ros/humble/lib/nav2_'       2>/dev/null
    pgrep -f 'cartographer_'                   2>/dev/null
    pgrep -f 'robot_state_publisher'           2>/dev/null
    pgrep -f 'robot_localization/ekf_node'     2>/dev/null
    pgrep -f 'sim_clock_resampler'             2>/dev/null
    pgrep -x gzserver                          2>/dev/null
    pgrep -x gzclient                          2>/dev/null
    pgrep -x rviz2                             2>/dev/null
    pgrep -f 'ros2 launch stemm_cartographer_exploration_sim' 2>/dev/null
  } | grep -vwE "$$|$PPID" | sort -u | while read -r p; do
      is_ours "$p" && echo "$p"
  done
}
PIDS=$(list_pids)
if [ -z "$PIDS" ]; then echo "[stack_kill] 没有需要清理的进程"; exit 0; fi
echo "[stack_kill] 目标 $(echo "$PIDS" | wc -w) 个进程"
if [ "$MODE" = "term" ]; then
  for p in $PIDS; do kill -TERM "$p" 2>/dev/null; done
  sleep 4
fi
for p in $(list_pids); do kill -9 "$p" 2>/dev/null; done
sleep 2
LEFT=$(list_pids | wc -w)
echo "[stack_kill] 剩余 $LEFT 个"
[ "$LEFT" -eq 0 ] || { echo "[stack_kill] ✗ 仍有残留，请手动检查"; exit 1; }
exit 0
