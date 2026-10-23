#!/usr/bin/env bash
# smoke_check.sh —— 起栈后 10 秒自检（放进 round.sh 的 SMOKE 模式，或单独调用）。
# 目的：在跑整轮之前就抓住"参数没落地 / 节点没起来 / 话题没数据 / TF guard 超时"这类白跑。
# 用法: smoke_check.sh <launch.log>
set +u
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source /home/kbf/stemm_sim_ws/install/setup.bash >/dev/null 2>&1
source /home/kbf/stemm_explore_ws/install/setup.bash >/dev/null 2>&1
source /home/kbf/stemm_exploration_sim_ws/install/setup.bash >/dev/null 2>&1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
LOG="${1:-/dev/null}"
FAIL=0
ok()   { echo "  ✓ $*"; }
bad()  { echo "  ✗ $*"; FAIL=$((FAIL+1)); }

echo "[smoke] 1) 关键节点"
NODES=$(timeout 20 ros2 node list 2>/dev/null)
for n in /stemm_cartographer_nav2_manager /controller_server /planner_server /bt_navigator \
         /stemm_cartographer_node /frontier_arbiter \
         /collision_probe /escape_supervisor /sim_session_driver; do
  case "$NODES" in *"$n"*) ok "$n" ;; *) bad "$n 缺失" ;; esac
done

echo "[smoke] 2) 关键参数是否真落地"
pget() { timeout 12 ros2 param get "$1" "$2" 2>/dev/null | tail -1; }
V=$(pget /stemm_cartographer_nav2_manager exploration_retreat_enabled)
case "$V" in *False*) ok "exploration_retreat_enabled=False" ;; *) bad "exploration_retreat_enabled=$V" ;; esac
V=$(pget /stemm_cartographer_nav2_manager frontier_goal_topic)
case "$V" in *validated_frontier_goal*) ok "frontier_goal_topic=$V" ;; *) bad "frontier_goal_topic=$V" ;; esac
# tf_guard 通过校验后会自行退出，故用日志证据判断（节点可能已经不在）
if [ -f "$LOG" ] && grep -q "mapping TF is healthy" "$LOG"; then
  ok "mapping tf_guard 健康检查通过（日志）"
else
  bad "未见 'mapping TF is healthy'"
fi
V=$(pget /local_costmap/local_costmap footprint)
HALF=$(echo "$V" | python3 -c "
import re,sys
nums=[float(x) for x in re.findall(r'-?[0-9]+\.[0-9]+', sys.stdin.read())]
print(max(abs(n) for n in nums[1::2]) if len(nums)>2 else 0)
" 2>/dev/null)
if python3 -c "import sys;sys.exit(0 if abs(float('${HALF:-0}'))>=0.230 else 1)" 2>/dev/null; then
  ok "footprint 半宽=$HALF（>=0.230）"
else
  bad "footprint 半宽=$HALF（车端原值 0.20，P0 补丁没生效？）"
fi

echo "[smoke] 3) 关键话题有数据"
for t in /scan_raw /odom_combined /map /collision/min_clearance; do
  if timeout 10 ros2 topic echo "$t" --once >/dev/null 2>&1; then ok "$t 有数据"; else bad "$t 无数据"; fi
done

echo "[smoke] 4) 探索环是否真的转起来"
if [ -f "$LOG" ]; then
  if grep -q "navigation goal dispatched" "$LOG"; then
    ok "首次探索目标已派发"
  else
    bad "起栈后 50 s 内没有派发任何探索目标（探索环没转起来）"
  fi
  if grep -q "候选打分前\|仲裁通过" "$LOG"; then ok "仲裁在工作（有打分/派发记录）"; else bad "仲裁没有任何打分/派发记录"; fi
else
  echo "  (未提供 launch.log，跳过)"
fi

echo "[smoke] 5) 日志无致命错误"
if [ -f "$LOG" ]; then
  if grep -q "TF health check timed out" "$LOG"; then bad "mapping_tf_guard 超时"; else ok "无 TF guard 超时"; fi
  N=$(grep -c "process has died" "$LOG" 2>/dev/null | head -1)
  N=${N:-0}
  if [ "${N:-0}" -gt 0 ]; then bad "$N 个进程死亡"; else ok "无进程死亡"; fi
else
  echo "  (未提供 launch.log，跳过)"
fi

echo "[smoke] 6) 离线不变量（打分/仲裁签名，<1 秒）"
W=/home/kbf/stemm_exploration_sim_ws
if python3 "$W/harness/test_arbiter_static.py" >/dev/null 2>&1; then ok "仲裁 path_and_width 签名一致"; else bad "仲裁返回值/调用点签名不一致（会直接崩）"; fi
if python3 "$W/harness/test_scoring.py" >/dev/null 2>&1; then ok "打分内核离线断言通过"; else bad "打分内核离线断言失败"; fi
if python3 "$W/harness/test_launch_dryrun.py" >/dev/null 2>&1; then ok "launch 可构造（干跑）"; else bad "launch 构造失败（import/类型错）"; fi

if [ "$FAIL" -eq 0 ]; then echo "[smoke] 结果：PASS"; exit 0; else echo "[smoke] 结果：FAIL（$FAIL 项）"; exit 1; fi
