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

# ---------------------------------------------------------------------------
# 0) 静态断言（S5 §5-E）：不需要起栈，< 1 秒。
#    放在最前面，这样"阈值已被改坏"能在花 4 分钟起栈之前就被挡住。
#    新加的 test_*.py 自动纳入，不用改这里。
# ---------------------------------------------------------------------------
HARNESS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "[smoke] 0) 静态断言"
for t in "$HARNESS_DIR"/test_robot_profile.py "$HARNESS_DIR"/test_no_magic_numbers.py \
         "$HARNESS_DIR"/test_sweep_kernel.py "$HARNESS_DIR"/test_bfs_equivalence.py; do
  [ -f "$t" ] || continue
  N=$(basename "$t" .py)
  if OUT=$(timeout 20 python3 "$t" 2>&1); then
    ok "$N（$(echo "$OUT" | tail -1)）"
  else
    bad "$N"
    echo "$OUT" | grep -E '^ *✗' | head -6 | sed 's/^/       /'
  fi
done


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
if python3 "$W/harness/test_arbiter_attrs.py" >/dev/null 2>&1; then ok "仲裁无幽灵属性（只读未声明）"; else bad "仲裁存在只读未声明的属性（运行到该分支必崩）"; fi
if python3 "$W/harness/test_arbiter_static.py" >/dev/null 2>&1; then ok "仲裁 path_and_width 签名一致"; else bad "仲裁返回值/调用点签名不一致（会直接崩）"; fi
if python3 "$W/harness/test_scoring.py" >/dev/null 2>&1; then ok "打分内核离线断言通过"; else bad "打分内核离线断言失败"; fi
if python3 "$W/harness/test_launch_dryrun.py" >/dev/null 2>&1; then ok "launch 可构造（干跑）"; else bad "launch 构造失败（import/类型错）"; fi
if python3 "$W/harness/test_fallback_obs.py" >/dev/null 2>&1; then ok "保底观察点逻辑（合成净空场）"; else bad "保底观察点逻辑异常（卡死那刻会崩）"; fi
if python3 "$W/harness/test_veto.py" >/dev/null 2>&1; then ok "后置 veto 判据（10 组约束）"; else bad "后置 veto 判据异常（可能误掐目标）"; fi
if python3 "$W/harness/test_param_types.py" >/dev/null 2>&1; then ok "launch→节点 参数类型一致"; else bad "launch 传参类型与节点 declare 不一致（节点会当场崩）"; fi
if python3 "$W/harness/test_tail_park.py" >/dev/null 2>&1; then ok "末段停车友好偏置（7 组边界）"; else bad "末段停车友好偏置异常"; fi
if python3 "$W/harness/test_standoff.py" >/dev/null 2>&1; then ok "返航前接管点选择（11 项断言）"; else bad "返航前接管点异常（可能把车送进死角）"; fi
if python3 "$W/harness/test_return_feasible.py" >/dev/null 2>&1; then ok "可达但不可返回/脱困目标（6 项断言）"; else bad "可达但不可返回判据异常（会把车派进出不来的地方）"; fi
if python3 "$W/harness/test_driver_hooks.py" >/dev/null 2>&1; then ok "关键接线检查（nav_state/兜底/接管点/veto）"; else bad "有功能没接上（写了但没生效）"; fi
if python3 "$W/harness/test_harness_setu.py" >/dev/null 2>&1; then ok "harness 脚本无 set -u × source 静默退出地雷"; else bad "有 harness 脚本在 set -u 下 source ROS setup（会静默退出、探针/日志全丢）"; fi

if [ "$FAIL" -eq 0 ]; then echo "[smoke] 结果：PASS"; exit 0; else echo "[smoke] 结果：FAIL（$FAIL 项）"; exit 1; fi
