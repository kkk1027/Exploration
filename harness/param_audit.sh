#!/usr/bin/env bash
# param_audit.sh —— P0-乙：运行期参数是否真的落地（防"看起来正常但没设上"）
# 用法: param_audit.sh <输出json> [expect_backstep 1|0]
# 返回: 0=全部符合；1=有不符合项（会打印明细）
# 背景: 2026-10-24 patch_sim_time 的 ros2 param set 曾永久阻塞，倒车包络/footprint/use_sim_time
#       全都没生效，而日志看起来"没报错"。这里每轮结束后逐项 get 回来核对。
set +u
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy ROS_LOCALHOST_ONLY
source /opt/ros/humble/setup.bash >/dev/null 2>&1
OUT="${1:-/dev/null}"; BACKSTEP="${2:-1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
M=/stemm_cartographer_nav2_manager
FP='[[-0.18,-0.235],[-0.18,0.235],[0.32,0.235],[0.32,-0.235]]'
BAD=0; N=0; DETAIL=""

get() { timeout 6 ros2 param get "$1" "$2" 2>/dev/null | sed 's/^[A-Za-z ]*is: //'; }

chk() {  # $1=node $2=param $3=期望（子串匹配）$4=标签
  local v; v=$(get "$1" "$2"); N=$((N+1))
  if printf '%s' "$v" | grep -qiF "$3"; then
    DETAIL="$DETAIL{\"param\":\"$2\",\"ok\":true,\"want\":\"$3\",\"got\":\"$v\"},"
  else
    BAD=$((BAD+1)); DETAIL="$DETAIL{\"param\":\"$2\",\"ok\":false,\"want\":\"$3\",\"got\":\"$v\"},"
    echo "  [audit] ✗ $2 期望含 '$3'，实际 '$v'"
  fi
}

# use_sim_time（5 个节点）
for n in "$M" /stemm_cartographer_mapping_tf_guard /stemm_cartographer_state_saver /stemm_cartographer_map_service_adapter /stemm_cartographer_odom_session_guard; do
  chk "$n" use_sim_time True
done
# 足迹（真机含轮宽）
chk /local_costmap/local_costmap footprint "$FP"
chk /global_costmap/global_costmap footprint "$FP"
# 后退相关
chk "$M" return_home_retreat_enabled False
if [ "$BACKSTEP" = "1" ]; then
  chk "$M" exploration_retreat_enabled True
  chk "$M" exploration_retreat_max_distance_m 0.15
  chk "$M" exploration_retreat_speed_mps 0.05
  chk "$M" blocked_retreat_max_per_session 2
else
  chk "$M" exploration_retreat_enabled False
fi

ok=$([ "$BAD" -eq 0 ] && echo true || echo false)
echo "  [audit] 参数落地自检：$((N-BAD))/$N 通过（backstep=$BACKSTEP）"
if [ "$OUT" != "/dev/null" ]; then
  printf '{"check":"param_audit","ok":%s,"pass":%d,"total":%d,"items":[%s]}\n' \
    "$ok" "$((N-BAD))" "$N" "${DETAIL%,}" > "$OUT"
fi
[ "$BAD" -eq 0 ] && exit 0 || exit 1
