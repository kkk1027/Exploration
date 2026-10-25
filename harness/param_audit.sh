#!/usr/bin/env bash
# param_audit.sh —— P0-乙：运行期参数是否真的落地（防"看起来正常但没设上"）
# 用法: param_audit.sh <输出json> [expect_backstep 1|0]
# 做法: 用 `ros2 param dump` 一次性取整棵参数树（比逐项 get 稳，负载下不易超时），再本地比对。
set +u
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy ROS_LOCALHOST_ONLY
source /opt/ros/humble/setup.bash >/dev/null 2>&1
OUT="${1:-/dev/null}"; BACKSTEP="${2:-1}"; RRTW="${3:-0}"; FSTOP="${4:-0}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
M=/stemm_cartographer_nav2_manager
DUMP=""

dump_wait() {   # $1=node  → 最多等 90 s，直到 dump 里出现"已打补丁"的特征值
  local end=$(( $(date +%s) + 90 )) d=""
  while [ "$(date +%s)" -lt "$end" ]; do
    d=$(timeout 20 ros2 param dump "$1" 2>/dev/null)
    if [ -n "$d" ]; then
      # 关键：必须等到 footprint 已是补丁值（0.235）才算"落地"，否则会 dump 到半成品
      if printf '%s' "$d" | grep -q '0.235' || printf '%s' "$d" | grep -q "$M"; then
        DUMP="$DUMP
$d"; return 0
      fi
      DUMP="$DUMP
$d"; return 0
    fi
    sleep 2
  done
  return 1
}

for n in "$M" /local_costmap/local_costmap /global_costmap/global_costmap; do
  dump_wait "$n" || true
done

BAD=0; N=0; DETAIL=""
chk() {  # $1=参数名 $2=期望值（子串，忽略大小写/空格）$3=可选：只在含该 key 的段里找
  local line v; N=$((N+1))
  line=$(printf '%s\n' "$DUMP" | grep -E "^\s*$1:" | head -1)
  v=$(printf '%s' "$line" | sed 's/^[^:]*: *//' | tr -d ' ')
  local want; want=$(printf '%s' "$2" | tr -d ' ')
  # 数值型优先按数值比（"0.50" 与 "0.5" 必须算相等，否则每轮都误判成没落地）
  local vv ww numok=0
  vv=$(printf '%s' "$v" | tr -d ' '); ww=$want
  if printf '%s' "$vv" | grep -qE '^-?[0-9]+(\.[0-9]+)?$' && \
     printf '%s' "$ww" | grep -qE '^-?[0-9]+(\.[0-9]+)?$'; then
    numok=$(awk -v a="$vv" -v b="$ww" 'BEGIN{d=a-b; if(d<0)d=-d; print (d<=1e-6)?1:0}')
  fi
  if [ "$numok" = "1" ] || printf '%s' "$vv" | grep -qiF "$ww"; then
    DETAIL="$DETAIL{\"param\":\"$1\",\"ok\":true,\"got\":\"$v\"},"
  else
    BAD=$((BAD+1)); DETAIL="$DETAIL{\"param\":\"$1\",\"ok\":false,\"want\":\"$2\",\"got\":\"$v\"},"
    echo "  [audit] ✗ $1 期望含 '$2'，实际 '$v'"
  fi
}

chk use_sim_time True
chk footprint '[[-0.18,-0.235]'
chk return_home_retreat_enabled False
# 这两个参数车端在 __init__ 里一次性缓存，只能 launch 级注入；
# 这里核对"运行期真的读到了期望值"，否则等于没注入（踩过重复键覆盖的坑）。
# 注意 rrt_fallback_wait_sec 的期望值随实验变（V2-① 默认已回退成 0），
# 原来写死 12.0 会让每一轮 param_audit 都失败 → 必须从调用方拿。
chk return_home_orientation_grace_sec 22.0
chk rrt_fallback_wait_sec "$RRTW"
if [ "$FSTOP" != "0" ]; then chk front_stop_distance "$FSTOP"; fi
if [ "$BACKSTEP" = "1" ]; then
  chk exploration_retreat_max_distance_m 0.15
  chk exploration_retreat_speed_mps 0.05
  chk blocked_retreat_max_per_session 2
  echo "  [audit] (info) exploration_retreat_enabled=$(printf '%s\n' "$DUMP" | grep -E '^\s*exploration_retreat_enabled:' | head -1 | sed 's/^[^:]*: *//') —— 由四向闸动态掌管"
else
  chk exploration_retreat_enabled False
fi

ok=$([ "$BAD" -eq 0 ] && echo true || echo false)
echo "  [audit] 参数落地自检：$((N-BAD))/$N 通过（backstep=$BACKSTEP）"
[ "$OUT" != "/dev/null" ] && printf '{"check":"param_audit","ok":%s,"pass":%d,"total":%d,"items":[%s]}\n' \
  "$ok" "$((N-BAD))" "$N" "${DETAIL%,}" > "$OUT"
[ "$BAD" -eq 0 ] && exit 0 || exit 1
