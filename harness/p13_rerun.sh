#!/usr/bin/env bash
# p13_rerun.sh —— 补跑 E1~E5 里因基础设施问题（sensor preflight 超时 /
# gzserver 早退）而没产出可用数据的轮次。
#
# 为什么需要它
# ------------
# 实测（p13e1_a1，2026-11-10 13:10）：launch 后 41 s 内
#   [stemm_cartographer_sensor_preflight]: sensor preflight timed out after 8.0s
#   ERROR: Core sensor preflight failed.
# 仿真随即整体关停，该轮**没有任何 timeline.jsonl / 派发记录**。
# 这与本仓库的算法改动无关，是启动竞态（传感器首发延迟 > 8 s）。
#
# 判据（"可用"的定义）
#   logs/round_*_<prefix>_<tag>/timeline.jsonl 存在且行数 >= MIN_LINES
# 只补不可用的轮次，不重跑已成功的轮次（避免白白多花机时）。
#
# 用法：
#   bash harness/p13_rerun.sh <prefix> <variant> <轮数> [探索秒数=180.0]
#   bash harness/p13_rerun.sh p13e1 verdict 10
#
# 注意：本脚本与 p13_ab.sh 一样会调用 round.sh（内部 source ROS 环境），
#       请在你自己的终端里执行。它使用 /tmp/stemm_round.lock，不会与
#       正在跑的批次并发冲突（会等待锁）。

set -u
PREFIX="${1:?用法: p13_rerun.sh <prefix> <variant> <N> [explore_sec]}"
VARIANT="${2:-verdict}"
N="${3:-10}"
EXPLORE="${4:-180.0}"
BASE=/home/kbf/stemm_exploration_sim_ws
MIN_LINES="${MIN_LINES:-100}"

cd "$BASE" || exit 1

ok_count () {          # $1=tag
  local f
  f=$(ls -d "logs/round_*_${PREFIX}_$1" 2>/dev/null | head -1)
  [ -z "$f" ] && { echo 0; return; }
  if [ -f "$f/timeline.jsonl" ]; then
    wc -l < "$f/timeline.jsonl"
  else
    echo 0
  fi
}

run_round () {         # $1=组名  $2=轮号
  local grp="$1" i="$2"
  local tag="${PREFIX}_${grp}${i}"
  echo
  echo "-------- [补跑 ${tag}] $(date '+%H:%M:%S') --------"
  if [ "$VARIANT" = "verdict" ]; then
    if [ "$grp" = "a" ]; then local V=0; else local V=1; fi
    RISK=1 RAW_SCORE=1 ESC_ALLOW_NARROW=1 ROUTE_GATE_SOFT=1 VERDICT="$V" \
      VETO_INPLACE=0 \
      bash "$BASE/harness/round.sh" "$tag" "$EXPLORE"
  elif [ "$grp" = "a" ]; then
    RISK=0 RAW_SCORE=0 ESC_ALLOW_NARROW=0 ROUTE_GATE_SOFT=0 VERDICT=0 \
      VETO_INPLACE=0 \
      bash "$BASE/harness/round.sh" "$tag" "$EXPLORE"
  else
    RISK=1 RAW_SCORE=1 ESC_ALLOW_NARROW=1 ROUTE_GATE_SOFT=1 VERDICT=1 \
      VETO_INPLACE=0 \
      bash "$BASE/harness/round.sh" "$tag" "$EXPLORE"
  fi
}

echo "=============================================="
echo " 补跑检查：prefix=${PREFIX} variant=${VARIANT} N=${N}"
echo " 判据：timeline.jsonl 行数 >= ${MIN_LINES}"
echo "=============================================="

need=()
for i in $(seq 1 "$N"); do
  for grp in a b; do
    tag="${grp}${i}"
    n=$(ok_count "$tag")
    if [ "$n" -lt "$MIN_LINES" ]; then
      echo "  ✗ ${PREFIX}_${tag}  timeline=${n} 行 → 需补跑"
      need+=("${grp}:${i}")
    else
      echo "  ✓ ${PREFIX}_${tag}  timeline=${n} 行"
    fi
  done
done

if [ ${#need[@]} -eq 0 ]; then
  echo
  echo "全部 ${N}×2 轮均可用，无需补跑。"
  exit 0
fi

echo
echo "需要补跑 ${#need[@]} 轮：${need[*]}"
echo "（若同时还有批次在跑，round.sh 的 flock 会阻塞等待，不会冲突）"
echo

for item in "${need[@]}"; do
  run_round "${item%%:*}" "${item##*:}"
done

echo
echo "=============================================="
echo " 补跑后复核"
echo "=============================================="
bad=0
for i in $(seq 1 "$N"); do
  for grp in a b; do
    tag="${grp}${i}"
    n=$(ok_count "$tag")
    if [ "$n" -lt "$MIN_LINES" ]; then
      echo "  ✗ ${PREFIX}_${tag} 仍不可用（timeline=${n}）"
      bad=$((bad+1))
    fi
  done
done
[ $bad -eq 0 ] && echo "  ✓ 全部可用" || echo "  ⚠ 仍有 ${bad} 轮不可用"
exit $([ $bad -eq 0 ] && echo 0 || echo 1)
