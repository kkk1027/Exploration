#!/usr/bin/env bash
# p13_ab.sh —— P13 候选派发风险评分 + 脱困放宽 的 A/B 跑批（交错执行）
#
# ★ 本脚本会调用 harness/round.sh，而 round.sh 内部要 source ROS 环境。
#   请在你自己的终端里执行（本仓库的 AGENTS.md 禁止智能体执行 source/colcon）。
#
# 用法:
#   bash harness/p13_ab.sh [轮数=5] [标签前缀=p13ab] [探索秒数=180.0]
#
# 产物:
#   logs/round_<ts>_<prefix>_a<i>   基线组（P13 全关）
#   logs/round_<ts>_<prefix>_b<i>   实验组（P13 全开）
#   runs/…                          round.sh 的六项验收摘要
#   最后自动运行 p13_ab_report.py 打印对比结论
#
# 设计要点:
#   * 交错执行（a1 b1 a2 b2 …）而不是先跑完 A 再跑 B —— 避免机器负载/室温/缓存
#     随时间漂移把差异记到分组头上（历史 ab6 就吃过这个亏）。
#   * 两组都强制 VETO_INPLACE=0：inplace 会让 nav2 GoalUpdater 跨目标污染 {goal}，
#     产生"30 ms 假到达"（详见 docs/P13_基线.md §2.1、BT 文件里的 P13-5 说明）。
#   * 每组每轮都落 candidate_probe / timeline / merge，报告脚本才有数据。

set -u
N="${1:-5}"
PREFIX="${2:-p13ab}"
EXPLORE="${3:-180.0}"
BASE=/home/kbf/stemm_exploration_sim_ws

echo "=============================================="
echo " P13 A/B 跑批：每组 ${N} 轮，交错执行"
echo "   基线组 A: RISK=0 RAW_SCORE=0 ESC_ALLOW_NARROW=0 ROUTE_GATE_SOFT=0 VERDICT=0"
echo "   实验组 B: RISK=1 RAW_SCORE=1 ESC_ALLOW_NARROW=1 ROUTE_GATE_SOFT=1 VERDICT=1"
echo "   两组都: VETO_INPLACE=0（避免幻影成功污染）"
echo "=============================================="

run_one () {           # $1=组名(a/b)  $2=轮号
  local grp="$1" i="$2"
  local tag="${PREFIX}_${grp}${i}"
  echo
  echo "-------- [${tag}] $(date '+%H:%M:%S') --------"
  if [ "$grp" = "a" ]; then
    RISK=0 RAW_SCORE=0 ESC_ALLOW_NARROW=0 ROUTE_GATE_SOFT=0 VERDICT=0 \
      VETO_INPLACE=0 \
      bash "$BASE/harness/round.sh" "$tag" "$EXPLORE"
  else
    RISK=1 RAW_SCORE=1 ESC_ALLOW_NARROW=1 ROUTE_GATE_SOFT=1 VERDICT=1 \
      VETO_INPLACE=0 \
      bash "$BASE/harness/round.sh" "$tag" "$EXPLORE"
  fi
  local rc=$?
  if [ $rc -ne 0 ]; then
    echo "⚠ [${tag}] round.sh 返回 $rc（并发锁/预检失败？）——记下但继续"
  fi
  return 0
}

for i in $(seq 1 "$N"); do
  run_one a "$i"
  run_one b "$i"
done

echo
echo "=============================================="
echo " 跑批完成，生成对比报告"
echo "=============================================="
cd "$BASE" || exit 1
python3 harness/p13_ab_report.py \
  "logs/round_*_${PREFIX}_a*" "logs/round_*_${PREFIX}_b*" \
  --json "/tmp/${PREFIX}_ab.json"
echo
echo "明细可用："
echo "  python3 harness/analyze_dispatch_risk.py 'logs/round_*_${PREFIX}_a*' -v"
echo "  python3 harness/analyze_dispatch_risk.py 'logs/round_*_${PREFIX}_b*' -v"
