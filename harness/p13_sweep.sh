#!/usr/bin/env bash
# p13_sweep.sh —— 多臂参数扫描（论文 E2 / E5 / E3 共用）
#
# 为什么需要它
# ------------
# p13_ab.sh 只能做两臂（a/b）。但 E2（force_after 扫描）需要 3~4 臂，
# E5（去掉饿死保护）其实是同一条轴上 force=∞ 的那个点，
# E3（换 world）又是另一条轴。与其为每个实验各写一份脚本，不如做成通用扫描。
#
# 用法
# ----
#   bash harness/p13_sweep.sh <prefix> <N> <explore_sec> <arm1> <arm2> ...
#
# 每个 arm 的格式：`标签:KEY=值,KEY=值,...`
#   * KEY 是传给 round.sh 的环境变量名（如 VERDICT_FORCE / WORLD / VERDICT）
#   * 标签只允许字母数字下划线，会成为 round 目录名的一部分
#
# 例（E2 + E5 合并：force_after 扫描，inf 那臂就是"去掉饿死保护"的消融）
#   bash harness/p13_sweep.sh p13e2 5 180.0 \
#        inf:VERDICT=1,VERDICT_FORCE=999999 \
#        f8:VERDICT=1,VERDICT_FORCE=8 \
#        f15:VERDICT=1,VERDICT_FORCE=15 \
#        f30:VERDICT=1,VERDICT_FORCE=30
#
# 例（E3：换 world 看泛化）
#   bash harness/p13_sweep.sh p13e3 10 180.0 \
#        house:VERDICT=1 \
#        open:VERDICT=1,WORLD=<另一个 world 路径>
#
# 执行顺序：**按轮号外层循环、按臂内层循环**（1 轮 a、1 轮 b、1 轮 c… 再进第 2 轮）。
# 这与 p13_ab.sh 的交错执行同理：让机器负载/温度漂移均摊到所有臂上，
# 否则"先跑完的臂"会系统性占优（历史 ab6 就吃过这个亏）。
#
# 产物：logs/round_<ts>_<prefix>_<标签><轮号>/
# 分析：analyze_efficiency.py / analyze_dispatch_risk.py 直接用 glob 匹配标签前缀。

set -u
PREFIX="${1:?用法: p13_sweep.sh <prefix> <N> <explore_sec> <arm1> [arm2 ...]}"
N="${2:?}"
EXPLORE="${3:?}"
shift 3
ARMS=("$@")
[ ${#ARMS[@]} -eq 0 ] && { echo "至少要给一个 arm" >&2; exit 2; }
BASE=/home/kbf/stemm_exploration_sim_ws

# 共同基线：已定型的最优配置（与 p13_ab.sh 的 verdict 模式一致）
COMMON='RISK=1 RAW_SCORE=1 ESC_ALLOW_NARROW=1 ROUTE_GATE_SOFT=1 VETO_INPLACE=0'

echo "=============================================="
echo " P13 参数扫描：prefix=${PREFIX}  每臂 ${N} 轮  探索 ${EXPLORE}s"
echo " 共同基线: ${COMMON}"
echo " 臂（${#ARMS[@]} 个）:"
for a in "${ARMS[@]}"; do
  echo "   $(echo "$a" | cut -d: -f1)  ⇐  $(echo "$a" | cut -d: -f2)"
done
echo " 执行顺序: 轮号在外、臂在内（漂移均摊）"
echo "=============================================="

run_arm () {           # $1=arm 定义  $2=轮号
  local arm="$1" i="$2"
  local label="${arm%%:*}"
  local spec="${arm#*:}"
  local tag="${PREFIX}_${label}${i}"
  # 把 KEY=值,KEY=值 变成 env 前缀
  local envs
  envs=$(echo "$spec" | tr ',' ' ')
  echo
  echo "-------- [${tag}] $(date '+%H:%M:%S')  ${envs} --------"
  # shellcheck disable=SC2086
  env $COMMON $envs bash "$BASE/harness/round.sh" "$tag" "$EXPLORE"
  local rc=$?
  if [ $rc -ne 0 ]; then
    echo "⚠ [${tag}] round.sh 返回 $rc —— 记下，收尾后用 p13_rerun.sh 补跑"
  fi
  return 0
}

for i in $(seq 1 "$N"); do
  for arm in "${ARMS[@]}"; do
    run_arm "$arm" "$i"
  done
done

echo
echo "=============================================="
echo " 扫描完成。各臂检查（timeline 行数）"
echo "=============================================="
cd "$BASE" || exit 1
for arm in "${ARMS[@]}"; do
  label="${arm%%:*}"
  bad=0
  for i in $(seq 1 "$N"); do
    # ★ glob 不能放双引号里 —— 引号内 bash 不展开通配符，
    #   `ls -d "logs/round_*_p13e6_e01"` 会因字面路径不存在而失败，
    #   于是每一轮都被误报成"缺失"（E6 收尾时实测踩到：
    #   20 轮全部有 450+ 行数据，却报 0/5 可用）。
    f=""
    for cand in logs/round_*_${PREFIX}_${label}${i}; do
      [ -d "$cand" ] && f="$cand"
    done
    if [ -z "$f" ] || [ ! -f "$f/timeline.jsonl" ]; then
      bad=$((bad+1))
    fi
  done
  printf "  %-10s %d/%d 轮可用\n" "$label" $((N-bad)) "$N"
  [ $bad -gt 0 ] && echo "             ↳ 建议补跑：$label 有 $bad 轮缺失"
done

echo
echo "分析命令模板："
for arm in "${ARMS[@]}"; do
  label="${arm%%:*}"
  echo "  python3 harness/analyze_dispatch_risk.py 'logs/round_*_${PREFIX}_${label}*'"
  echo "  python3 harness/analyze_efficiency.py    'logs/round_*_${PREFIX}_${label}*'"
done
