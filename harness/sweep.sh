#!/usr/bin/env bash
# sweep.sh —— 连续稳定性验收：逐轮跑 round.sh + evaluate.py，任一轮失败即停并记录。
# 用法: sweep.sh <总轮数> <120s轮数> <180s轮数>   例: sweep.sh 6 4 2
set +u
W=/home/kbf/stemm_exploration_sim_ws
N=${1:-6}; N120=${2:-4}; N180=${3:-2}
SUM="$W/runs/sweep-$(date +%Y%m%d-%H%M%S).txt"
echo "sweep 开始 $(date -Iseconds)  共 $N 轮（120s×$N120 + 180s×$N180）" | tee "$SUM"
PASS=0
for i in $(seq 1 "$N"); do
  if [ "$i" -le "$N120" ]; then SEC=120.0; else SEC=180.0; fi
  TAG="sweep${i}"
  echo "--- 第 $i 轮（explore=${SEC}s）---" | tee -a "$SUM"
  bash "$W/harness/round.sh" "$TAG" "$SEC" > "/tmp/${TAG}.out" 2>&1
  D=$(ls -td "$W"/logs/round_*_"$TAG" | head -1)
  POSE=$(grep -o "本轮随机起点.*" "$D/harness.log" 2>/dev/null | head -1)
  RES=$(python3 "$W/harness/evaluate.py" "$D/launch.log" "$TAG" 2>/dev/null)
  V=$(printf '%s' "$RES" | grep -o '"verdict": "[A-Z]*"' | head -1 | cut -d'"' -f4)
  echo "$POSE → $V" | tee -a "$SUM"
  if [ "$V" = "PASS" ]; then PASS=$((PASS+1)); else
    echo "!!! 第 $i 轮失败，停止 sweep（连续计数 $PASS）" | tee -a "$SUM"
    echo "日志: $D/launch.log" | tee -a "$SUM"
    break
  fi
done
echo "sweep 结束 $(date -Iseconds)  连续通过 $PASS 轮" | tee -a "$SUM"
