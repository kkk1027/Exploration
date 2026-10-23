#!/usr/bin/env bash
# ab_vision.sh —— 方案A 的 A/B 对照：同一固定起点、交替跑 VISION=0 / VISION=1，各 N 轮
# 用法: harness/ab_vision.sh [轮数=3] [探索秒数=120]
# 交替而不是"先全跑A再全跑B"，用来抵消机器负载/室温/随机性漂移。
set +u
W=/home/kbf/stemm_exploration_sim_ws
N="${1:-3}"; SECS="${2:-120}"
SUM="$W/runs/ab-vision-$(date +%Y%m%d-%H%M%S).txt"
echo "A/B 开始 $(date -Iseconds)：VISION=0/1 各 $N 轮，探索 ${SECS}s，固定起点 (0,0,0)" | tee "$SUM"
for i in $(seq 1 "$N"); do
  for V in 0 1; do
    TAG="abv${V}r${i}"
    echo "--- 第 $i 组  VISION=$V ---" | tee -a "$SUM"
    POSE_OVERRIDE="0.0 0.0 0.0" VISION="$V" bash "$W/harness/round.sh" "$TAG" "$SECS" > "/tmp/${TAG}.out" 2>&1
    D=$(ls -td "$W"/logs/round_*_"$TAG" 2>/dev/null | head -1)
    V_TXT=$(python3 - "$D/evaluate.json" 2>/dev/null <<'PY'
import json,sys
try: d=json.load(open(sys.argv[1]))
except Exception: print('NA'); raise SystemExit
c=d.get('collision',{}); v=d.get('vision',{})
print('%s phys=%s wheel=%s chassis=%s cells=%s' % (
    d.get('verdict'), c.get('physical_contacts'), c.get('wheel_contacts'),
    c.get('chassis_contacts'), (v or {}).get('cells')))
PY
)
    MEM=$(grep -h '"check":"memory"' "$(dirname "$D")/../runs"/*"$TAG"*.jsonl 2>/dev/null | tail -1)
    MEM2=$(grep -h '"check":"memory"' "$W/runs"/*"$TAG"*.jsonl 2>/dev/null | tail -1)
    echo "  $V_TXT" | tee -a "$SUM"
    [ -n "$MEM2" ] && echo "  内存: $MEM2" | tee -a "$SUM"
  done
done
echo "A/B 结束 $(date -Iseconds)" | tee -a "$SUM"
