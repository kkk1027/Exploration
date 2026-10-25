#!/usr/bin/env bash
# run_batch.sh <tag前缀> <轮数> [ENV=VAL ...]
# 用法示例：bash harness/batches/run_batch.sh mc2 10 MAX_CAND=24
# 产物：进度写到 /tmp/<前缀>_progress.log；每轮日志 /tmp/<前缀>r<i>.log
set -u
cd /home/kbf/stemm_exploration_sim_ws || exit 9
PREFIX="$1"; N="$2"; shift 2
PROG="/tmp/${PREFIX}_progress.log"
: > "$PROG"
for i in $(seq 1 "$N"); do
  echo "START ${PREFIX}r$i $(date +%H:%M:%S)" >> "$PROG"
  env GRACE_SEC=22 VISION=1 POSE_OVERRIDE="0.0 0.0 0.0" "$@" \
      bash harness/round_retry.sh "${PREFIX}r$i" 120 2 > "/tmp/${PREFIX}r$i.log" 2>&1
  D=$(ls -td logs/round_*_"${PREFIX}r$i" 2>/dev/null | head -1)
  echo "DONE ${PREFIX}r$i $(date +%H:%M:%S) dir=$D" >> "$PROG"
done
echo "ALL DONE $(date +%H:%M:%S)" >> "$PROG"
