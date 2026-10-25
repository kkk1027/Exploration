#!/usr/bin/env bash
# run_batch_range.sh <tag前缀> <起始> <结束> [ENV=VAL ...]  —— 续跑指定区间，进度追加
set -u
cd /home/kbf/stemm_exploration_sim_ws || exit 9
PREFIX="$1"; A="$2"; B="$3"; shift 3
PROG="/tmp/${PREFIX}_progress.log"
for i in $(seq "$A" "$B"); do
  echo "START ${PREFIX}r$i $(date +%H:%M:%S)" >> "$PROG"
  env GRACE_SEC=22 VISION=1 POSE_OVERRIDE="0.0 0.0 0.0" "$@" \
      bash harness/round_retry.sh "${PREFIX}r$i" 120 2 > "/tmp/${PREFIX}r$i.log" 2>&1
  D=$(ls -td logs/round_*_"${PREFIX}r$i" 2>/dev/null | head -1)
  echo "DONE ${PREFIX}r$i $(date +%H:%M:%S) dir=$D" >> "$PROG"
done
echo "RANGE DONE $A-$B $(date +%H:%M:%S)" >> "$PROG"
