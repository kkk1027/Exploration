#!/usr/bin/env bash
# round_retry.sh —— 单轮探索 + "起栈早失败"自动重试（P0-3）
# 用法: round_retry.sh <标签> <探索秒数> [最多尝试次数=2]
# 判定：起栈失败 = launch.log 里没有 ready_guard_timing，或出现 "did not become active/ready"
# 重试时把 STACK_DELAY 加 15 s（给 nav2 生命周期更多余量），并打上 _r2 标签。
set +u
W=/home/kbf/stemm_exploration_sim_ws
TAG="${1:-run}"; SECS="${2:-120}"; MAX="${3:-2}"
ATT=1
while :; do
  T="${TAG}"; [ "$ATT" -gt 1 ] && T="${TAG}_r${ATT}"
  echo "[retry] 第 ${ATT}/${MAX} 次：tag=${T} secs=${SECS} stack_delay=${STACK_DELAY:-65}"
  bash "$W/harness/round.sh" "$T" "$SECS" > "/tmp/${T}.out" 2>&1
  D=$(ls -td "$W"/logs/round_*_"$T" 2>/dev/null | head -1)
  FAILED_START=0
  if [ -n "$D" ]; then
    if ! grep -q "ready_guard_timing" "$D/launch.log" 2>/dev/null; then
      FAILED_START=1
    fi
    if grep -qE "Nav2 did not become active|did not become ready" "$D/launch.log" 2>/dev/null; then
      FAILED_START=1
    fi
  fi
  if [ "$FAILED_START" = "1" ] && [ "$ATT" -lt "$MAX" ]; then
    echo "[retry] 起栈失败 → 清理后重试（STACK_DELAY+15）"
    printf '{"check":"startup_fail","tag":"%s","attempt":%d}\n' "$T" "$ATT" >> "$W/runs/startup_fail.jsonl"
    PG=$(cat "$D/pgid" 2>/dev/null); [ -n "$PG" ] && kill -TERM -- "-$PG" 2>/dev/null
    sleep 3; pkill -x gzserver 2>/dev/null; sleep 3
    [ -n "$PG" ] && kill -9 -- "-$PG" 2>/dev/null
    STACK_DELAY=$(( ${STACK_DELAY:-65} + 15 ))
    ATT=$((ATT+1))
    continue
  fi
  [ "$FAILED_START" = "1" ] && echo "[retry] 起栈失败且已达重试上限"
  echo "[retry] 完成：$D"
  exit 0
done
