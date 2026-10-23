#!/usr/bin/env bash
# stack_down.sh —— 收掉 stack_up.sh 起的栈，并撤销视觉参数补丁
set +u
D=$(cat /tmp/live_stack_dir 2>/dev/null)
PG=$(cat "$D/pgid" 2>/dev/null)
[ -n "$PG" ] && kill -TERM -- "-$PG" 2>/dev/null
for i in $(seq 1 20); do pgrep -x gzserver >/dev/null || break; sleep 1; done
pkill -x gzserver 2>/dev/null; sleep 2
[ -n "$PG" ] && kill -9 -- "-$PG" 2>/dev/null
cd /home/kbf/stemm_exploration_sim_ws
python3 harness/vision_nav_params.py --restore 2>&1 | tail -2
echo "残留: gzserver=$(pgrep -cx gzserver)"
