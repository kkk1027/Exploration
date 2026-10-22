#!/usr/bin/env bash
# test_escape.sh —— P0-3 脱困监督的单元测试（不需要车端整套栈 / Nav2）
#
# 做法：只起机器人 + collision_probe + escape_supervisor，再用 harness/fake_nav_status
# 喂"有导航目标在执行"的状态，把车停在墙前 0.5 m 处不动 —— 对监督节点来说这就是
# "有任务、已停住、旁边有障碍"，它应当接管：转到最空旷的方向 → 前进一小段。
#
# 用法：test_escape.sh <tag> <pose "x y yaw"> [额外 -p 参数 ...]
# 产物：logs/escape_<tag>/ 下的 probe.jsonl / supervisor.jsonl / launch.log
set +u
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy ROS_LOCALHOST_ONLY
source /home/kbf/stemm_sim_ws/domain_guard.sh || exit 9
source /opt/ros/humble/setup.bash
source /home/kbf/stemm_sim_ws/install/setup.bash
source /home/kbf/stemm_explore_ws/install/setup.bash
source /home/kbf/stemm_exploration_sim_ws/install/setup.bash
TAG="${1:-esc}"; POSE="${2:-0.0 4.0 1.5708}"; shift 2 || true
EXTRA="$*"
BASE=/home/kbf/stemm_exploration_sim_ws
OUT="$BASE/logs/escape_${TAG}"; rm -rf "$OUT"; mkdir -p "$OUT"
set -- $POSE; SX=$1; SY=$2; SYAW=$3
pkill -x gzserver 2>/dev/null; sleep 3
echo "[escape-test] tag=$TAG pose=($SX,$SY,$SYAW) extra=[$EXTRA]" | tee -a "$OUT/harness.log"
setsid bash -c "echo \$\$ > '$OUT/pgid'; exec ros2 launch stemm_cartographer_exploration_sim sim_explore.launch.py \
  auto_explore:=false stack:=false stack_delay:=0 rviz:=false \
  spawn_x:=$SX spawn_y:=$SY spawn_yaw:=$SYAW \
  probe_log:=$OUT/probe.jsonl supervisor_log:=$OUT/supervisor.jsonl" \
  > "$OUT/launch.log" 2>&1 < /dev/null &
sleep 45
# 可选用 PARAMS="k=v k2=v2" 在运行时改监督节点参数（做降级/超时这类机制验证）
if [ -n "${PARAMS:-}" ]; then
  for kv in $PARAMS; do
    k=${kv%%=*}; v=${kv#*=}
    ros2 param set /escape_supervisor "$k" "$v" >/dev/null 2>&1 \
      && echo "[escape-test] param $k=$v ✓" | tee -a "$OUT/harness.log"
  done
fi
python3 "$BASE/harness/fake_nav_status" > "$OUT/fake.log" 2>&1 &
FAKE=$!
sleep "${HOLD:-35}"
kill $FAKE 2>/dev/null
PGID=$(cat "$OUT/pgid" 2>/dev/null)
[ -n "$PGID" ] && kill -TERM -- "-$PGID" 2>/dev/null
sleep 6; pkill -x gzserver 2>/dev/null; sleep 2
echo "=== supervisor 事件 ==="
python3 - "$OUT/supervisor.jsonl" <<'PY'
import json, sys
try:
    rows = [json.loads(l) for l in open(sys.argv[1]) if l.strip()]
except FileNotFoundError:
    print('（无 supervisor.jsonl）'); raise SystemExit
for r in rows:
    keep = {k: v for k, v in r.items() if k not in ('nav', 'stats')}
    print(json.dumps(keep, ensure_ascii=False))
PY
echo "=== 车体最小间隙变化（probe）==="
python3 - "$OUT/probe.jsonl" <<'PY'
import json, sys
try:
    rows = [json.loads(l) for l in open(sys.argv[1]) if l.strip()]
except FileNotFoundError:
    print('（无 probe.jsonl）'); raise SystemExit
sm = [r for r in rows if r.get('type') == 'summary']
for r in sm:
    print(f"  t={r.get('t'):.0f} min_d={r.get('min_d_now')} ever={r.get('min_d_ever')} "
          f"pos={r.get('pos')} wheels={r.get('wheel_contacts')} chassis={r.get('chassis_contacts')}")
rev = [r for r in rows if r.get('type') == 'reverse' and r.get('phase') == 'end']
print(f"  反向指令窗口 = {len(rev)}（必须为 0）")
PY
echo "OUT=$OUT"
