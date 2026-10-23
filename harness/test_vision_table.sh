#!/usr/bin/env bash
# test_vision_table.sh —— 方案A 定点复现测试：能不能穿过"两张 cafe_table 之间的假口子"
#
# 背景（P0_验收记录 §3b）：两张 cafe_table 桌面各 0.913 m 见方、桌面高 0.755 m，
# 中心间距 0.92 m ⇒ 两桌面几乎相贴（真实不可通行）；但 2D 雷达只看得见两根 4.2 cm 立柱，
# /map 里呈现为 0.92 m 的口子 ⇒ 规划器往里开，轮子撞的是 0.56 m 见方的贴地底板。
#
# 用法: harness/test_vision_table.sh <0|1> [标签]     0=纯雷达基线  1=视觉层
# 判据: 视觉轮必须"物理接触=0"（绕行或干净失败均可），基线轮预期 >0（用于证明测试有效）。
set +u
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy ROS_LOCALHOST_ONLY
source /home/kbf/stemm_sim_ws/domain_guard.sh || exit 9
source /opt/ros/humble/setup.bash
source /home/kbf/stemm_sim_ws/install/setup.bash
source /home/kbf/stemm_explore_ws/install/setup.bash
source /home/kbf/stemm_exploration_sim_ws/install/setup.bash
V="${1:-1}"; TAG="${2:-tbl}"
BASE=/home/kbf/stemm_exploration_sim_ws
OUT="$BASE/logs/table_v${V}_${TAG}_$(date +%Y%m%d-%H%M%S)"; mkdir -p "$OUT"
SX="${SX:-2.04}"; SY="${SY:-1.37}"; GX="${GX:-5.90}"; GY="${GY:-1.37}"; SECS="${SECS:-90}"
say() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$OUT/harness.log"; }
residue() {
  ps -eo pid,args | grep -E "wheeltec_two_wheel_model/(scan_deadzone_relay|imu_bias_shim)|cartographer_exploration_sim/(collision_probe|session_driver)|gzserver" \
    | grep -vE "grep|test_vision_table" | awk '{print $1}' | tr '\n' ' '
}
R=$(residue); [ -n "$R" ] && { say "清理残留: $R"; for p in $R; do kill -TERM "$p" 2>/dev/null; done; sleep 3;
  for p in $R; do kill -9 "$p" 2>/dev/null; done; sleep 1; }
say "起车前残留=[$(residue)]"

if [ "$V" = "1" ]; then
  python3 harness/vision_nav_params.py --apply 2>&1 | tee -a "$OUT/harness.log"
  [ "${PIPESTATUS[0]}" = "0" ] || { say "注入失败"; exit 11; }
else
  python3 harness/vision_nav_params.py --restore 2>&1 | tee -a "$OUT/harness.log" || true
fi
trap 'python3 /home/kbf/stemm_exploration_sim_ws/harness/vision_nav_params.py --restore >/dev/null 2>&1' EXIT INT TERM

say "起栈：vision=$V 起点=($SX,$SY) 目标=($GX,$GY)"
setsid bash -c "echo \$\$ > '$OUT/pgid'; exec ros2 launch stemm_cartographer_exploration_sim sim_explore.launch.py \
  vision:=$V rviz:=false escape_supervisor:=false auto_explore:=false explore_sec:=900.0 \
  spawn_x:=$SX spawn_y:=$SY spawn_yaw:=0.0 probe_log:=$OUT/probe.jsonl" > "$OUT/launch.log" 2>&1 < /dev/null &
sleep 3; PGID=$(cat "$OUT/pgid" 2>/dev/null); say "PGID=$PGID"

for i in $(seq 1 60); do
  grep -q "ready_guard_timing" "$OUT/launch.log" 2>/dev/null && { say "栈就绪（$((i*4))s）"; break; }
  sleep 4
done
sleep 12   # 让 cartographer 出 map 帧 / 视觉开始累积

if [ "$V" = "1" ]; then
  say "视觉层交叉验证"
  python3 harness/vision_check.py --timeout 45 --json "$OUT/vision_check.json" 2>&1 | tee -a "$OUT/harness.log"
fi

say "下发目标 ($GX,$GY)，最多等 ${SECS}s"
timeout $((SECS+20)) ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $GX, y: $GY, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" \
  --feedback > "$OUT/goal.log" 2>&1
GOAL_RC=$?
say "目标返回 rc=$GOAL_RC（0=成功 4=被拒/中止）"
grep -E "status:|Goal finished|succeeded|aborted|canceled|rejected" "$OUT/goal.log" | tail -4 | tee -a "$OUT/harness.log"

say "收尾"
[ -n "$PGID" ] && kill -TERM -- "-$PGID" 2>/dev/null
for i in $(seq 1 20); do pgrep -x gzserver >/dev/null || break; sleep 1; done
pkill -x gzserver 2>/dev/null; sleep 2
[ -n "$PGID" ] && kill -9 -- "-$PGID" 2>/dev/null
python3 harness/vision_nav_params.py --restore 2>&1 | tee -a "$OUT/harness.log" || true

python3 - "$OUT" "$V" <<'PY'
import json, os, sys, collections
out, v = sys.argv[1], sys.argv[2]
p = os.path.join(out, 'probe.jsonl')
c = collections.Counter(); last = {}
mind = None
for line in open(p, encoding='utf-8', errors='ignore'):
    try: r = json.loads(line)
    except Exception: continue
    t = r.get('type')
    if t in ('contact', 'graze', 'alert', 'stall'):
        c[t] += 1
    for k in ('min_d', 'min_d_now', 'min_d_ever'):
        val = r.get(k)
        if isinstance(val, (int, float)) and val >= 0 and (mind is None or val < mind):
            mind = val
    if t == 'summary':
        last = r
res = {'vision': int(v),
       'contacts': c['contact'], 'grazes': c['graze'], 'alerts': c['alert'],
       'chassis_contacts': last.get('chassis_contacts', 0),
       'wheel_contacts': last.get('wheel_contacts', 0),
       'physical_contacts': last.get('chassis_contacts', 0) + last.get('wheel_contacts', 0),
       'min_clearance_ever': round(mind, 3) if mind is not None else None,
       'final_pos': last.get('pos')}
json.dump(res, open(os.path.join(out, 'table_result.json'), 'w'), ensure_ascii=False, indent=1)
print('[table] ' + json.dumps(res, ensure_ascii=False))
PY
say "产物：$OUT"
