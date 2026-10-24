#!/usr/bin/env bash
# round.sh —— 探索建图单轮运行器（确定性启动/收尾 + 六项验收取证）
# 用法: round.sh <标签> <探索秒数> [启动后等待秒数(仅空跑用)]
# 产物: runs/<时间戳>-<标签>.jsonl 摘要 + logs/round_<时间戳>_<标签>/launch.log
set +u
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy ROS_LOCALHOST_ONLY
source /home/kbf/stemm_sim_ws/domain_guard.sh || exit 9
source /opt/ros/humble/setup.bash
source /home/kbf/stemm_sim_ws/install/setup.bash
source /home/kbf/stemm_explore_ws/install/setup.bash
source /home/kbf/stemm_exploration_sim_ws/install/setup.bash

TAG="${1:-run}"; HOLD="${3:-0}"
# 探索秒数必须是 DOUBLE（session_driver 把 explore_sec 声明成 double，传 120 会 InvalidParameterType）
EXPLORE=$(python3 -c "print(float('${2:-120.0}'))")
BASE=/home/kbf/stemm_exploration_sim_ws
TS=$(date +%Y%m%d-%H%M%S)
OUT="$BASE/logs/round_${TS}_${TAG}"; mkdir -p "$OUT"
J="$BASE/runs/${TS}-${TAG}.jsonl"
# 返航"等精确终态"的宽限（P0-1，用户要求暂时放宽）：>0 时用可回退注入改车端 launch，0=原厂 6 s
GRACE_SEC="${GRACE_SEC:-22.0}"
# 方案A 视觉层开关（VISION=1 时注入 nav2 视觉图层 + 起视觉障碍源；0=纯雷达基线）
VISION="${VISION:-0}"
MEMLOG="$OUT/mem.jsonl"

# 内存采样：本机 15 GiB 且 **无 swap**，视觉链路必须证明"加得起"
mem_sample() {
  local avail total=0 pid r
  avail=$(awk '/MemAvailable/{printf "%d", $2/1024}' /proc/meminfo 2>/dev/null)
  rss_of() {
    local sum=0 pp rr
    for pp in $(pgrep -f "$1" 2>/dev/null); do
      rr=$(awk '/VmRSS/{print $2}' /proc/$pp/status 2>/dev/null)
      [ -n "$rr" ] && sum=$((sum + rr/1024))
    done
    echo "$sum"
  }
  # CPU：同一批进程的累计 tick（utime+stime），由汇总脚本按时间差换算成"几个核"
  cpu_of() {
    local sum=0 pp
    for pp in $(pgrep -f "$1" 2>/dev/null); do
      [ -r "/proc/$pp/stat" ] || continue
      sum=$((sum + $(awk '{print $14+$15}' "/proc/$pp/stat" 2>/dev/null || echo 0)))
    done
    echo "$sum"
  }
  printf '{"t":%s,"avail_mb":%s,"gz_mb":%s,"vision_mb":%s,"nav2_mb":%s,"carto_mb":%s,"cpu_gz":%s,"cpu_vision":%s,"cpu_nav2":%s,"cpu_carto":%s}\n' \
    "$(date +%s)" "${avail:-0}" "$(rss_of 'gzserver')" "$(rss_of 'vision_obstacle_grid')" \
    "$(rss_of 'controller_server|planner_server|bt_navigator|behavior_server|smoother_server|waypoint_follower')" \
    "$(rss_of 'cartographer_node')" \
    "$(cpu_of 'gzserver')" "$(cpu_of 'vision_obstacle_grid')" \
    "$(cpu_of 'controller_server|planner_server|bt_navigator|behavior_server|smoother_server|waypoint_follower')" \
    "$(cpu_of 'cartographer_node')" >> "$MEMLOG"
}
say() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$OUT/harness.log"; }
rec() { printf '{"ts":"%s","tag":"%s","%s"}\n' "$(date -Iseconds)" "$TAG" "$1" >> "$J"; }

# ---- 1) 启动前断言：域 42 零节点 + 无 gzserver ----
proc_residue() { pgrep -af "waypoint_follower|controller_server|planner_server|bt_navigator|behavior_server|smoother_server|cartographer_node|occupancy_grid|stemm_cartographer|robot_state_publisher|ekf_node|imu_bias_shim|scan_deadzone_relay|async_slam_toolbox|nav2_" 2>/dev/null | grep -vE "harness/(round|sweep)\.sh|grep|pgrep" | awk '{print $1}' | tr '\n' ' '; }
NODES=$(ros2 node list 2>/dev/null | wc -l)
GZ=$(pgrep -cx gzserver)
PRES=$(proc_residue)
say "启动前：ROS 节点数=$NODES（含DDS僵尸，仅记录）  gzserver=$GZ  进程残留=[${PRES}]"
rec "\"check\":\"pre_launch\",\"nodes\":$NODES,\"gzserver\":$GZ,\"residue\":\"${PRES}\""
if [ -n "$PRES" ] || [ "$GZ" -gt 0 ]; then
  say "存在残留，先清理"
  for p in $(proc_residue); do kill -TERM "$p" 2>/dev/null; done
  sleep 2
  for p in $(proc_residue); do kill -9 "$p" 2>/dev/null; done
  pkill -x gzserver 2>/dev/null; sleep 4
  NODES=$(ros2 node list 2>/dev/null | wc -l); GZ=$(pgrep -cx gzserver); PRES=$(proc_residue)
  say "清理后：ROS 节点数=$NODES  gzserver=$GZ  进程残留=[${PRES}]"
  rec "\"check\":\"post_cleanup\",\"nodes\":$NODES,\"gzserver\":$GZ,\"residue\":\"${PRES}\""
  { [ -n "$PRES" ] || [ "$GZ" -gt 0 ]; } && { say "清理不彻底，放弃本轮"; rec '"verdict":"FAIL_preclean"'; exit 10; }
fi

rm -f /home/kbf/stemm_sim_ws/maps/explore_latest.pgm /home/kbf/stemm_sim_ws/maps/explore_latest.yaml
T_START=$(date +%s)

# 随机起点（5 个候选，房间原点附近的安全偏移位姿）
POSES=("0.0 0.0 0.0" "0.4 0.3 1.5708" "-0.3 0.4 3.1416" "0.3 -0.4 -1.5708" "-0.4 -0.3 0.7854")
POSE=${POSE_OVERRIDE:-${POSES[$((RANDOM % ${#POSES[@]}))]}}
set -- $POSE; SX=$1; SY=$2; SYAW=$3
say "本轮随机起点：x=$SX y=$SY yaw=$SYAW"
rec "\"check\":\"pose\",\"x\":$SX,\"y\":$SY,\"yaw\":$SYAW"

# ---- 2) 启动（记录真实 PGID） ----
# P0 变体开关（默认=改造后：关盲退 + 开脱困监督）
DISABLE_RETREAT="${DISABLE_RETREAT:-1}"
SUPERVISOR="${SUPERVISOR:-true}"
CAR_ESCAPE="${CAR_ESCAPE:-$([ "${DISABLE_CAR_ESCAPE:-1}" = "1" ] && echo false || echo true)}"
say "P0 变体：disable_retreat=$DISABLE_RETREAT escape_supervisor=$SUPERVISOR"
rec "\"check\":\"variant\",\"disable_retreat\":$DISABLE_RETREAT,\"supervisor\":\"$SUPERVISOR\""

# ---- 方案A：nav2 视觉图层注入（幂等、可回退；teardown 后自动撤销）----
# ---- P0-1：返航姿态宽限注入（可回退；teardown 撤销）----
if [ "$(python3 -c "print(1 if float('$GRACE_SEC')>0 else 0)")" = "1" ]; then
  python3 harness/patch_car_launch.py --apply --grace "$GRACE_SEC" 2>&1 | tee -a "$OUT/harness.log"
  say "返航姿态宽限：${GRACE_SEC}s（原厂 6 s；GRACE_SEC=0 可关）"
  trap 'python3 /home/kbf/stemm_exploration_sim_ws/harness/patch_car_launch.py --restore >/dev/null 2>&1; python3 /home/kbf/stemm_exploration_sim_ws/harness/vision_nav_params.py --restore >/dev/null 2>&1' EXIT INT TERM
fi

say "视觉层 VISION=$VISION"
rec "\"check\":\"vision\",\"enabled\":$VISION"
trap 'python3 /home/kbf/stemm_exploration_sim_ws/harness/vision_nav_params.py --restore >/dev/null 2>&1' EXIT INT TERM
if [ "$VISION" = "1" ]; then
  python3 harness/vision_nav_params.py --apply 2>&1 | tee -a "$OUT/harness.log"
  if [ "${PIPESTATUS[0]}" != "0" ]; then
    say "✗ 视觉图层注入失败，放弃本轮（参数文件未被改动）"
    rec '"verdict":"FAIL_vision_params"'
    exit 11
  fi
else
  python3 harness/vision_nav_params.py --restore 2>&1 | tee -a "$OUT/harness.log" || true
python3 harness/patch_car_launch.py --restore 2>&1 | tee -a "$OUT/harness.log" || true
fi
setsid bash -c "echo \$\$ > '$OUT/pgid'; exec ros2 launch stemm_cartographer_exploration_sim sim_explore.launch.py explore_sec:=$EXPLORE spawn_x:=$SX spawn_y:=$SY spawn_yaw:=$SYAW disable_retreat:=$DISABLE_RETREAT escape_supervisor:=$SUPERVISOR car_escape:=$CAR_ESCAPE rviz:=${RVIZ:-false} stack_delay:=${STACK_DELAY:-0.0} scoring:=${SCORING:-v1} corridor_width:=${CORRIDOR_W:-1.05} supervisor_rotate_min_range:=${SUP_ROT_MIN:-0.34} supervisor_advance_stop_range:=${SUP_ADV_STOP:-0.34} probe_log:=$OUT/probe.jsonl supervisor_log:=$OUT/supervisor.jsonl vision:=$VISION vision_dump:=$([ "$VISION" = "1" ] && echo "$OUT/vision_grid" || echo '') watchdog_log:=$OUT/watchdog.jsonl nudge_log:=$OUT/nudge.jsonl" \
  > "$OUT/launch.log" 2>&1 < /dev/null &
sleep 3; PGID=$(cat "$OUT/pgid" 2>/dev/null || echo "")
say "已启动：PGID=$PGID  日志 $OUT/launch.log"
rec "\"check\":\"launch\",\"pgid\":\"$PGID\""

if [ "${SMOKE:-0}" = "1" ]; then
  say "SMOKE 模式：等就绪后做 10 项自检（不跑探索）"
  # 等"栈就绪 + 运行期补丁全部落地"两个条件都满足，再做自检
  for i in $(seq 1 30); do
    grep -q "ready_guard_timing" "$OUT/launch.log" 2>/dev/null && break
    sleep 3
  done
  for i in $(seq 1 25); do
    grep -q "全部阶段完成" "$OUT/launch.log" 2>/dev/null && break
    sleep 2
  done
  # 保守口径：再等最多 50 s 等"第一个探索目标派发"，用它证明探索环真的转起来了
  for i in $(seq 1 25); do
    grep -q "navigation goal dispatched" "$OUT/launch.log" 2>/dev/null && break
    sleep 2
  done
  sleep 3
  bash harness/smoke_check.sh "$OUT/launch.log" | tee -a "$OUT/harness.log"
  SMOKE_RC=${PIPESTATUS[0]}
  rec "\"check\":\"smoke\",\"rc\":$SMOKE_RC"
  [ "$SMOKE_RC" = "0" ] && rec '"verdict":"PASS_smoke"' || rec '"verdict":"FAIL_smoke"'
  PGID_SAVED=$PGID
  teardown_once() {
    [ -n "$PGID_SAVED" ] && kill -TERM -- "-$PGID_SAVED" 2>/dev/null
    for i in $(seq 1 20); do pgrep -x gzserver >/dev/null || break; sleep 1; done
    pkill -x gzserver 2>/dev/null; sleep 2
  }
  teardown_once
  echo "JSONL: $J"
  exit "$SMOKE_RC"
fi

if [ "$HOLD" -gt 0 ]; then
  sleep "$HOLD"
  say "空跑 $HOLD s 结束，执行 teardown 验证"
else
  # 全流程模式：等会话自然收尾（会话结束 / 地图落盘 / 超时上限）
  MAXW=$(python3 -c "print(int(float('$EXPLORE') + 300))")
  say "全流程模式：最多等待 ${MAXW}s（探索 ${EXPLORE}s + 返航/存图余量）"
  waited=0
  AUDIT_DONE=0
  BACKSTEP_EXPECT=$([ "${EXPLORE_BACKSTEP:-1}" = "1" ] && echo 1 || echo 0)
  while [ "$waited" -lt "$MAXW" ]; do
    sleep 10; waited=$((waited+10)); mem_sample
    # P0-乙：起栈就绪后立刻做参数落地自检（此时 patch_sim_time 已跑完、栈还活着）
    if [ "$AUDIT_DONE" = "0" ] && grep -q "ready_guard_timing" "$OUT/launch.log" 2>/dev/null; then
      bash harness/param_audit.sh "$OUT/param_audit.json" "$BACKSTEP_EXPECT" 2>&1 | tee -a "$OUT/harness.log"
      if [ "${PIPESTATUS[0]}" = "0" ]; then rec '"check":"param_audit","ok":true'
      else say "✗ 参数自检未全通过 → 本轮 invalid(param_audit)"; rec '"check":"param_audit","ok":false'; fi
      AUDIT_DONE=1
    fi
    if grep -q "会话结束" "$OUT/launch.log" 2>/dev/null; then say "检测到会话结束（${waited}s）"; break; fi
    # 注意：会话驱动在返航期每 2 s 滚动存图，"地图文件出现"不再是收尾信号，
    # 必须等"会话结束"或 manager 明确返航到位（否则会在返航刚开始时就收尾）。
    # 只认 manager 的正向完成行；不要匹配"会话未走到返航到位"这类否定句
    if grep -qE "return-home completed by position|return-home goal reached" "$OUT/launch.log" 2>/dev/null; then
      say "返航到位（${waited}s）"; sleep 8; break
    fi
    pgrep -x gzserver >/dev/null || { say "gzserver 已退出（${waited}s）"; break; }
  done
fi

# ---- 3) teardown（按 PGID） ----
teardown() {
  [ -n "$PGID" ] && kill -TERM -- "-$PGID" 2>/dev/null
  for i in $(seq 1 20); do pgrep -x gzserver >/dev/null || break; sleep 1; done
  pkill -x gzserver 2>/dev/null
  for i in $(seq 1 10); do [ "$(ros2 node list 2>/dev/null | wc -l)" -le 1 ] && break; sleep 1; done
}
teardown
NODES=$(ros2 node list 2>/dev/null | wc -l); GZ=$(pgrep -cx gzserver)
RES=$(pgrep -af "cartographer_node|sim_session_driver|controller_server|planner_server|bt_navigator|imu_bias_shim|scan_deadzone_relay" 2>/dev/null | grep -E "/(lib/|install/|opt/ros)" | grep -v "harness/round.sh" | awk '{print $1}' | tr '\n' ' ')
say "收尾后：ROS 节点数=$NODES  gzserver=$GZ  残留=[${RES}]"
rec "\"check\":\"post_teardown\",\"nodes\":$NODES,\"gzserver\":$GZ,\"residue\":\"${RES}\""
[ "$GZ" = "0" ] && [ -z "$RES" ] && rec '"verdict":"PASS_clean"' || rec '"verdict":"FAIL_residue"'

# ---- 起栈诊断：ready_guard 时序 + 生命周期 nudge 事件 ----
if grep -q "ready_guard_timing" "$OUT/launch.log" 2>/dev/null; then
  say "起栈时序：$(grep -o 'ready_guard_timing[^\n]*' "$OUT/launch.log" | tail -1 | cut -c1-110)"
elif grep -qE "Nav2 did not become active|did not become ready" "$OUT/launch.log" 2>/dev/null; then
  say "✗ 起栈失败：nav2 生命周期未在 guard 超时内激活"
  rec '"check":"startup_fail"'
fi
if [ -s "$OUT/nudge.jsonl" ]; then
  python3 - "$OUT/nudge.jsonl" <<'PYEOF3'
import json, sys
rows = []
for line in open(sys.argv[1], encoding='utf-8', errors='ignore'):
    line = line.strip()
    if line:
        try:
            rows.append(json.loads(line))
        except Exception:
            pass
nudges = [r for r in rows if r.get('type') == 'nudge']
summ = [r for r in rows if r.get('type') == 'summary']
print('[nudge] 插队 %d 次%s' % (len(nudges),
      ('；' + ', '.join('%s×%d' % (n['node'].lstrip('/'), n['attempt']) for n in nudges)) if nudges else '（起栈正常，未插队）'))
if summ:
    print('  收尾：%d 个 ACTIVE' % len(summ[-1].get('active', [])))
PYEOF3
fi

# ---- P1-2：静止看门狗事件汇总 ----
if [ -f "$OUT/watchdog.jsonl" ]; then
  python3 - "$OUT/watchdog.jsonl" <<'PYEOF2'
import json, sys, collections
rows = []
for line in open(sys.argv[1], encoding='utf-8', errors='ignore'):
    line = line.strip()
    if line:
        try:
            rows.append(json.loads(line))
        except Exception:
            pass
why = collections.Counter(r.get('why', '?') for r in rows)
print('[watchdog] 静止事件 %d 次：%s' % (len(rows), dict(why)))
for r in rows[:6]:
    print('    %.1fs @(%.2f,%.2f) %s err=%s'
          % (r.get('stall_sec', 0), r['pos'][0], r['pos'][1], r.get('why'), r.get('last_error')))
PYEOF2
fi

# ---- 方案A：撤销视觉图层注入 + 内存峰值汇总 ----
python3 harness/vision_nav_params.py --restore 2>&1 | tee -a "$OUT/harness.log" || true
python3 harness/patch_car_launch.py --restore 2>&1 | tee -a "$OUT/harness.log" || true
if [ -s "$MEMLOG" ]; then
  python3 - "$MEMLOG" "$J" <<'PYEOF'
import json, sys
rows = [json.loads(l) for l in open(sys.argv[1]) if l.strip()]
# 只在"栈还活着"的采样区间里算 CPU（收尾后进程消失，tick 会归零，否则会算出负值）
rows_live = [r for r in rows if (r.get('gz_mb') or 0) > 0 and (r.get('cpu_gz') or 0) >= 0]
peak = {k: max((r.get(k) or 0) for r in rows) for k in
        ('gz_mb', 'vision_mb', 'nav2_mb', 'carto_mb')}
low = min((r.get('avail_mb') or 0) for r in rows)
summary = {'samples': len(rows), 'peak_gz_mb': peak['gz_mb'], 'peak_vision_mb': peak['vision_mb'],
           'peak_nav2_mb': peak['nav2_mb'], 'peak_carto_mb': peak['carto_mb'],
           'min_avail_mb': low}
# CPU 换算：100 = 一个核跑满；tick 差 / (时间差 × CLK_TCK)
try:
    import os as _os
    hz = _os.sysconf('SC_CLK_TCK')
    src = rows_live if len(rows_live) >= 2 else rows
    dt = max(1, src[-1]['t'] - src[0]['t'])
    for k, tag in (('cpu_gz', 'gz'), ('cpu_vision', 'vision'),
                   ('cpu_nav2', 'nav2'), ('cpu_carto', 'carto')):
        vals = [r.get(k) for r in src if r.get(k) is not None]
        if len(vals) >= 2:
            summary['cpu_%s_pct' % tag] = round(100.0 * (vals[-1] - vals[0]) / (dt * hz), 1)
except Exception:
    pass
with open(sys.argv[2], 'a') as f:
    f.write('{"check":"memory",' + json.dumps(summary, ensure_ascii=False)[1:] + '\n')
print('[memory] ' + json.dumps(summary, ensure_ascii=False))
PYEOF
fi

# ---- 4) 撞障取证汇总（P0）----
if [ -f "$OUT/probe.jsonl" ]; then
  python3 - "$OUT/probe.jsonl" "$J" <<'PYEOF'
import json, sys, collections
src, jout = sys.argv[1], sys.argv[2]
c = collections.Counter(); mind = None; rev = []
for line in open(src, encoding='utf-8', errors='ignore'):
    line = line.strip()
    if not line:
        continue
    try:
        r = json.loads(line)
    except Exception:
        continue
    t = r.get('type')
    if t in ('contact', 'graze', 'alert', 'stall', 'giveup', 'no_direction'):
        c[t] += 1
    if t == 'reverse':
        if r.get('phase') == 'end':
            rev.append(r)
    for k in ('min_d', 'min_d_now', 'min_d_ever'):
        v = r.get(k)
        if isinstance(v, (int, float)) and v >= 0 and (mind is None or v < mind):
            mind = v
last = {}
for line in open(src, encoding='utf-8', errors='ignore'):
    try:
        r = json.loads(line)
    except Exception:
        continue
    if r.get('type') == 'summary':
        last = r
hard = [x for x in rev if (x.get('dist') or 0.0) >= 0.05 and not x.get('assist')]
assist = [x for x in rev if x.get('assist')]
summary = {'contacts': c['contact'], 'grazes': c['graze'], 'alerts': c['alert'],
           'stalls': c['stall'], 'reverse_windows': len(rev),
           'reverse_m': round(sum(x.get('dist', 0.0) for x in rev), 3),
           'reverse_assist_windows': len(assist),
           'reverse_assist_m': round(sum((x.get('dist') or 0.0) for x in assist), 3),
           'reverse_hard_windows': len(hard),
           'reverse_hard_m': round(sum((x.get('dist') or 0.0) for x in hard), 3),
           'chassis_contacts': last.get('chassis_contacts', 0),
           'wheel_contacts': last.get('wheel_contacts', 0),
           'physical_contacts': last.get('chassis_contacts', 0) + last.get('wheel_contacts', 0),
           'min_clearance_ever': round(mind, 3) if mind is not None else None}
line = '{"tag":"%s","check":"collision","%s"}' % (jout.rsplit('/', 1)[-1], '')
with open(jout, 'a') as f:
    f.write('{"check":"collision","summary":%s}\n' % json.dumps(summary, ensure_ascii=False))
print('[collision] ' + json.dumps(summary, ensure_ascii=False))
PYEOF
fi
# 地图快照（下一轮开头会删 explore_latest.*，不快照就再也评不到本轮的地图了）
MF=/home/kbf/stemm_sim_ws/maps/explore_latest.pgm
if [ -f "$MF" ] && [ "$(stat -c %Y "$MF")" -ge "$T_START" ]; then
  cp -f "$MF" "$OUT/map.pgm"
  cp -f "/home/kbf/stemm_sim_ws/maps/explore_latest.yaml" "$OUT/map.yaml" 2>/dev/null
  say "地图快照：$OUT/map.pgm（$(stat -c %s "$MF") B）"
fi
# 本轮验收（七项 + P0 两项）
if [ -f "$OUT/launch.log" ]; then
  python3 harness/evaluate.py "$OUT/launch.log" "$TAG" "$OUT/probe.jsonl" --p0 \
    > "$OUT/evaluate.json" 2>&1 || true
  python3 - "$OUT/evaluate.json" <<'PYEOF'
import json,sys
try:
    d=json.load(open(sys.argv[1]))
    print('[evaluate] verdict=%s checks=%s' % (d['verdict'],
          {k:v for k,v in d['checks'].items() if not v} or 'all true'))
except Exception as e:
    print('[evaluate] 解析失败:', e)
PYEOF
fi
if [ -f "$OUT/launch.log" ]; then
  echo
  python3 harness/report.py "$OUT" "$TAG" > "$OUT/report.stdout" 2>&1 || true
  head -40 "$OUT/report.stdout"
  echo "[report] 完整报告：$OUT/report.md"
fi
if [ -f "$OUT/supervisor.jsonl" ]; then
  echo "[supervisor] 事件汇总：" 
  grep -o '"type": "[a-z_0-9]*"' "$OUT/supervisor.jsonl" | sort | uniq -c || true
fi
echo "JSONL: $J"
