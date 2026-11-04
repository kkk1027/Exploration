#!/usr/bin/env bash
# proto_costmap_accept.sh <tag>
#
# 「局部代价地图视觉层虚假/残留 lethal」验收协议（**只读**：不改任何参数/配置，只发 /cmd_vel 让车转圈）
#
# 为什么要有它：前面每一步都靠临时命令对比，口径容易漂。这里把协议钉死成一条命令，
# 后续 C1/C2/A 每一步都用它复测，数字才可比。
#
# 协议（每次完全相同）：
#   起 nav_only（原始口径：局部+全局都有视觉层、live 云、raytrace 3.5、累积 120 s、
#               无 ω 门控、初始位姿 = 地图原点、rviz 关）→ 等 amcl active
#   → ① 静止：vision_vs_truth_probe（视觉点贴合雷达/静态图）
#   → ② 静止基线 3 拍 → 连转 3 整圈（每圈停 10 s）→ 转回 → 停 40 s（4 拍）
#      local_residue_probe 记录 raw_lethal / 膨胀面积 / ratio
#   → ③ 静止再测一次 vision_vs_truth_probe
#   → 收尾，输出 summary.json（五项判据 + 原始序列）
#
# 输出目录：logs/proto_<时间戳>_<tag>/
#   launch.log  vvt_before.json  residue.jsonl  vvt_after.json  contacts.jsonl  summary.json
#
# 用法：bash harness/proto_costmap_accept.sh base
set -u
cd /home/kbf/stemm_exploration_sim_ws || exit 9
TAG="${1:-run}"
OUT="logs/proto_$(date +%Y%m%d-%H%M%S)_${TAG}"
mkdir -p "$OUT"

set +u
source /home/kbf/stemm_sim_ws/domain_guard.sh >/dev/null 2>&1
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source /home/kbf/stemm_sim_ws/install/setup.bash >/dev/null 2>&1
source /home/kbf/stemm_explore_ws/install/setup.bash >/dev/null 2>&1
source /home/kbf/stemm_exploration_sim_ws/install/setup.bash >/dev/null 2>&1
export ROS_LOCALHOST_ONLY=1
set -u

exec 9>/tmp/stemm_round.lock
flock -n 9 || { echo "[proto] 已有轮次在跑"; exit 3; }
bash harness/preflight_dds.sh -q || { echo "[proto] 环境不健康，先 bash harness/stack_reset.sh"; exit 4; }

cleanup() {
  [ -n "${LPID:-}" ] && kill -TERM -- "-$LPID" 2>/dev/null
  sleep 3
  [ -n "${LPID:-}" ] && kill -KILL -- "-$LPID" 2>/dev/null
  bash harness/stack_kill.sh kill >/dev/null 2>&1
  python3 harness/vision_nav_params.py --restore >/dev/null 2>&1
  echo "[proto] 收尾 gz=$(pgrep -cx gzserver 2>/dev/null || true)"
}
trap cleanup EXIT INT TERM

echo "[proto] 起栈（EXTRA_ARGS=${EXTRA_ARGS:-<无>}）→ $OUT/launch.log"
setsid ros2 launch stemm_cartographer_exploration_sim nav_only.launch.py \
  map_yaml:=/home/kbf/stemm_sim_ws/maps/explore_latest.yaml \
  spawn_x:=0.0 spawn_y:=0.0 spawn_yaw:=0.0 \
  vision:=true vision_mode:=accum vision_tf_stamp:=true \
  gui:=false rviz:=false ${EXTRA_ARGS:-} \
  contact_log:="$OUT/contacts.jsonl" > "$OUT/launch.log" 2>&1 &
LPID=$!

for _ in $(seq 1 120); do
  st=$(timeout 8 ros2 lifecycle get /amcl 2>/dev/null | head -1)
  case "$st" in active*) break ;; esac
  sleep 2
done
for _ in $(seq 1 60); do
  timeout 6 ros2 topic hz /local_costmap/costmap 2>/dev/null | grep -q 'average rate' && break
  sleep 2
done
sleep 8

echo "[proto] ① 静止：视觉点 vs 雷达/静态图"
timeout 150 python3 harness/vision_vs_truth_probe.py --samples 5 --tol 0.15 \
  --cloud-topic /vision/live_cloud --out "$OUT/vvt_before.json" 2>&1 | tee "$OUT/vvt_before.log"

echo "[proto] ② 三轮旋转 + 停稳（局部代价地图指标）"
timeout 900 python3 harness/local_residue_probe.py --turns 3 --settle 10 --hold 40 \
  --period 10 --base-n 3 --out "$OUT/residue.jsonl" 2>&1 | tee "$OUT/residue.log"

echo "[proto] ③ 静止再测一次视觉点"
timeout 150 python3 harness/vision_vs_truth_probe.py --samples 3 --tol 0.15 \
  --cloud-topic /vision/live_cloud --out "$OUT/vvt_after.json" 2>&1 | tee "$OUT/vvt_after.log"

echo "[proto] 汇总"
python3 - "$OUT" <<'PY' | tee "$OUT/summary.txt"
import json, os, statistics as st, sys
d = sys.argv[1]
def load(p):
    p = os.path.join(d, p)
    if not os.path.exists(p): return []
    try: return json.load(open(p, encoding='utf-8'))
    except Exception: return []
rows = [json.loads(l) for l in open(os.path.join(d, 'residue.jsonl'), encoding='utf-8') if l.strip()] if os.path.exists(os.path.join(d,'residue.jsonl')) else []
base = [r['raw_lethal'] for r in rows if r['tag'].startswith('base') and r.get('raw_lethal')]
peak = max([r['raw_lethal'] for r in rows if r.get('raw_lethal')] or [0])
hold = [r['raw_lethal'] for r in rows if r['tag'].startswith('hold') and r.get('raw_lethal')]
ratios = [r['ratio'] for r in rows if r.get('ratio')]
b = st.median(base) if base else None
h = st.median(hold) if hold else None
s = dict(
    base_raw_lethal=b, peak_raw_lethal=peak,
    delta_peak=(peak - b) if (b and peak) else None,
    settle_raw_lethal=h,
    settle_return_pct=(round((h - b) / b * 100, 1) if (b and h) else None),
    ratio_median=(round(st.median(ratios), 3) if ratios else None),
    vgrid_peak=(max([r.get('vgrid') or 0 for r in rows]) or None),
)
cone_base = [r['cone_in'] for r in rows if r['tag'].startswith('base') and r.get('cone_in') is not None]
cone_hold = [r['cone_in'] for r in rows if r['tag'].startswith('hold') and r.get('cone_in') is not None]
cb = st.median(cone_base) if cone_base else None
ch = st.median(cone_hold) if cone_hold else None
s['cone_in_base'] = cb
s['cone_in_settle'] = ch
s['cone_in_return_pct'] = (round((ch - cb) / cb * 100, 1) if (cb and ch) else None)
# 局部层的视觉专属贡献（与"无视觉"对照比）：vgrid 是全局累积栅格，供 C2 对照

for key in ('vvt_before', 'vvt_after'):
    v = load(key + '.json')
    fr = [x['frac_near_lidar'] for x in v if isinstance(x.get('frac_near_lidar'), float)]
    fm = [x['frac_near_map'] for x in v if isinstance(x.get('frac_near_map'), float)]
    s[key + '_near_lidar'] = round(sum(fr) / len(fr), 4) if fr else None
    s[key + '_near_map'] = round(sum(fm) / len(fm), 4) if fm else None
# 接触
cp = os.path.join(d, 'contacts.jsonl')
if os.path.exists(cp):
    last = None
    for line in open(cp, encoding='utf-8'):
        try: o = json.loads(line)
        except Exception: continue
        if o.get('type') == 'summary': last = o
    if last:
        s['contact_episodes'] = last.get('contact_episodes')
        s['contact_secs'] = last.get('contact_secs')
print('\n===== 验收指标（协议固定：原点→转3圈→转回→停40s）=====')
def show(k, hint):
    print(f'  {k:24s} = {s.get(k)!s:>10s}   {hint}')
show('base_raw_lethal', '静止基线 raw_lethal')
show('peak_raw_lethal', '旋转期峰值')
show('delta_peak', '判据≤+800')
show('settle_raw_lethal', '停稳后中位')
show('settle_return_pct', '判据回落到基线±10%')
show('ratio_median', '判据 0.6±0.1（膨胀/障碍）')
show('cone_in_base', '前方视野锥内 lethal（静止基线）')
show('cone_in_settle', '前方视野锥内 lethal（停稳后）')
show('cone_in_return_pct', '★锥内回落（用户真正的痛点指标）')
show('vvt_before_near_lidar', '判据≥0.80（视觉点贴雷达）')
show('vvt_after_near_lidar', '同上（旋转后）')
show('vvt_before_near_map', '视觉点贴静态图')
show('contact_episodes', 'S0：必须 0')
show('contact_secs', 'S0：必须 0')
json.dump(s, open(os.path.join(d, 'summary.json'), 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
print(f'\n[proto] summary.json → {os.path.join(d, "summary.json")}')
PY

echo "[proto] 输出 $OUT"
