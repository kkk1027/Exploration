#!/usr/bin/env python3
"""evaluate.py —— 一轮 launch.log 的验收（七项原有 + P0 撞障/后退两项）。

用法：
  evaluate.py <launch.log> [tag] [probe.jsonl] [--p0]
    --p0  把"接触=0 且 反向指令=0"也算进 verdict（P0 改造后的验收口径）。
          不给 --p0 时这两项只作信息输出（对照基线里盲退是预期存在的）。
"""
import json
import os
import re
import sys

args = [a for a in sys.argv[1:] if not a.startswith('--')]
flags = {a for a in sys.argv[1:] if a.startswith('--')}
log = args[0]
tag = args[1] if len(args) > 1 else 'run'
probe = args[2] if len(args) > 2 else None

txt = open(log, encoding='utf-8', errors='ignore').read()


def has(*pats):  # 任意一行同时含全部关键字
    for line in txt.splitlines():
        if all(p in line for p in pats):
            return line.strip()[:160]
    return None


checks = {}
checks['preflight'] = has('preflight', 'passed') or has('system service and process isolation checks passed')
checks['tf_healthy'] = has('mapping TF is healthy')
checks['nav2_active'] = has('stably ready') or has('ready_guard_timing')
checks['frontier'] = has('assigned RRT frontier') or has('Begin navigating')
checks['mapping_done'] = has('SetBool_Response', 'success=True') or has('mapping done acknowledged')
m = re.findall(r'返航(?:到位|完成)[^\n]*', txt)
checks['return_home'] = m[-1][:120] if m else None
mapfile = os.path.expanduser('~/stemm_sim_ws/maps/explore_latest.pgm')
checks['map_saved'] = f'{os.path.getsize(mapfile)} B' if os.path.exists(mapfile) else None

# ---- P0：撞障取证 ----
collision = None
if probe and os.path.exists(probe):
    import collections
    c = collections.Counter()
    rev = 0
    rev_m = 0.0
    mind = None
    for line in open(probe, encoding='utf-8', errors='ignore'):
        line = line.strip()
        if not line:
            continue
        try:
            r = json.loads(line)
        except Exception:
            continue
        t = r.get('type')
        if t in ('contact', 'graze', 'alert'):
            c[t] += 1
        elif t == 'reverse' and r.get('phase') == 'end':
            rev += 1
            rev_m += float(r.get('dist') or 0.0)
        for k in ('min_d', 'min_d_now', 'min_d_ever'):
            v = r.get(k)
            if isinstance(v, (int, float)) and v >= 0 and (mind is None or v < mind):
                mind = v
    collision = {'contacts': c['contact'], 'grazes': c['graze'], 'alerts': c['alert'],
                 'reverse_windows': rev, 'reverse_m': round(rev_m, 2),
                 'min_clearance_ever': round(mind, 3) if mind is not None else None}
    checks['no_contact'] = collision['contacts'] == 0
    checks['no_reverse'] = collision['reverse_windows'] == 0

p0 = '--p0' in flags
hard = [k for k in checks if k not in ('return_home',)]
if not p0:
    hard = [k for k in hard if k not in ('no_contact', 'no_reverse')]
ok = all(checks.get(k) for k in hard)
verdict = 'PASS' if ok and checks.get('return_home') else 'FAIL'
out = {'tag': tag, 'verdict': verdict,
       'checks': {k: bool(v) for k, v in checks.items()},
       'evidence': {k: v for k, v in checks.items() if v}}
if collision is not None:
    out['collision'] = collision
print(json.dumps(out, ensure_ascii=False, indent=1))
sys.exit(0 if verdict == 'PASS' else 1)
