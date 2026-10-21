#!/usr/bin/env python3
"""evaluate.py —— 对一轮 launch.log 做六项验收，输出 PASS/FAIL 与证据行。"""
import json, os, re, sys

log, tag = sys.argv[1], (sys.argv[2] if len(sys.argv) > 2 else 'run')
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
m = re.findall(r'返航到位：距 map 原点 ([\d.]+) m', txt)
checks['return_home'] = f'距原点 {m[-1]} m' if m else None
mapfile = os.path.expanduser('~/stemm_sim_ws/maps/explore_latest.pgm')
checks['map_saved'] = f'{os.path.getsize(mapfile)} B' if os.path.exists(mapfile) else None
ok = all(v for k, v in checks.items() if k != 'return_home')
verdict = 'PASS' if ok and checks['return_home'] else 'FAIL'
print(json.dumps({'tag': tag, 'verdict': verdict,
                  'checks': {k: (v is not None) for k, v in checks.items()},
                  'evidence': {k: v for k, v in checks.items() if v}}, ensure_ascii=False, indent=1))
sys.exit(0 if verdict == 'PASS' else 1)
