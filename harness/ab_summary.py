#!/usr/bin/env python3
"""ab_summary.py —— 汇总 A/B 轮次（VISION=0/1）的验收、物理接触与内存峰值，输出 markdown 表。

用法: ab_summary.py [<glob 前缀>=abv] [轮数=3]
读取 runs/<ts>-abv{V}r{i}.jsonl 与 logs/round_*_abv{V}r{i}/evaluate.json
"""
import glob
import json
import os
import sys

base = '/home/kbf/stemm_exploration_sim_ws'
prefix = sys.argv[1] if len(sys.argv) > 1 else 'abv'
rounds = int(sys.argv[2]) if len(sys.argv) > 2 else 3


def run_jsonl(tag):
    hits = sorted(glob.glob(os.path.join(base, 'runs', f'*-{tag}.jsonl')))
    recs = []
    for h in hits:
        for line in open(h, encoding='utf-8', errors='ignore'):
            line = line.strip()
            if not line:
                continue
            try:
                recs.append(json.loads(line))
            except Exception:
                pass
    return recs


def ev(tag):
    ds = sorted(glob.glob(os.path.join(base, 'logs', f'round_*_{tag}', 'evaluate.json')))
    if not ds:
        return {}
    try:
        return json.load(open(ds[-1], encoding='utf-8'))
    except Exception:
        return {}


rows = []
for v in (0, 1):
    for i in range(1, rounds + 1):
        tag = f'{prefix}{v}r{i}'
        e = ev(tag)
        c = e.get('collision') or {}
        vis = e.get('vision') or {}
        mem = [r for r in run_jsonl(tag) if r.get('check') == 'memory']
        m = mem[-1] if mem else {}
        rows.append({
            'tag': tag, 'vision': v,
            'verdict': e.get('verdict', 'NA'),
            'phys': c.get('physical_contacts'), 'wheel': c.get('wheel_contacts'),
            'chassis': c.get('chassis_contacts'),
            'cells': vis.get('cells'),
            'goal_wait_s': None,
            'peak_vision_mb': m.get('peak_vision_mb'), 'peak_nav2_mb': m.get('peak_nav2_mb'),
            'min_avail_mb': m.get('min_avail_mb'),
            'checks_failed': [k for k, val in (e.get('checks') or {}).items() if not val],
        })

print('| 轮次 | 判定 | 物理接触 | 轮/底盘 | 视觉格 | 视觉峰值 | Nav2 峰值 | 可用内存最低 | 未过项 |')
print('| --- | --- | --- | --- | --- | --- | --- | --- | --- |')
for r in rows:
    print('| {tag} | {verdict} | {phys} | {wheel}/{chassis} | {cells} | {vmm} MB | {nmm} MB | {av} MB | {failed} |'
          .format(tag=r['tag'], verdict=r['verdict'],
                  phys='—' if r['phys'] is None else r['phys'],
                  wheel='—' if r['wheel'] is None else r['wheel'],
                  chassis='—' if r['chassis'] is None else r['chassis'],
                  cells='—' if r['cells'] is None else r['cells'],
                  vmm=r['peak_vision_mb'] if r['peak_vision_mb'] is not None else '—',
                  nmm=r['peak_nav2_mb'] if r['peak_nav2_mb'] is not None else '—',
                  av=r['min_avail_mb'] if r['min_avail_mb'] is not None else '—',
                  failed=','.join(r['checks_failed']) or '无'))
