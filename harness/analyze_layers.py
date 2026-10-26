#!/usr/bin/env python3
"""analyze_layers.py —— C/A 两层的"是否真的发生"离线统计（只读，不改任何东西）。

为什么需要它：C（探索期受控倒车）与 A（pinch 可退出性）的**触发次数本来就少**
（≥6 s 的 blocked 段约 0.3 段/轮），只看 digest 的 S0/S1 会看不到"这一层到底有没有工作过"。
本脚本把每轮的"发生证据"直接列出来，便于判断"没效果"是"没触发"还是"触发了但没用"。

用法：
  python3 harness/analyze_layers.py base2 c3        # 支持多批，按前缀匹配 logs/round_*_<前缀>*
指标口径：
  blocked 段   = 探索期(排除 return/home) nav=blocked_waiting_clearance/blocked_precheck_pending
                 或 mode=safety_stop 的连续 tick（间隔 >2.5 s 视为断开）
  接触         = evaluate.json → collision.physical_contacts
  C            = supervisor.jsonl 里 rev_probe(would_fire) 与 reverse_assist_start(phase=explore)
  A            = timeline.jsonl 末行的 reject_noexit / noexit_passed / noexit_checked / pinch_len_max
"""
import glob
import json
import os
import sys
from collections import Counter

ROOT = '/home/kbf/stemm_exploration_sim_ws'


def jsonl(path):
    out = []
    try:
        fh = open(path, encoding='utf-8', errors='replace')
    except FileNotFoundError:
        return out
    with fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except Exception:
                pass
    return out


def blocked_runs(rows):
    runs = []
    cur = None
    for r in rows:
        blob = ((r.get('mode') or '') + ' ' + (r.get('nav') or '')).lower()
        ret = ('return' in blob) or ('home' in blob)
        hit = (not ret) and (('blocked_waiting_clearance' in blob)
                             or ('blocked_precheck' in blob)
                             or ('safety_stop' in (r.get('mode') or '').lower()))
        if hit:
            if cur is not None and r['t'] - cur['t'] > 2.5:
                runs.append(cur)
                cur = None
            if cur is None:
                cur = {'t0': r['t'], 't': r['t'], 'fr': []}
            cur['t'] = r['t']
            if isinstance(r.get('front'), (int, float)):
                cur['fr'].append(r['front'])
        elif cur is not None:
            runs.append(cur)
            cur = None
    if cur is not None:
        runs.append(cur)
    return [(round(x['t'] - x['t0'], 1), min(x['fr']) if x['fr'] else None) for x in runs]


def one(d):
    o = {'dir': os.path.basename(d)}
    try:
        ev = json.load(open(os.path.join(d, 'evaluate.json'), encoding='utf-8'))
        col = ev.get('collision') or {}
        o['contact'] = col.get('physical_contacts')
        o['chassis'] = col.get('chassis_contacts')
        o['wheel'] = col.get('wheel_contacts')
        o['rev_explore_m'] = col.get('reverse_explore_m')
        o['verdict'] = ev.get('verdict')
    except Exception:
        pass
    sup = jsonl(os.path.join(d, 'supervisor.jsonl'))
    probe = [r for r in sup if r.get('type') == 'rev_probe']
    starts = [r for r in sup if r.get('type') == 'reverse_assist_start']
    o['probe_rows'] = len(probe)
    o['would_fire'] = sum(1 for r in probe if r.get('would_fire'))
    o['hold_max'] = max([r.get('hold_s') or 0 for r in probe], default=0.0)
    why = Counter()
    for r in probe:
        for w in (r.get('why') or []):
            why['not_holding' if str(w).startswith('not_holding_') else w] += 1
    o['why'] = dict(why.most_common(4))
    o['rev_start_explore'] = sum(1 for s in starts if s.get('phase') == 'explore')
    o['rev_start_contact'] = sum(1 for s in starts if s.get('phase') == 'contact')
    o['rev_start_return'] = sum(1 for s in starts if s.get('phase') == 'return')
    o['units'] = sum(1 for e in sup if e.get('type') == 'unit_start')
    tl = jsonl(os.path.join(d, 'timeline.jsonl'))
    runs = blocked_runs(tl)
    o['blk_n'] = len(runs)
    o['blk_ge10'] = sum(1 for x in runs if x[0] >= 10)
    o['blk_max'] = max([x[0] for x in runs], default=0)
    if tl:
        last = tl[-1]
        for k in ('reject_noexit', 'noexit_passed', 'noexit_checked',
                  'pinch_len_max', 'rejected_narrow'):
            o[k] = last.get(k)
    return o


def main():
    prefixes = sys.argv[1:] or ['base2']
    for pre in prefixes:
        dirs = sorted(glob.glob(os.path.join(ROOT, 'logs', 'round_*_' + pre + '*')))
        if not dirs:
            print('（没有匹配 round_*_%s*）' % pre)
            continue
        print('\n==== 批次 %s（%d 轮） ====' % (pre, len(dirs)))
        hdr = ('轮次', '接触', '判定', 'blocked(段/≥10s/最长)', 'C探针行', 'C可触发',
               'C倒车(探/接触/返)', '脱困单元', 'A否决', 'A放行', 'A检查', '最长pinch', 'narrow否决')
        print('%-30s %s' % (hdr[0], ' '.join(hdr[1:])))
        tot = Counter()
        for d in dirs:
            o = one(d)
            print('%-30s %5s %-5s %3s/%2s/%4ss %8s %7s %6s %7s %6s %6s %6s %9s %9s' % (
                o['dir'][:30], o.get('contact'), str(o.get('verdict'))[:4],
                o.get('blk_n', 0), o.get('blk_ge10', 0), o.get('blk_max', 0),
                o.get('probe_rows'), o.get('would_fire'),
                '%d/%d/%d' % (o.get('rev_start_explore', 0), o.get('rev_start_contact', 0),
                              o.get('rev_start_return', 0)),
                o.get('units'), o.get('reject_noexit'), o.get('noexit_passed'),
                o.get('noexit_checked'), o.get('pinch_len_max'), o.get('rejected_narrow')))
            for k in ('contact', 'would_fire', 'rev_start_explore', 'units',
                      'reject_noexit', 'noexit_passed', 'noexit_checked', 'blk_ge10'):
                tot[k] += int(o.get(k) or 0)
        n = len(dirs)
        print('合计: 接触=%d 轮(%.0f%%) C可触发=%d C倒车=%d 脱困单元=%d A否决=%d A放行=%d ≥10s段=%d' % (
            tot['contact'], 100.0 * sum(1 for d in dirs if (one(d).get('contact') or 0) > 0) / n,
            tot['would_fire'], tot['rev_start_explore'], tot['units'],
            tot['reject_noexit'], tot['noexit_passed'], tot['blk_ge10']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
