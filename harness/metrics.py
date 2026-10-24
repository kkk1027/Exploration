#!/usr/bin/env python3
"""metrics.py —— 固定五项指标的同口径汇总（P0-4）。

用法: metrics.py [--last N] [--prefix ""] [--dirs a,b,c]

五项指标：
  1) 有效轮数 / 全绿率        —— "有效轮" = 起栈成功(有 ready_guard_timing) 且派发过目标
  2) 返航失败（return_home）
  3) 物理接触 > 0 的轮数
  4) 出现静默静止（看门狗 '静止超时'）的轮数
  5) 起栈失败轮数（无 ready 或 did-not-become-active）
另附仲裁统计中位数：rejected_path / rejected_narrow / rejected_size。
"""
import argparse
import glob
import json
import os
import statistics

W = '/home/kbf/stemm_exploration_sim_ws'


def load(d):
    log = os.path.join(d, 'launch.log')
    txt = open(log, encoding='utf-8', errors='ignore').read() if os.path.exists(log) else ''
    ev = {}
    p = os.path.join(d, 'evaluate.json')
    if os.path.exists(p):
        try:
            ev = json.load(open(p, encoding='utf-8'))
        except Exception:                   # noqa: BLE001
            ev = {}
    stats = {}
    for line in reversed(txt.splitlines()):
        if '统计 {' in line:
            try:
                stats = json.loads(line[line.index('{'):])
            except Exception:               # noqa: BLE001
                pass
            break
    return txt, ev, stats


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--last', type=int, default=8)
    ap.add_argument('--prefix', default='')
    ap.add_argument('--dirs', default='')
    a = ap.parse_args()
    if a.dirs:
        dirs = [os.path.join(W, d) if not d.startswith('/') else d for d in a.dirs.split(',')]
    else:
        dirs = sorted(glob.glob(os.path.join(W, 'logs', 'round_*')))[-a.last * 3:]
        if a.prefix:
            dirs = [d for d in dirs if a.prefix in os.path.basename(d)]
        dirs = dirs[-a.last:]

    rows = []
    for d in dirs:
        txt, ev, stats = load(d)
        ready = 'ready_guard_timing' in txt
        dispatched = 'navigation goal dispatched' in txt
        startup_fail = (not ready) or ('did not become active' in txt) or ('did not become ready' in txt)
        checks = (ev.get('checks') or {})
        rows.append({
            'tag': os.path.basename(d).split('_', 2)[-1],
            'dir': os.path.basename(d),
            'verdict': ev.get('verdict', 'NA'),
            'valid': ready and dispatched,
            'startup_fail': startup_fail,
            'return_fail': ('return_home' in checks and not checks['return_home']),
            'contact_fail': ('no_contact' in checks and not checks['no_contact']),
            'reverse_fail': ('no_reverse' in checks and not checks['no_reverse']),
            'stall': ('静止超时' in txt),
            'path': stats.get('rejected_path'),
            'narrow': stats.get('rejected_narrow'),
            'size': stats.get('rejected_size'),
        })

    valid = [r for r in rows if r['valid']]
    n = len(valid) or 1
    pass_n = sum(1 for r in valid if r['verdict'] == 'PASS')
    print('轮次明细：')
    for r in rows:
        print('  %-14s 有效=%-5s 判定=%-4s 起栈失败=%-5s 返航失败=%-5s 接触=%-5s 静止=%-5s rejected_path=%s narrow=%s'
              % (r['tag'], r['valid'], r['verdict'], r['startup_fail'],
                 r['return_fail'], r['contact_fail'], r['stall'], r['path'], r['narrow']))
    print()
    print('五项指标（分母=有效轮 %d）：' % len(valid))
    print('  1) 全绿率            : %d/%d = %d%%' % (pass_n, len(valid), round(100 * pass_n / n)))
    print('  2) 返航失败          : %d/%d = %d%%' % (sum(r['return_fail'] for r in valid), len(valid),
                                                round(100 * sum(r['return_fail'] for r in valid) / n)))
    print('  3) 物理接触轮次      : %d/%d' % (sum(r['contact_fail'] for r in valid), len(valid)))
    print('  4) 静默静止轮次      : %d/%d = %d%%' % (sum(r['stall'] for r in valid), len(valid),
                                                 round(100 * sum(r['stall'] for r in valid) / n)))
    print('  5) 起栈失败轮次      : %d/%d' % (sum(r['startup_fail'] for r in rows), len(rows)))
    for k, label in (('path', 'rejected_path'), ('narrow', 'rejected_narrow'), ('size', 'rejected_size')):
        vals = [r[k] for r in valid if isinstance(r[k], (int, float))]
        if vals:
            print('  中位 %-16s: %d' % (label, statistics.median(vals)))


if __name__ == '__main__':
    main()
