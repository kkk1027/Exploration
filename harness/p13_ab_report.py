#!/usr/bin/env python3
"""p13_ab_report.py —— P13 A/B 对比报告（只读）。

输入：两组 round 目录（基线组 / 实验组）
输出：逐轮表 + 汇总对比 + 逐项判定，并给出"该回退哪个提交"的建议。

判定项与 docs/P13_实施说明.md §7 完全一致：
  派发距离中位、>6 m 占比、取消率（剔除幻影）、幻影率、脱困转化、覆盖率、S0 接触

用法：
  python3 harness/p13_ab_report.py 'logs/round_*_a*' 'logs/round_*_b*'
  python3 harness/p13_ab_report.py 'logs/round_*_a*' 'logs/round_*_b*' --json /tmp/ab.json
"""
import argparse
import glob
import json
import os
import statistics as st
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import analyze_dispatch_risk as A          # noqa: E402


def load(patterns):
    dirs, seen = [], set()
    for pat in patterns:
        for p in sorted(glob.glob(pat)) or [pat]:
            rp = os.path.realpath(p)
            if os.path.isdir(p) and rp not in seen:
                seen.add(rp)
                dirs.append(p)
    rows, per_round = [], []
    for d in dirs:
        disp, _err, cnt = A.parse_ledger(d)
        if not disp:
            continue
        A.attach_features(d, disp)
        rows.extend(disp)
        merge = A._merge_ratio(d)
        per_round.append({
            '_dir': d,
            'round': os.path.basename(d), 'disp': cnt['disp'],
            'real': cnt['real_reach'], 'phantom': cnt['phantom'],
            'canceled': cnt['canceled'], 'merge': merge,
        })
    return dirs, rows, per_round


def summarize(rows, per_round):
    n = len(rows)
    known = [r for r in rows if r['label'] in ('reached', 'canceled', 'failed')]
    phantom = [r for r in rows if r.get('phantom')]
    eff = [r for r in known if not r.get('phantom')]
    canc = [r for r in eff if r['label'] == 'canceled']
    dists = [r['dist'] for r in rows]
    turns = [r['turn'] for r in rows]
    esc = [r for r in rows if r.get('esc_dirs') is not None
           and r['label'] in ('reached', 'canceled', 'failed')
           and not r.get('phantom')]
    return {
        'rounds': len(per_round), 'disp': n, 'paired': len(known),
        'phantom': len(phantom),
        'phantom_rate': round(100.0 * len(phantom) / max(1, len(known)), 1),
        'real': len(known) - len(phantom) - len(canc) - sum(
            1 for r in eff if r['label'] == 'failed'),
        'canceled': len(canc),
        'cancel_rate': round(100.0 * len(canc) / max(1, len(eff)), 1),
        'dist_med': round(st.median(dists), 2) if dists else None,
        'dist_gt4': round(100.0 * sum(1 for x in dists if x > 4) / n, 1) if n else None,
        'dist_gt6': round(100.0 * sum(1 for x in dists if x > 6) / n, 1) if n else None,
        'turn_med': round(st.median(turns)) if turns else None,
        'turn_gt150': round(100.0 * sum(1 for x in turns if x > 150) / n, 1) if n else None,
        'esc_med': round(st.median([r['esc_dirs'] for r in esc]), 1) if esc else None,
        'score_cover': round(st.median(
            [pr['merge']['score_cover'] for pr in per_round
             if pr['merge'].get('score_cover') is not None] or [0]), 1),
        'escape_goal_per_round': round(
            sum(A.count_escape_goals(pr['_dir']) for pr in per_round)
            / max(1, len(per_round)), 2),
    }


def main():
    ap = argparse.ArgumentParser(description='P13 A/B 对比')
    ap.add_argument('base_pat', nargs='+', help='基线组 round 目录')
    ap.add_argument('test_pat', nargs='+', help='实验组 round 目录')
    ap.add_argument('--json', default='')
    a = ap.parse_args()

    bdirs, brows, brounds = load(a.base_pat)
    tdirs, trows, trounds = load(a.test_pat)
    if not brows or not trows:
        print('两组都必须有派发记录；基线 %d 条 / 实验 %d 条'
              % (len(brows), len(trows)), file=sys.stderr)
        return 2

    B = summarize(brows, brounds)
    T = summarize(trows, trounds)

    print('=' * 76)
    print('P13 A/B 对比报告')
    print('=' * 76)
    print('基线组 %d 轮：%s' % (len(bdirs), ', '.join(os.path.basename(d) for d in bdirs)))
    print('实验组 %d 轮：%s' % (len(tdirs), ', '.join(os.path.basename(d) for d in tdirs)))

    KEYS = [
        ('disp', '派发次数', 'info', None),
        ('phantom_rate', '幻影成功率 %', 'lower', 0.1),
        ('cancel_rate', '取消率 %（剔幻影）', 'lower', None),
        ('dist_med', '派发距离中位 m', 'lower', 4.0),
        ('dist_gt6', '>6 m 占比 %', 'lower', 18.0),
        ('turn_med', '派发转角中位 °', 'lower', None),
        ('turn_gt150', '>150° 占比 %', 'lower', 20.0),
        ('score_cover', '评分覆盖率 %', 'higher', 85.0),
        ('escape_goal_per_round', '脱困目标 次/轮', 'higher', 1.0),
    ]
    print('\n%-26s %12s %12s   %s' % ('指标', '基线', '实验', '判定'))
    print('-' * 76)
    verdicts = {}
    for key, label, better, target in KEYS:
        b, t = B.get(key), T.get(key)
        if b is None or t is None:
            verdicts[key] = 'n/a'
            print('%-26s %12s %12s   %s' % (label, b, t, 'n/a'))
            continue
        if better == 'info':
            verdicts[key] = 'info'
            print('%-26s %12s %12s   %s' % (label, b, t, '参考'))
            continue
        if target is None:
            ok = (t < b) if better == 'lower' else (t > b)
        else:
            # 有绝对目标时必须按方向判：'lower' 是"不超过"，'higher' 是"不低于"。
            # （首版漏了这条：'higher' 也走了 t <= target，导致 36 < 85 被判成 PASS）
            ok = (t <= target) if better == 'lower' else (t >= target)
        verdicts[key] = 'PASS' if ok else 'FAIL'
        mark = '\u2713' if ok else '\u2717'
        tgt = (' 目标 %.1f' % target) if target is not None else ''
        print('%-26s %12s %12s   %s%s' % (label, b, t, mark, tgt))

    print('\n逐轮幻影/取消：')
    print('  %-34s %6s %7s %8s %9s' % ('round', 'disp', 'phantom', 'cancel', 'cover%'))
    for tag, rounds in (('BASE', brounds), ('TEST', trounds)):
        for r in rounds:
            print('  %-4s %-29s %6d %7d %8d %9s'
                  % (tag, r['round'][-29:], r['disp'], r['phantom'],
                     r['canceled'], r['merge'].get('score_cover')))

    # ---- 结论与回退建议 ----
    fails = [k for k, v in verdicts.items() if v == 'FAIL']
    print('\n' + '=' * 76)
    if not fails:
        print('结论：PASS —— 全部判据达标，可以保留 P13-1 / P13-2A / P13-4a')
    else:
        print('结论：FAIL —— 未达标项：%s' % ', '.join(fails))
        print('\n回退建议（按未达标项定位，单独 revert，不要整批回退）：')
        if any(k in fails for k in ('dist_med', 'dist_gt6', 'turn_med', 'turn_gt150')):
            print('  距离/转角未改善 → git revert 1b37500   # P13-1 风险评分')
        if 'score_cover' in fails:
            print('  评分覆盖率未提升 → git revert fd3464c   # P13-2A raw 候选评分')
        if 'cancel_rate' in fails:
            print('  取消率未下降 → 先查 risk_dist_half_m（调大=放松），'
                  '仍不行再 git revert 1b37500')
        if 'phantom_rate' in fails:
            print('  幻影率回升 → ⚠ 说明 BT 又出现了 GoalUpdater 污染，'
                  '检查 stemm_cartographer_nav_to_pose.xml')
        if B.get('esc_med') is not None and T.get('esc_med') is not None:
            print('  参考：落点可脱困方向数 基线 %.1f → 实验 %.1f'
                  % (B['esc_med'], T['esc_med']))

    if a.json:
        os.makedirs(os.path.dirname(a.json) or '.', exist_ok=True)
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump({'base': B, 'test': T, 'verdicts': verdicts,
                       'rounds': {'base': brounds, 'test': trounds}},
                      f, ensure_ascii=False, indent=1)
        print('\nJSON → %s' % a.json)
    return 0 if not fails else 1


if __name__ == '__main__':
    sys.exit(main())
