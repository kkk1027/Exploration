#!/usr/bin/env python3
"""p13_stats.py —— A/B 组间差异的配对 bootstrap 置信区间。

为什么必须做
------------
`p13d` 的逐轮覆盖率：A 组 41.5/46.8/40.2/33.3/26.2，B 组 50.0/40.5/19.8/32.6/47.0。
组内极差达 20~30 pt，且分布明显非正态（有小样本离群）。
直接报「均值 ± 标准差」会把噪声写成结论，评审一眼就能看穿。

做法
----
* **配对**：`p13_ab.sh` 是交错执行（a1 b1 a2 b2 …），第 i 轮天然配对。
  配对能消掉「机器冷热/磁盘缓存/仿真随机种子漂移」这层共同噪声。
* **bootstrap**：对「配对差值」做 1000 次有放回重采样，取 2.5%/97.5% 分位。
* **报告差值的中位 + CI**，而不是两组的均值。若 CI 跨 0，判定为「无显著差异」。

用法
----
  python3 harness/p13_stats.py 'logs/round_*_p13f_a*' 'logs/round_*_p13f_b*'
  python3 harness/p13_stats.py A_GLOB B_GLOB --metric dist_med --metric cancel_rate
  python3 harness/p13_stats.py A_GLOB B_GLOB --json /tmp/stats.json

默认指标取自三个分析脚本的口径，全部现算，不依赖中间文件。
"""
import argparse
import glob
import json
import math
import os
import random
import statistics as st
import sys

import analyze_dispatch_risk as adr
import analyze_efficiency as aef


def _dirs(pats):
    out = []
    seen = set()
    for p in pats:
        got = sorted(glob.glob(p)) or ([p] if os.path.isdir(p) else [])
        for d in got:
            rp = os.path.realpath(d)
            if rp in seen:
                continue
            seen.add(rp)
            out.append(d)
    return out


def _tag(d):
    """取轮次标签（如 p13f_a1），用于按轮号配对。"""
    n = os.path.basename(d.rstrip('/'))
    if '_' in n:
        return n.rsplit('_', 1)[-1]
    return n


def collect(dirs):
    """每轮算一组指标，返回 {tag: {metric: value}}。"""
    import collections
    out = collections.OrderedDict()
    for d in dirs:
        m = {}
        try:
            m.update(_dispatch_metrics(d))
        except Exception as exc:                   # noqa: BLE001
            print('  ! %s 派发指标失败: %s' % (_tag(d), exc), file=sys.stderr)
        try:
            eff = aef.efficiency_one(d)
            if eff:
                m['path_m'] = eff['path_m']
                m['move_pct'] = eff['move_pct']
                m['known_cells'] = eff['known_cells']
        except Exception as exc:                   # noqa: BLE001
            print('  ! %s 效率指标失败: %s' % (_tag(d), exc), file=sys.stderr)
        out[_tag(d)] = m
    return out


def _dispatch_metrics(d):
    """复用 analyze_dispatch_risk 的内部函数，算出单轮的派发质量指标。

    刻意不 import 它的 main()：main 会打印整份报表，在批量统计里是噪音。
    所以这里直接调用它暴露的纯函数（parse_ledger / attach_features /
    _merge_ratio / _verdict_report），口径与报表**完全一致**。
    """
    disp, _errs, cnt = adr.parse_ledger(d)
    if not disp:
        return {}
    adr.attach_features(d, disp)
    known = cnt['reached'] + cnt['canceled'] + cnt['failed']
    dists = [x['dist'] for x in disp]
    turns = [x['turn'] for x in disp]
    dm, _dp90, _dmax = adr._stat(dists)
    tm, _tp90, _tmax = adr._stat(turns)
    merge = adr._merge_ratio(d)
    verdict = adr._verdict_report(d)
    m = {
        'disp': cnt['disp'],
        'cancel_rate': round(adr._pct(cnt['canceled'], known), 1),
        'phantom_rate': round(adr._pct(cnt['phantom'], max(1, known)), 1),
        'dist_med': dm,
        'dist_gt4_pct': round(adr._pct(sum(1 for x in dists if x > 4), len(dists)), 1),
        'dist_gt6_pct': round(adr._pct(sum(1 for x in dists if x > 6), len(dists)), 1),
        'turn_med': tm,
        'turn_gt150_pct': round(adr._pct(sum(1 for x in turns if x > 150),
                                         len(turns)), 1),
        'score_cover': merge.get('score_cover'),
        'vbr': verdict.get('vbr'),
        'v_block': verdict.get('v_block'),
        'v_force': verdict.get('v_force'),
    }
    # 落点可脱困方向数（只有 probe 特征才准；全 none 时留空而不是填 0）
    esc = [x['esc_dirs'] for x in disp if x.get('esc_dirs') is not None]
    m['esc_med'] = round(st.median(esc), 1) if esc else None
    return m


def pair_stats(a_vals, b_vals, n_boot=1000, seed=12345):
    """配对 bootstrap。返回 dict。"""
    pairs = [(a, b) for a, b in zip(a_vals, b_vals)
             if a is not None and b is not None]
    if len(pairs) < 3:
        return None
    diffs = [b - a for a, b in pairs]
    rnd = random.Random(seed)
    boot = []
    n = len(diffs)
    for _ in range(n_boot):
        s = [diffs[rnd.randrange(n)] for _ in range(n)]
        boot.append(st.median(s))
    boot.sort()
    lo = boot[int(0.025 * n_boot)]
    hi = boot[int(0.975 * n_boot) - 1]
    return {'n': n, 'med_diff': round(st.median(diffs), 3),
            'ci_lo': round(lo, 3), 'ci_hi': round(hi, 3),
            'sig': (lo > 0) or (hi < 0),
            'a_med': round(st.median([a for a, _ in pairs]), 3),
            'b_med': round(st.median([b for _, b in pairs]), 3),
            'diffs': [round(x, 3) for x in diffs]}


def main():
    ap = argparse.ArgumentParser(description='A/B 配对 bootstrap 置信区间')
    ap.add_argument('a_pat', nargs='+')
    ap.add_argument('b_pat', nargs='+')
    ap.add_argument('--metric', action='append', default=[])
    ap.add_argument('--json', default='')
    ap.add_argument('--boot', type=int, default=1000)
    a = ap.parse_args()

    adirs = _dirs(a.a_pat)
    bdirs = _dirs(a.b_pat)
    if not adirs or not bdirs:
        print('两组都必须有 round 目录', file=sys.stderr)
        return 2
    print('A 组 %d 轮：%s' % (len(adirs), ', '.join(_tag(d) for d in adirs)))
    print('B 组 %d 轮：%s' % (len(bdirs), ', '.join(_tag(d) for d in bdirs)))

    A = collect(adirs)
    B = collect(bdirs)
    # 按轮号配对：a1↔b1，a2↔b2
    ka = list(A.keys())
    kb = list(B.keys())
    metrics = a.metric or ['dist_med', 'cancel_rate', 'turn_med',
                           'dist_gt6_pct', 'score_cover', 'vbr',
                           'path_m', 'move_pct', 'known_cells']
    print('\n%-16s %10s %10s %12s %20s %s'
          % ('指标', 'A中位', 'B中位', '差(B-A)', '95%CI', '判定'))
    print('-' * 88)
    out = {}
    for m in metrics:
        av = [A[k].get(m) for k in ka]
        bv = [B[k].get(m) for k in kb]
        if all(v is None for v in av + bv):
            continue
        s = pair_stats(av, bv, n_boot=a.boot)
        if s is None:
            print('%-16s %10s %10s %12s %20s %s'
                  % (m, '-', '-', '-', '-', '样本不足(<3 对)'))
            continue
        verdict = ('显著' if s['sig'] else '无显著差异')
        print('%-16s %10.2f %10.2f %+12.2f   [%+7.2f, %+7.2f]  %s'
              % (m, s['a_med'], s['b_med'], s['med_diff'],
                 s['ci_lo'], s['ci_hi'], verdict))
        out[m] = s

    n_ok = sum(1 for v in out.values() if v['sig'])
    print('\n%d/%d 个指标 CI 不跨 0（显著）。'
          % (n_ok, len(out)))
    print('注：bootstrap 只说明「差异是否超出组内波动」，不说明差异是否有实际意义。')
    if a.json:
        os.makedirs(os.path.dirname(a.json) or '.', exist_ok=True)
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump({'A': {k: v for k, v in A.items()},
                       'B': {k: v for k, v in B.items()},
                       'stats': out}, f, ensure_ascii=False, indent=1)
        print('JSON → %s' % a.json)
    return 0


if __name__ == '__main__':
    sys.exit(main())
