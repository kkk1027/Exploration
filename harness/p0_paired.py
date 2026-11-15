#!/usr/bin/env python3
"""P0 配对 A/B 分析：同出生点、交替运行，按轮次配对比较。

为什么必须配对
--------------
`p14vS`（8 轮，固定单一出生点）显示两组均 0 接触、known 相差 0.35%，
但那一轮用的是同一个出生点、样本量只有 4。旧 `p4base`/`p4far` 十轮更是
**随机出生点**：323 次接触里有 313 集中在同一轮，组均值被单轮灾难拉爆，
根本不能做组间比较。

本脚本的思路：`/tmp/p14vT_pairs.txt` 记录每轮用的出生点，
把 far{n} 与 base{n} 配成一对（同出生点、相邻时刻运行，环境漂移同步），
只在**对内求差**，再做符号检验 / Wilcoxon。

用法
----
  python3 harness/p0_paired.py                       # 默认读 p14vT
  python3 harness/p0_paired.py --pairs /tmp/xx.txt --pattern 'p14vX'

输出
----
1. 逐轮明细表
2. 逐对差值表
3. 汇总：中位差值、符号检验 p 值、探索段干净轮数
4. **明确的结论行**：达到 / 未达到 P0 验收（探索段显著改善 + contact 不恶化）
"""
import argparse
import glob
import json
import os
import re
import statistics as st
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# digest.md 里的中文指标
RE_WAIT = re.compile(r'等候选\(waiting_for_rrt_goal\): \*\*([\d.]+) s\*\*')
RE_STALL = re.compile(r'停滞总秒数: \*\*([\d.]+) s\*\*')
RE_STUCK = re.compile(r'卡死事件（探索区间内无位移≥3 s）: \*\*(\d+)\*\*')
RE_STUCK_MOV = re.compile(r'想动却动不了\(cmd非0\): \*\*(\d+)\*\*')
RE_FAIL = re.compile(r'失败项: (.*)')


def find_round(tag):
    hits = sorted(glob.glob(os.path.join(ROOT, 'logs', 'round_*_%s' % tag)))
    return hits[-1] if hits else None


def load_round(d):
    """返回一拍的指标字典；缺文件时对应项为 None。"""
    out = {'dir': d}

    ev = os.path.join(d, 'evaluate.json')
    if os.path.exists(ev):
        with open(ev) as f:
            e = json.load(f)
        c = e.get('collision') or {}
        out['verdict'] = e.get('verdict')
        out['contacts'] = c.get('physical_contacts')
        out['reverse_explore_m'] = c.get('reverse_explore_m')
        checks = e.get('checks') or {}
        out['return_home'] = checks.get('return_home')
        out['no_contact'] = checks.get('no_contact')
        out['no_reverse'] = checks.get('no_reverse')
        out['failed'] = [k for k, v in checks.items() if not v]
        rh = e.get('return_home_kind') or {}
        out['final_dist_home'] = rh.get('final_dist_to_home')
    else:
        out.update({'verdict': None, 'contacts': None, 'failed': None})

    # 候选池：已探明面积、阶段拍数
    pool = os.path.join(d, 'cand_pool.jsonl')
    known, frame, fill, npool = [], 0, 0, []
    if os.path.exists(pool):
        with open(pool) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    r = json.loads(line)
                except Exception:
                    continue
                if r.get('known') is not None:
                    known.append(r['known'])
                ph = r.get('phase')
                if ph == 'frame':
                    frame += 1
                elif ph == 'fill':
                    fill += 1
                if r.get('n_pool') is not None:
                    npool.append(r['n_pool'])
    out['known0'] = known[0] if known else None
    out['known1'] = known[-1] if known else None
    out['known_gain'] = (known[-1] - known[0]) if len(known) >= 2 else None
    out['frame'] = frame
    out['fill'] = fill
    out['n_pool_med'] = st.median(npool) if npool else None

    # timeline：tick 耗时、仿真终点
    tl_path = os.path.join(d, 'timeline.jsonl')
    tavg, simend = [], None
    if os.path.exists(tl_path):
        with open(tl_path) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    r = json.loads(line)
                except Exception:
                    continue
                v = r.get('tick_ms_avg')
                if isinstance(v, (int, float)):
                    tavg.append(v)
                if r.get('sim') is not None:
                    simend = r['sim']
    out['tick_avg_med'] = st.median(tavg) if tavg else None
    out['sim_end'] = simend

    # digest：等候选、停滞、卡死
    dg = os.path.join(d, 'digest.md')
    if os.path.exists(dg):
        txt = open(dg, encoding='utf-8', errors='replace').read()
        for key, rx, fn in (('wait_s', RE_WAIT, float), ('stall_s', RE_STALL, float),
                            ('stuck', RE_STUCK, int), ('stuck_mov', RE_STUCK_MOV, int)):
            m = rx.search(txt)
            out[key] = fn(m.group(1)) if m else None
    return out


def sign_test(diffs):
    """双侧符号检验（忽略 0）。返回 (正数个数, 负数个数, p 值)。"""
    pos = sum(1 for d in diffs if d > 0)
    neg = sum(1 for d in diffs if d < 0)
    n = pos + neg
    if n == 0:
        return pos, neg, 1.0
    # 精确二项双侧
    from math import comb
    k = min(pos, neg)
    tail = sum(comb(n, i) for i in range(0, k + 1)) / (2 ** n)
    return pos, neg, min(1.0, 2 * tail)


def fmt(v, spec='%.3f'):
    return '-' if v is None else spec % v


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument('--pairs', default='/tmp/p14vT_pairs.txt')
    ap.add_argument('--pattern', default='p14vT')
    args = ap.parse_args(argv[1:])

    if not os.path.exists(args.pairs):
        print('缺少配对表 %s' % args.pairs)
        return 2
    pairs = []
    for line in open(args.pairs):
        parts = line.split()
        if len(parts) >= 4:
            pairs.append((parts[0], ' '.join(parts[3:])))
    if not pairs:
        print('配对表为空')
        return 2

    rows = []
    for n, pose in pairs:
        fa = find_round('%s_far%s' % (args.pattern, n))
        ba = find_round('%s_base%s' % (args.pattern, n))
        rows.append((n, pose, load_round(fa) if fa else None,
                     load_round(ba) if ba else None))

    print('## 1. 逐轮明细\n')
    hdr = ('%-4s %-5s %-6s %-7s %-8s %-8s %-7s %-6s %-6s %-6s' %
           ('轮', '组', '接触', '返航', 'known末', 'known增', 'frame', 'fill',
            'tick', '等候选'))
    print(hdr)
    print('-' * len(hdr))
    for n, pose, fa, ba in rows:
        for name, r in (('far', fa), ('base', ba)):
            if r is None:
                print('%-4s %-5s (缺失)' % (n, name))
                continue
            print('%-4s %-5s %-6s %-7s %-8s %-8s %-7s %-6s %-6s %-6s' % (
                n, name, fmt(r.get('contacts'), '%d'), str(r.get('return_home')),
                fmt(r.get('known1'), '%d'), fmt(r.get('known_gain'), '%d'),
                r.get('frame'), r.get('fill'),
                fmt(r.get('tick_avg_med'), '%.1f'), fmt(r.get('wait_s'), '%.0f')))
        print()

    print('## 2. 逐对差值（far − base）\n')
    keys = [('contacts', '接触', '%d'), ('known1', 'known末', '%d'),
            ('known_gain', 'known增', '%d'), ('wait_s', '等候选s', '%.0f'),
            ('stall_s', '停滞s', '%.0f'), ('tick_avg_med', 'tickms', '%.1f'),
            ('frame', 'frame拍', '%d'), ('fill', 'fill拍', '%d')]
    print('%-6s ' % '轮' + ' '.join('%-10s' % k[1] for k in keys))
    print('-' * (7 + 11 * len(keys)))
    diffs = {k[0]: [] for k in keys}
    for n, pose, fa, ba in rows:
        if fa is None or ba is None:
            continue
        cells = []
        for k, _label, spec in keys:
            va, vb = fa.get(k), ba.get(k)
            if isinstance(va, (int, float)) and isinstance(vb, (int, float)):
                d = va - vb
                diffs[k].append(d)
                cells.append(('+' if d > 0 else '') + (spec % d))
            else:
                cells.append('-')
        print('%-6s ' % n + ' '.join('%-10s' % c for c in cells))

    print('\n## 3. 汇总\n')
    print('%-10s %-10s %-10s %-6s %-6s %s' %
          ('指标', '中位差值', '均值差值', '正', '负', '符号检验 p'))
    print('-' * 62)
    for k, label, spec in keys:
        d = diffs[k]
        if not d:
            print('%-10s %-10s' % (label, '-'))
            continue
        pos, neg, p = sign_test(d)
        print('%-10s %-10s %-10s %-6d %-6d %.4f' % (
            label, spec % st.median(d), spec % (sum(d) / len(d)), pos, neg, p))

    def clean(r):
        """探索段干净：无接触且失败项里没有探索相关项。"""
        if r is None:
            return False
        if (r.get('contacts') or 0) > 0:
            return False
        bad = set(r.get('failed') or [])
        return not (bad & {'no_contact', 'no_reverse'})

    nf = sum(1 for _n, _p, fa, _b in rows if clean(fa))
    nb = sum(1 for _n, _p, _a, ba in rows if clean(ba))
    tf = sum(1 for _n, _p, fa, _b in rows if fa and fa.get('return_home'))
    tb = sum(1 for _n, _p, _a, ba in rows if ba and ba.get('return_home'))
    cf = sum((fa or {}).get('contacts') or 0 for _n, _p, fa, _b in rows)
    cb = sum((ba or {}).get('contacts') or 0 for _n, _p, _a, ba in rows)
    print('\n探索段干净轮数：far %d / base %d' % (nf, nb))
    print('返航成功轮数  ：far %d / base %d' % (tf, tb))
    print('接触合计      ：far %d / base %d' % (cf, cb))

    print('\n## 4. 结论判定\n')
    kd = diffs.get('known1') or []
    ok_signal = bool(kd) and st.median(kd) > 0 and sign_test(kd)[2] <= 0.10
    ok_contact = cf <= cb
    print('- 探索段覆盖改善（known末 far>base 且符号检验 p<=0.10）：%s'
          % ('达到' if ok_signal else '**未达到**'))
    print('- contact 不恶化（far 接触合计 <= base）：%s'
          % ('达到' if ok_contact else '**未达到**'))
    print('- P0 验收：%s' % ('**通过**' if (ok_signal and ok_contact) else '**未通过**'))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
