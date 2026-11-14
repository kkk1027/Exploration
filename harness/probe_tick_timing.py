#!/usr/bin/env python3
"""probe_tick_timing.py —— 从 timeline.jsonl 统计 frontier_arbiter 每拍耗时。

为什么必须先有这个（S5 §4 Step 0 的强制前置）
----------------------------------------------
S5 §5-B 的验收判据是：

    tick P95 ≤ 基线 P95 × 1.5，且 ≤ 80 ms

但改造前**根本没有任何 tick 耗时数据**（实测）：
    probe.jsonl            唯一耗时类字段是 reverse_msgs
    timeline.jsonl         无耗时字段
    candidate_probe.jsonl  无耗时字段

⇒ 如不先补，"单拍 P95 增量 ≤ 10 ms" 这类判据永远无法判定，
   Step 2 之后的所有性能验收全部作废。

报文口径（2026-11-14 起 frontier_arbiter 写入）
-----------------------------------------------
    tick_ms_prev      上一拍实测耗时（ms）。滞后 ≤ 1 s，因为 tick() 开头就
                      调 write_timeline()，本拍还不可能知道自己的耗时。
    tick_paw_prev     上一拍 path_and_width 调用次数（BFS 路径 + 测宽，性能热点）
    tick_ms_max       累计最大值
    tick_ms_avg       累计均值
    tick_n            累计已测拍数
    tick_over_budget  超过 tick_budget_ms（默认 80）的拍数

用法
----
    python3 harness/probe_tick_timing.py                  # 最新一轮
    python3 harness/probe_tick_timing.py logs/round_xxx
    python3 harness/probe_tick_timing.py --all            # 汇总所有轮（做基线）
    python3 harness/probe_tick_timing.py --freeze         # 写 runs/baseline_frozen.jsonl
    python3 harness/probe_tick_timing.py --compare runs/baseline_frozen.jsonl logs/round_yyy
"""
import argparse
import glob
import json
import os
import statistics
import sys

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BUDGET_MS = 80.0


def pct(vals, q):
    """分位数（q ∈ 0~100），样本 < 2 时退化为该值本身。"""
    if not vals:
        return None
    s = sorted(vals)
    if len(s) == 1:
        return s[0]
    k = (len(s) - 1) * (q / 100.0)
    lo, hi = int(k), min(int(k) + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def load_round(d):
    """读一轮 timeline.jsonl，返回统计字典；无字段返回 None。"""
    tl = os.path.join(d, 'timeline.jsonl')
    if not os.path.exists(tl):
        return None
    ms, paw = [], []
    for line in open(tl, encoding='utf-8', errors='replace'):
        line = line.strip()
        if not line:
            continue
        try:
            r = json.loads(line)
        except Exception:                              # noqa: BLE001
            continue
        v = r.get('tick_ms_prev')
        if isinstance(v, (int, float)):
            ms.append(float(v))
        p = r.get('tick_paw_prev')
        if isinstance(p, (int, float)):
            paw.append(int(p))
    if not ms:
        return None
    return {
        'round': os.path.basename(d),
        'n': len(ms),
        'p50': round(pct(ms, 50), 2),
        'p95': round(pct(ms, 95), 2),
        'p99': round(pct(ms, 99), 2),
        'max': round(max(ms), 2),
        'mean': round(statistics.fmean(ms), 2),
        'over_budget': sum(1 for x in ms if x > BUDGET_MS),
        'paw_p95': round(pct(paw, 95), 2) if paw else None,
        'paw_max': max(paw) if paw else None,
    }


def fmt(s):
    return ('  %-34s n=%-4d P50=%-6s P95=%-6s P99=%-6s max=%-7s mean=%-6s '
            '超限=%-3s pawP95=%-6s pawMax=%s'
            % (s['round'][6:], s['n'], s['p50'], s['p95'], s['p99'],
               s['max'], s['mean'], s['over_budget'],
               s['paw_p95'], s['paw_max']))


def rounds_with_tick():
    out = []
    for d in sorted(glob.glob(os.path.join(BASE, 'logs', 'round_*'))):
        s = load_round(d)
        if s is not None:
            out.append((d, s))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('round', nargs='?', default=None,
                    help='round 目录（默认取最新一轮）')
    ap.add_argument('--all', action='store_true', help='汇总所有含 tick 字段的轮')
    ap.add_argument('--freeze', action='store_true',
                    help='把所有轮的统计写入 runs/baseline_frozen.jsonl')
    ap.add_argument('--compare', nargs=2, metavar=('BASELINE', 'ROUND'),
                    help='对照基线：判据 = P95 ≤ 基线 P95×1.5 且 ≤ 80 ms')
    a = ap.parse_args()

    if a.compare:
        bl_path, rd = a.compare
        base = None
        for line in open(bl_path, encoding='utf-8'):
            line = line.strip()
            if not line:
                continue
            r = json.loads(line)
            if r.get('p95') is not None:
                base = (base or [])
                base.append(r)
        if not base:
            print('✗ 基线文件里没有可用记录', file=sys.stderr)
            return 3
        bp95 = pct([r['p95'] for r in base], 50)
        cur = load_round(rd) if os.path.isdir(rd) else None
        if cur is None:
            print('✗ %s 没有 tick 数据' % rd, file=sys.stderr)
            return 3
        limit = min(bp95 * 1.5, BUDGET_MS)
        ok = cur['p95'] <= limit
        print('[tick 对照] 基线 P95(中位)=%.2f ms  上限=min(×1.5, %.0f)=%.2f ms'
              % (bp95, BUDGET_MS, limit))
        print(fmt(cur))
        print('  判定: %s（当前 P95=%.2f）' % ('PASS' if ok else 'FAIL', cur['p95']))
        return 0 if ok else 1

    if a.all or a.freeze:
        rows = rounds_with_tick()
        if not rows:
            print('✗ 没有任何轮次带 tick 字段。frontier_arbiter 需先跑过一轮。',
                  file=sys.stderr)
            return 3
        print('[tick 耗时] 共 %d 轮带数据' % len(rows))
        for _d, s in rows:
            print(fmt(s))
        allp95 = [s['p95'] for _d, s in rows]
        print('  轮级 P95 的中位 = %.2f ms，最大 = %.2f ms'
              % (pct(allp95, 50), max(allp95)))
        if a.freeze:
            outdir = os.path.join(BASE, 'runs')
            os.makedirs(outdir, exist_ok=True)
            path = os.path.join(outdir, 'baseline_frozen.jsonl')
            with open(path, 'w', encoding='utf-8') as f:
                for _d, s in rows:
                    f.write(json.dumps(s, ensure_ascii=False) + '\n')
            print('  ✓ 已写入 %s（%d 行）' % (path, len(rows)))
        return 0

    d = a.round
    if d is None:
        cand = [x for x in sorted(glob.glob(os.path.join(BASE, 'logs', 'round_*')))
                if os.path.exists(os.path.join(x, 'timeline.jsonl'))]
        if not cand:
            print('✗ 找不到 round 目录', file=sys.stderr)
            return 3
        d = cand[-1]
    s = load_round(d)
    if s is None:
        print('✗ %s 的 timeline.jsonl 里没有 tick_ms_prev 字段。'
              '\n  说明该轮是在 2026-11-14 探针加入之前跑的，需要重跑。' % d,
              file=sys.stderr)
        return 3
    print('[tick 耗时]')
    print(fmt(s))
    return 0


if __name__ == '__main__':
    sys.exit(main())
