#!/usr/bin/env python3
"""p13_verdict_dist.py —— 判决分布表（论文新 Fig.2 的数据源）。

为什么需要它
------------
E6 跑批中发现的核心现象：**放宽一个硬门，拒绝会整体转移到下一个硬门**。

  e3 (raw_min_esc_dirs=3): dead_end 158, too_narrow 56, unreachable 4  → feasible 0
  e0 (raw_min_esc_dirs=0): dead_end   0, too_narrow 114, unreachable 100 → feasible 0

把 dead_end 归零之后，too_narrow 与 unreachable 顶上来了，`feasible` 仍然是 0。
所以「判据无区分度」不是某个阈值太严，而是**所有硬门联合拒绝了 100% 的候选**。

单看某一个计数器看不出这件事 —— 必须把五类判决并排放在一起看。
本脚本就做这件事。

口径
----
从 `launch.log` 里取 arbiter 打印的**最后一次累计统计块**（`{'seen': ...}`）。
这些是累计值，所以每轮最后一行就是该轮的最终分布。
`verdict_dead_end` 等计数在 `_record_verdict` 里累加，**判决翻转时会重复计数**，
所以它们的和会大于 `verdict_pub`（去重后的实际发布数）。
本脚本同时给出两者，并显式说明差异，避免被误读成 bug。

用法
----
  python3 harness/p13_verdict_dist.py 'logs/round_*_p13e6_*'
  python3 harness/p13_verdict_dist.py 'logs/round_*_p13e6_*' --json /tmp/vd.json
"""
import argparse
import ast
import glob
import os
import re
import statistics as st
import sys

import analyze_dispatch_risk as adr

VERDICTS = ('feasible', 'dead_end', 'too_narrow', 'unreachable', 'over_dist')
RE_BLOCK = re.compile(r"\{'seen':.*?\}")


def _tail_stats(round_dir):
    """取 launch.log 里最后一个累计统计块。"""
    p = os.path.join(round_dir, 'launch.log')
    if not os.path.exists(p):
        return None
    txt = open(p, encoding='utf-8', errors='ignore').read()
    blocks = RE_BLOCK.findall(txt)
    if not blocks:
        return None
    try:
        return ast.literal_eval(blocks[-1])
    except Exception:                              # noqa: BLE001
        return None


def _best_dirs(pattern):
    """按 (臂, 轮号) 去重，取 timeline 行数最大的一份；并剔除无效轮。"""
    best = {}
    for d in sorted(glob.glob(pattern)):
        tail = os.path.basename(d.rstrip('/')).rsplit('_', 1)[-1]
        m = re.match(r'^(.*?)(\d)$', tail)
        arm, rnd = (m.group(1), m.group(2)) if m else (tail, '0')
        t = os.path.join(d, 'timeline.jsonl')
        n = 0
        if os.path.exists(t):
            try:
                with open(t, encoding='utf-8', errors='ignore') as f:
                    n = sum(1 for _ in f)
            except Exception:                      # noqa: BLE001
                n = 0
        k = (arm, rnd)
        if k not in best or n > best[k][0]:
            best[k] = (n, d)
    out, skipped = [], []
    for k, (n, d) in sorted(best.items()):
        if n < 100:
            skipped.append((k, 'timeline=%d' % n))
            continue
        ok, why = adr.round_is_valid(d)
        if not ok:
            skipped.append((k, why))
            continue
        out.append((k, d))
    if skipped:
        print('⚠ 剔除 %d 个无效轮：%s'
              % (len(skipped), ', '.join('%s%s(%s)' % (k[0], k[1], w)
                                         for k, w in skipped)))
    return out


def main():
    ap = argparse.ArgumentParser(description='判决分布（跨臂对比）')
    ap.add_argument('pattern')
    ap.add_argument('--json', default='')
    a = ap.parse_args()

    rows = _best_dirs(a.pattern)
    if not rows:
        print('没有可用轮次', file=sys.stderr)
        return 2

    per_arm = {}
    for (arm, rnd), d in rows:
        s = _tail_stats(d)
        if not s:
            continue
        per_arm.setdefault(arm, []).append((rnd, d, s))

    print('=' * 104)
    print('判决分布 · %d 臂 · %d 轮（每臂取逐轮中位）' % (len(per_arm), len(rows)))
    print('=' * 104)
    head = ('%-6s %4s %8s ' % ('臂', '轮', 'raw_seen')
            + ' '.join('%10s' % v for v in VERDICTS)
            + ' %9s %9s %8s' % ('负判决和', 'pub(去重)', 'feasible%'))
    print(head)
    print('-' * 104)

    out = {}
    for arm in sorted(per_arm):
        items = per_arm[arm]
        def med(key):
            v = [int(s.get(key) or 0) for _r, _d, s in items]
            return round(st.median(v), 1)
        vals = {v: med('verdict_' + v) for v in VERDICTS}
        neg = sum(vals[v] for v in VERDICTS if v != 'feasible')
        pub = med('verdict_pub')
        raw_seen = med('raw_seen')
        feas_pct = (100.0 * vals['feasible'] / neg) if neg else 0.0
        out[arm] = {'n': len(items), 'raw_seen': raw_seen, 'pub_dedup': pub,
                    'verdicts': vals, 'neg_sum': neg,
                    'feasible_pct': round(feas_pct, 1),
                    'raw_publish': med('raw_publish')}
        print('%-6s %4d %8s ' % (arm, len(items), raw_seen)
              + ' '.join('%10s' % vals[v] for v in VERDICTS)
              + ' %9s %9s %7.1f%%' % (neg, pub, feas_pct))

    print('\n说明')
    print('  * 各 verdict_* 是**判决事件**计数，判决翻转会重复计，故其和 ≥ pub(去重)。')
    print('  * feasible% = feasible / 负判决和，即"有多少判决是放行"。')
    print('  * 若所有臂 feasible% ≈ 0 而负判决和 > 0，说明判据无区分度，')
    print('    契约退化为纯黑名单 —— 这正是 E6 要验证的核心命题。')

    # 关键对照：如果某臂把 dead_end 压到 0 但别的门顶上来了
    arms = sorted(out)
    shifted = []
    for i in range(len(arms)):
        for j in range(i + 1, len(arms)):
            A, B = out[arms[i]]['verdicts'], out[arms[j]]['verdicts']
            if A['dead_end'] != B['dead_end']:
                shifted.append((arms[i], arms[j], A, B))
    if shifted:
        print('\n判决转移检视（某个门被放宽后，拒绝是否转移到了别的门）')
        for an, bn, A, B in shifted:
            print('  %s vs %s:' % (an, bn))
            for v in VERDICTS:
                if A[v] != B[v]:
                    print('    %-12s %8s → %8s  (%+g)'
                          % (v, A[v], B[v], B[v] - A[v]))

    if a.json:
        os.makedirs(os.path.dirname(a.json) or '.', exist_ok=True)
        with open(a.json, 'w', encoding='utf-8') as f:
            import json
            json.dump(out, f, ensure_ascii=False, indent=1)
        print('\nJSON → %s' % a.json)
    return 0


if __name__ == '__main__':
    sys.exit(main())
