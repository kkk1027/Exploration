#!/usr/bin/env python3
"""compare_inplace_ab.py <A前缀glob> <B前缀glob> —— P12 的 A/B 对照表。

A = 现状（后置判据命中 → 取消目标 + 迁移）；B = inplace（命中 → 原地改目标，不取消）。

指标（全部来自每轮现成产物）：
  取消次数      launch.log 里 'navigation goal canceled' 的条数
  取消型迁移    'V2-③ 后置迁移'（旧路径，必然伴随一次取消）
  原地纠偏      'V2-③ 后置纠偏'（新路径，不取消）
  /goal_correction 记录条数
  无故打断      harness/analyze_interruption.py 的口径（中途未到达且计数器零增量）
  覆盖面积/占据面积/已知面积   cand_metrics.json（G1/G2）
  卡死/停滞/等候选            digest.md
  接触                        digest.md
用法：
  python3 harness/compare_inplace_ab.py 'logs/round_*_ab0r*' 'logs/round_*_ab1r*'
"""
import glob
import json
import os
import re
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_interruption import analyze_round            # noqa: E402


def jload(p):
    try:
        return json.load(open(p, encoding='utf-8'))
    except Exception:                                     # noqa: BLE001
        return {}


def digest_metrics(d):
    p = os.path.join(d, 'digest.md')
    out = {}
    if not os.path.exists(p):
        return out
    t = open(p, encoding='utf-8', errors='ignore').read()
    pats = {
        'contacts': r'物理接触: (\d+)',
        'stuck': r'卡死事件（探索区间内无位移≥3 s）: \*\*(\d+)\*\*',
        'stuck_max': r'最长 (\d+) s',
        'stall': r'停滞总秒数: \*\*(\d+) s\*\*',
        'wait': r'等候选\(waiting_for_rrt_goal\): \*\*(\d+) s\*\*',
    }
    for k, pat in pats.items():
        m = re.search(pat, t)
        if m:
            out[k] = int(m.group(1))
    return out


def group(prefix):
    rows = []
    for d in sorted(glob.glob(prefix)):
        if not os.path.isdir(d):
            continue
        lg = os.path.join(d, 'launch.log')
        txt = ''
        if os.path.exists(lg):
            txt = re.sub(r'\x1b\[[0-9;]*m', '',
                         open(lg, encoding='utf-8', errors='ignore').read())
        cm = jload(os.path.join(d, 'cand_metrics.json'))
        dg = digest_metrics(d)
        intr = analyze_round(d, 0.5, False) or {}
        rows.append(dict(
            round=os.path.basename(d),
            cancels=txt.count('navigation goal canceled'),
            migrate=txt.count('后置迁移'),
            inplace=txt.count('后置纠偏'),
            corrections=txt.count('goal_correction'),
            unexplained=intr.get('unexplained'),
            mid_way=intr.get('mid_way'),
            dispatches=intr.get('dispatches'),
            known_m2=cm.get('G1_known_m2'), free_m2=cm.get('G2_free_m2'),
            occ_m2=cm.get('G2_occ_m2'),
            stuck=dg.get('stuck'), stall=dg.get('stall'), wait=dg.get('wait'),
            contacts=dg.get('contacts')))
    return rows


def summ(rows, key):
    v = [r[key] for r in rows if isinstance(r.get(key), (int, float))]
    return (st.median(v), sum(v)) if v else (None, None)


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    A, B = group(sys.argv[1]), group(sys.argv[2])
    if not A or not B:
        print(f'组为空：A={len(A)} B={len(B)}')
        return 1
    keys = [('取消次数', 'cancels'), ('取消型迁移', 'migrate'), ('原地纠偏(新)', 'inplace'),
            ('goal_correction', 'corrections'), ('无故打断', 'unexplained'),
            ('中途更换', 'mid_way'), ('派发次数', 'dispatches'),
            ('已知面积', 'known_m2'), ('空闲面积', 'free_m2'), ('占据面积', 'occ_m2'),
            ('卡死事件', 'stuck'), ('最长卡死s', 'stuck_max'), ('停滞秒', 'stall'),
            ('等候选秒', 'wait'), ('物理接触', 'contacts')]
    print('=' * 78)
    print(f'A=现状(取消型) {len(A)} 轮     B=inplace(原地改目标) {len(B)} 轮')
    print('=' * 78)
    print(f"{'指标':<18}{'A 中位/合计':>18}{'B 中位/合计':>18}")
    for name, k in keys:
        ma, sa = summ(A, k)
        mb, sb = summ(B, k)
        f = (lambda v: '—' if v is None else (f'{v:.1f}' if isinstance(v, float) else str(v)))
        print(f'{name:<18}{f(ma)+" / "+f(sa):>18}{f(mb)+" / "+f(sb):>18}')
    print('\n逐轮：')
    for tag, rows in (('A', A), ('B', B)):
        for r in rows:
            print(f"  {tag} {r['round'][-8:]:>9}  取消={r['cancels']:2d} 迁移={r['migrate']:2d} "
                  f"纠偏={r['inplace']:2d} 记录={r['corrections']:2d} 无故打断={r['unexplained']} "
                  f"已知={r['known_m2']} 卡死={r['stuck']} 停滞={r['stall']} 接触={r['contacts']}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
