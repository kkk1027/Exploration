#!/usr/bin/env python3
"""summarize_explore_baseline.py —— 把一轮轮的产物汇总成"探索建图基线"体检表。

面向用户要求的两个重点：
  **建图记忆性**：地图记了多少、记得准不准、漏了什么、被重新标回未知没有
  **趋向性**  ：派发出去的候选有多"新"、多"近"、转身多大、重复率多高、被打断多少

数据来源（全部是每轮现成产物，不额外跑仿真）：
  evaluate.json     verdict / 接触 / vision 摘要
  cand_metrics.json S1..S6 供给链、C1..C5 趋向性、G1..G6 建图与运动
  digest.md         卡死/停滞/等候选/派发来源（解析文本）
  map.pgm/.yaml     地图本体（交给 harness/score_round_map.py 用真值打分）
  vision_grid.pgm   视觉累积栅格（可选）
  logs/interruption.json（由 harness/analyze_interruption.py 生成，可选并入）

用法：
  python3 harness/summarize_explore_baseline.py --glob 'logs/round_*_m5r*' [--json out.json]
"""
import argparse
import glob
import json
import os
import re
import statistics as st
import sys


def jload(p):
    try:
        return json.load(open(p, encoding='utf-8'))
    except Exception:                                        # noqa: BLE001
        return {}


def parse_digest(p):
    """从 digest.md 抽关键行（稳妥：抽不到就是 None）。"""
    out = {}
    if not os.path.exists(p):
        return out
    txt = open(p, encoding='utf-8', errors='ignore').read()
    pats = {
        'verdict': r'verdict: \*\*(\w+)\*\*',
        'contacts': r'物理接触: (\d+)',
        'reverse_m': r'倒车: ([\d.]+) m',
        'stuck': r'卡死事件（探索区间内无位移≥3 s）: \*\*(\d+)\*\*',
        'stuck_cmd': r'"想动却动不了\(cmd非0\)": \*\*(\d+)\*\*',
        'stuck_max_s': r'最长 (\d+) s',
        'stall_total_s': r'停滞总秒数: \*\*(\d+) s\*\*',
        'stall_ge30': r'≥30 s: \*\*(\d+)\*\*',
        'wait_rrt_s': r'等候选\(waiting_for_rrt_goal\): \*\*(\d+) s\*\*',
        'wait_rrt_max': r'最长一段 \*\*(\d+) s\*\*',
    }
    for k, pat in pats.items():
        m = re.search(pat, txt)
        if not m:
            continue
        v = m.group(1)
        if k == 'verdict':          # 文本字段，别当数字
            out[k] = v
            continue
        try:
            out[k] = float(v) if '.' in v else int(v)
        except ValueError:
            out[k] = v
    m = re.search(r'派发来源: (.+)', txt)
    if m:
        out['dispatch_sources'] = m.group(1).strip()
    return out


def collect(d):
    ev = jload(os.path.join(d, 'evaluate.json'))
    cm = jload(os.path.join(d, 'cand_metrics.json'))
    dg = parse_digest(os.path.join(d, 'digest.md'))
    row = dict(round=os.path.basename(d))
    row['verdict'] = dg.get('verdict') or ev.get('verdict')
    # —— 建图 ——
    for k_src, k_dst in (('G1_known_m2', 'known_m2'), ('G2_free_m2', 'free_m2'),
                         ('G2_occ_m2', 'occ_m2'), ('G6_grids_1m', 'grids_1m'),
                         ('G3_traverse_m', 'traverse_m'), ('G5_dur_s', 'dur_s')):
        if k_src in cm:
            row[k_dst] = cm[k_src]
    row.update({k: ev.get('vision', {}).get(k) for k in (ev.get('vision') or {})})
    # —— 趋向性（候选供给与选择）——
    for k in ('S1_cands_seen', 'S2_cands_unique', 'S3_cands_published',
              'S4_reject_ticks_per_cand', 'S5_repair_ratio', 'S6_cands_per_dispatch',
              'C1_novelty_2m', 'C2_known_frac_2m', 'C3_repeat_ratio',
              'C4_dispatch_dist_med', 'C4_dispatch_dist_max', 'C5_turn_deg_med',
              'plan_fail_iters', 'plan_fail_novalid'):
        if k in cm:
            row[k] = cm[k]
    if isinstance(cm.get('reject_detail'), dict):
        row['reject_detail'] = cm['reject_detail']
    # —— 安全/效率 ——
    for k in ('contacts', 'reverse_m', 'stuck', 'stuck_cmd', 'stuck_max_s',
              'stall_total_s', 'stall_ge30', 'wait_rrt_s', 'wait_rrt_max'):
        if k in dg:
            row[k] = dg[k]
    if dg.get('dispatch_sources'):
        row['dispatch_sources'] = dg['dispatch_sources']
    return row


def med(rows, k):
    v = [r[k] for r in rows if isinstance(r.get(k), (int, float))]
    return st.median(v) if v else None


def show(rows, k, name, fmt='{:.2f}', unit=''):
    m = med(rows, k)
    if m is None:
        return
    vals = ' / '.join(fmt.format(r[k]) for r in rows if isinstance(r.get(k), (int, float)))
    print(f'  {name:<28s} 中位 {fmt.format(m)}{unit:<3s}  各轮 [{vals}]')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--glob', default='logs/round_*')
    ap.add_argument('--json', default='')
    a = ap.parse_args()
    dirs = [d for d in sorted(glob.glob(a.glob)) if os.path.isdir(d)]
    rows = [collect(d) for d in dirs]
    rows = [r for r in rows if r.get('verdict') or r.get('S1_cands_seen')]
    if not rows:
        print('没有可用轮次')
        return 1

    print('=' * 96)
    print(f'探索建图基线体检（{len(rows)} 轮）')
    print('=' * 96)
    print('\n【安全 / 效率】')
    show(rows, 'contacts', '物理接触', '{:.0f}', ' 次')
    show(rows, 'stuck', '卡死事件', '{:.0f}', ' 次')
    show(rows, 'stuck_cmd', '  其中想动动不了', '{:.0f}', ' 次')
    show(rows, 'stuck_max_s', '最长卡死', '{:.0f}', ' s')
    show(rows, 'stall_total_s', '停滞总秒', '{:.0f}', ' s')
    show(rows, 'wait_rrt_s', '等候选总秒', '{:.0f}', ' s')
    show(rows, 'wait_rrt_max', '等候选最长', '{:.0f}', ' s')
    show(rows, 'traverse_m', '行驶里程', '{:.1f}', ' m')
    show(rows, 'dur_s', '探索时长', '{:.0f}', ' s')

    print('\n【建图：规模与记忆】')
    show(rows, 'known_m2', '已知面积', '{:.1f}', ' m²')
    show(rows, 'free_m2', '空闲面积', '{:.1f}', ' m²')
    show(rows, 'occ_m2', '占据面积', '{:.1f}', ' m²')
    show(rows, 'grids_1m', '1 m 栅格数', '{:.0f}', '')

    print('\n【趋向性：候选供给与选择】')
    show(rows, 'S1_cands_seen', '观察到的候选', '{:.0f}', ' 个')
    show(rows, 'S2_cands_unique', '去重候选', '{:.0f}', ' 个')
    show(rows, 'S3_cands_published', '实际发布', '{:.0f}', ' 次')
    show(rows, 'S6_cands_per_dispatch', '每次派发候选数', '{:.2f}', '')
    show(rows, 'C1_novelty_2m', '候选 2 m 内未知格数', '{:.0f}', ' 格')
    show(rows, 'C2_known_frac_2m', '候选 2 m 内已知比', '{:.3f}', '')
    show(rows, 'C3_repeat_ratio', '重复率', '{:.3f}', '')
    show(rows, 'C4_dispatch_dist_med', '派发距离中位', '{:.2f}', ' m')
    show(rows, 'C4_dispatch_dist_max', '派发距离最大', '{:.2f}', ' m')
    show(rows, 'C5_turn_deg_med', '派发转身中位', '{:.0f}', ' °')
    show(rows, 'S4_reject_ticks_per_cand', '每个候选被拒 tick', '{:.1f}', '')

    print('\n【各轮 verdict / 派发来源】')
    for r in rows:
        print(f"  {r['round'][-24:]:26s} {r.get('verdict')}  {r.get('dispatch_sources', '')}")

    if a.json:
        json.dump(rows, open(a.json, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
        print(f'\n输出 {a.json}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
