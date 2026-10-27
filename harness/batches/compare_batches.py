#!/usr/bin/env python3
"""compare_batches.py —— 把多批 round 结果拉平成一张对照表（A/B 用）。

用法：
  python3 harness/batches/compare_batches.py base after            # 前缀= base / after
  python3 harness/batches/compare_batches.py --out logs/cmp.md base after

数据来源（每个 round 目录）：digest.md、evaluate.json、timeline.jsonl、
costmap/summary.json。缺失的字段填 "-"，不猜。
"""
import argparse
import glob
import json
import os
import re
import sys

ROOT = '/home/kbf/stemm_exploration_sim_ws'

RE_VERDICT = re.compile(r'verdict: \*\*(\w+)\*\*　失败项: (\[[^\]]*\]|无)')
RE_COLL = re.compile(r'物理接触: (\d+)　倒车: ([\d.]+) m（探索段 ([\d.]+) / 返航受控 ([\d.]+)）')
RE_STUCK = re.compile(r'卡死事件（[^）]*）: \*\*(\d+)\*\*　其中"想动却动不了\(cmd非0\)": \*\*(\d+)\*\*　最长 ([\d.]+) s')
RE_DISP = re.compile(r'派发来源: (.+)')
RE_CONT = re.compile(r'接触段/时长: (\d+) 段 / ([\d.]+) s')
RE_STALLSET = re.compile(r'停滞总秒数: \*\*([\d.]+) s\*\*　≥30 s: \*\*(\d+)\*\* 条')
RE_MISC = re.compile(r'（本轮"前沿全灭→脱困目标"触发 (\d+) 次；时间线采样 (\d+) 拍')
# 2026-10-27 新增：候选供给断流的量化（用户口径的"路径已生成但车不动"）
RE_WAIT = re.compile(r'等候选\(waiting_for_rrt_goal\): \*\*([\d.]+) s\*\*　最长一段 \*\*([\d.]+) s\*\*')


def rounds_for(prefix):
    out = []
    for d in glob.glob(os.path.join(ROOT, 'logs', f'round_*_{prefix}*')):
        if os.path.isdir(d):
            out.append(d)
    return sorted(out, key=lambda p: os.path.getmtime(p))


def one(d):
    r = {'dir': os.path.basename(d), 'tag': os.path.basename(d).split('_')[-1]}
    dp = os.path.join(d, 'digest.md')
    if os.path.exists(dp):
        t = open(dp, encoding='utf-8', errors='replace').read()
        m = RE_VERDICT.search(t)
        if m:
            r['verdict'] = m.group(1)
            r['failed'] = m.group(2)
        m = RE_COLL.search(t)
        if m:
            r['contact'] = int(m.group(1))
            r['reverse_m'] = float(m.group(2))
            r['rev_explore'] = float(m.group(3))
            r['rev_return'] = float(m.group(4))
        m = RE_STUCK.search(t)
        if m:
            r['stuck'] = int(m.group(1))
            r['stuck_cmd'] = int(m.group(2))
            r['stuck_max_s'] = float(m.group(3))
        m = RE_CONT.search(t)
        if m:
            r['cont_ep'] = int(m.group(1))
            r['cont_s'] = float(m.group(2))
        m = RE_STALLSET.search(t)
        if m:
            r['stall_s'] = float(m.group(1))
            r['stall30'] = int(m.group(2))
        m = RE_WAIT.search(t)
        if m:
            r['wait_goal_s'] = float(m.group(1))
            r['wait_goal_max_s'] = float(m.group(2))
        m = RE_DISP.search(t)
        if m:
            r['dispatch'] = m.group(1).strip()
            for k, v in re.findall(r'(\w+)=(\d+)', m.group(1)):
                r['dispatch_' + k] = int(v)
        m = RE_MISC.search(t)
        if m:
            r['escape_goals'] = int(m.group(1))
            r['tl_samples'] = int(m.group(2))
        # S0/S1/S2/S3
        for lvl in ('S0', 'S1', 'S2', 'S3'):
            mm = re.search(r'\| ' + lvl + r' \|[^|]*\|[^|]*\| (✅|❌) \|', t)
            if mm:
                r[lvl] = (mm.group(1) == '✅')
    ep = os.path.join(d, 'evaluate.json')
    if os.path.exists(ep):
        try:
            ev = json.load(open(ep, encoding='utf-8'))
            r['eval_contact'] = ev.get('collision', {}).get('physical_contacts')
            r['eval_tf'] = ev.get('checks', {}).get('tf_healthy')
            r['eval_nav2'] = ev.get('checks', {}).get('nav2_active')
        except Exception:                       # noqa: BLE001
            pass
    # timeline 派生的覆盖度/位移
    tp = os.path.join(d, 'timeline.jsonl')
    if os.path.exists(tp):
        cells, xs, ys, sims = set(), [], [], []
        try:
            for line in open(tp, encoding='utf-8', errors='replace'):
                line = line.strip()
                if not line:
                    continue
                try:
                    row = json.loads(line)
                except Exception:               # noqa: BLE001
                    continue
                x, y = row.get('x'), row.get('y')
                if x is None or y is None:
                    continue
                cells.add((round(x / 0.5), round(y / 0.5)))
                xs.append(x)
                ys.append(y)
                if row.get('sim') is not None:
                    sims.append(row['sim'])
        except Exception:                       # noqa: BLE001
            pass
        r['cells_05m'] = len(cells)
        if xs:
            import math
            r['max_range_m'] = round(max(math.hypot(x, y) for x, y in zip(xs, ys)), 2)
        if sims:
            r['sim_span_s'] = round(max(sims) - min(sims), 1)
    rp = os.path.join(d, 'costmap', 'residue.jsonl')
    if os.path.exists(rp):
        rot_rows = rot_grew = 0
        grow_sum = 0
        occ_series = []
        try:
            for line in open(rp, encoding='utf-8', errors='replace'):
                line = line.strip()
                if not line:
                    continue
                try:
                    row = json.loads(line)
                except Exception:               # noqa: BLE001
                    continue
                o = row.get('raw_occ')
                if isinstance(o, int):
                    occ_series.append(o)
                if row.get('rotating'):
                    rot_rows += 1
                    d_occ = row.get('d_occ')
                    if isinstance(d_occ, int) and d_occ > 0:
                        rot_grew += 1
                        grow_sum += d_occ
        except Exception:                       # noqa: BLE001
            pass
        r['probe_rows'] = len(occ_series)
        r['probe_rot_rows'] = rot_rows
        r['probe_rot_grew'] = rot_grew
        r['probe_rot_grew_cells'] = grow_sum
        if occ_series:
            r['probe_occ_max'] = max(occ_series)
            r['probe_occ_last'] = occ_series[-1]
    sp = os.path.join(d, 'costmap', 'summary.json')
    if not os.path.exists(sp):
        cand = glob.glob(os.path.join(d, 'costmap*', 'summary.json'))
        sp = cand[0] if cand else sp
    if os.path.exists(sp):
        try:
            s = json.load(open(sp, encoding='utf-8'))
            r['res_samples'] = s.get('samples')
            r['res_occ_max'] = s.get('raw_occ_max')
            r['res_rot_events'] = s.get('rot_growth_events')
            r['res_occ_end'] = (s.get('events') or [{}])[-1].get('occ_end') \
                if s.get('events') else None
        except Exception:                       # noqa: BLE001
            pass
    shots = os.path.join(d, 'shots')
    if os.path.isdir(shots):
        r['shots'] = len([f for f in os.listdir(shots) if f.endswith('.png')])
    return r


def agg(rows, key):
    vals = [r[key] for r in rows if isinstance(r.get(key), (int, float))]
    if not vals:
        return '-'
    if all(isinstance(v, bool) for v in vals):
        return f'{sum(1 for v in vals if v)}/{len(vals)}'
    return f'{sum(vals) / len(vals):.2f}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('prefixes', nargs='+')
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    all_rows = {}
    for p in a.prefixes:
        all_rows[p] = [one(d) for d in rounds_for(p)]
    L = ['# 批次对照（%s）' % ' vs '.join(a.prefixes), '']
    for p, rows in all_rows.items():
        L.append(f'## {p}（{len(rows)} 轮）')
        if not rows:
            L.append('- 无数据'); L.append(''); continue
        hdr = ['tag', 'verdict', '失败项', 'S0', 'S1', 'S2', '卡死', '卡住', '最长s',
               '接触', '倒车m', '派发来源', '覆盖格', '最远m', '探针行', '旋转行',
               '旋转增格', 'raw_occ_max', '截图', '等候选s', '等候选最长s']
        L.append('| ' + ' | '.join(hdr) + ' |')
        L.append('| ' + ' | '.join(['---'] * len(hdr)) + ' |')
        for r in rows:
            def g(k):
                v = r.get(k, '-')
                if isinstance(v, bool):
                    return '✅' if v else '❌'
                return str(v)
            L.append('| ' + ' | '.join([
                g('tag'), g('verdict'), g('failed'), g('S0'), g('S1'), g('S2'),
                g('stuck'), g('stuck_cmd'), g('stuck_max_s'), g('contact'),
                g('reverse_m'), g('dispatch'), g('cells_05m'), g('max_range_m'),
                g('probe_rows'), g('probe_rot_rows'),
                g('probe_rot_grew_cells'), g('probe_occ_max'), g('shots'),
                g('wait_goal_s'), g('wait_goal_max_s')]) + ' |')
        L.append('')
        L.append('**均值**：卡死 %s 轮/轮，卡住 %s，接触 %s，覆盖格 %s，倒车 %s m，'
                 '等候选 %s s（最长 %s s）'
                 % (agg(rows, 'stuck'), agg(rows, 'stuck_cmd'), agg(rows, 'contact'),
                    agg(rows, 'cells_05m'), agg(rows, 'reverse_m'),
                    agg(rows, 'wait_goal_s'), agg(rows, 'wait_goal_max_s')))
        npass = sum(1 for r in rows if r.get('verdict') == 'PASS')
        L.append(f'**PASS**：{npass}/{len(rows)}')
        L.append('')
    # ---- 组间对照（第一组当基线）----
    if len(a.prefixes) >= 2:
        base = a.prefixes[0]
        rows_b = all_rows.get(base) or []
        L.append('## 组间对照（基线=%s）' % base)
        L.append('')
        L.append('| 指标 | ' + ' | '.join(a.prefixes) + ' |')
        L.append('| --- ' * (len(a.prefixes) + 1) + '|')
        KEYS = [('卡死/轮', 'stuck'), ('其中卡住(cmd≠0)/轮', 'stuck_cmd'),
                ('等候选 s/轮', 'wait_goal_s'), ('等候选最长 s', 'wait_goal_max_s'),
                ('最长卡死 s', 'stuck_max_s'), ('接触/轮', 'contact'),
                ('倒车 m/轮', 'reverse_m'), ('覆盖格', 'cells_05m'),
                ('最短 m', 'max_range_m'), ('旋转增格/轮', 'probe_rot_grew_cells'),
                ('raw_occ_max', 'probe_occ_max')]
        for label, key in KEYS:
            cells = []
            for pfx in a.prefixes:
                rows = all_rows.get(pfx) or []
                vals = [r[key] for r in rows if isinstance(r.get(key), (int, float))]
                cells.append('%.2f' % (sum(vals) / len(vals)) if vals else '-')
            L.append('| %s | %s |' % (label, ' | '.join(cells)))
        # S0/S1/S2 通过轮数
        for lvl in ('S0', 'S1', 'S2'):
            cells = []
            for pfx in a.prefixes:
                rows = [r for r in (all_rows.get(pfx) or []) if lvl in r]
                cells.append('%d/%d' % (sum(1 for r in rows if r[lvl]), len(rows)))
            L.append('| %s 通过轮数 | %s |' % (lvl, ' | '.join(cells)))
        cells = []
        for pfx in a.prefixes:
            rows = all_rows.get(pfx) or []
            cells.append('%d/%d' % (sum(1 for r in rows if r.get('verdict') == 'PASS'), len(rows)))
        L.append('| 全绿轮数(verdict PASS) | %s |' % ' | '.join(cells))
        L.append('')
    txt = '\n'.join(L) + '\n'
    out = a.out or os.path.join(ROOT, 'logs', 'compare_%s.md' % '_'.join(a.prefixes))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    open(out, 'w', encoding='utf-8').write(txt)
    print(txt)
    print(f'[compare] 写出 {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
