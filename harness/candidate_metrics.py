#!/usr/bin/env python3
"""candidate_metrics.py —— 候选打分专用的独立验收指标（用户 2026-10-28 要求）。

为什么单独一套
  现有的 evaluate.py 判"这一轮跑得合不合格"（PASS/接触/倒车…），它看不见
  "候选选得好不好"。候选打分改造（未探索区域优先）要能自己验证，所以这里给
  一组**只看候选与探索收获**的指标，互不替代、可逐轮对比。

指标分四组
  S 供给   ：候选从哪来、来多少、被丢多少
       S1 cands_seen        仲裁器看到的候选总数
       S2 cands_unique      不同坐标数（0.25 m 网格去重）
       S3 cands_published   实际派发数
       S4 reject_ratio      被拒比例（内容见 reject_detail）
       S5 repair_ratio      后验"修复改投"占比
       S6 cands_per_dispatch 供给冗余度 = seen / published
  G 收获   ：这一轮到底探出多少地方（用户提的"落盘区域"）
       G1 known_m2          最终地图已知面积（空闲+占据）
       G2 free_m2 / occ_m2
       G3 traverse_m        行驶里程
       G4 gain_per_m        已知面积 / 里程（m²/m，**核心效率指标**）
       G5 gain_per_s        已知面积 / 探索时长
       G6 grids_1m          被探索到的 1 m 网格数（分散度，避免只看总量）
  C 打分质量：候选打分是否真的指向"没探过的地方"（依赖 candidate_probe.jsonl）
       C1 novelty_2m        派发时目标 2 m 内未知格数（中位）
       C2 free_frac_2m      派发时目标 2 m 内已知空闲占比（中位；越接近 1 = 越熟悉）
       C3 repeat_ratio      派发时 2 m 内已知空闲占比 >=0.90 的比例（"往老地方跑"）
       C4 dispatch_dist_m   派发距离（中位/max）
       C5 turn_deg          派发时车头到目标的转角（中位）
  A 守门   ：不许为了覆盖率把车搞坏（沿用现有口径）
       A1 PASS / 接触段 / 卡死 / 卡住 / 停滞秒 / 等候选秒(最长)
       A2 plan_fail: iters / novalid / lethal_start
       A3 reverse_m（按新包络）

用法
  python3 harness/candidate_metrics.py <round_dir> [--json out.json]
输出：stdout 打印 markdown 表 + 一行 JSON 摘要（供脚本聚合）
"""
import argparse
import json
import math
import os
import re
import statistics as st
import sys

KNOWN, OCC, UNK = 'known', 'occ', 'unk'


def _load_pgm(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        return w, h, f.read()


def _jl(path):
    out = []
    if not os.path.exists(path):
        return out
    for line in open(path, encoding='utf-8', errors='replace'):
        line = line.strip()
        if line.startswith('{'):
            try:
                out.append(json.loads(line))
            except Exception:                      # noqa: BLE001
                pass
    return out


def _digest_metrics(d):
    """从 digest.md 取守门指标（digest 由 round_digest.py 生成）。"""
    t = ''
    p = os.path.join(d, 'digest.md')
    if os.path.exists(p):
        t = open(p, encoding='utf-8', errors='replace').read()
    m = {}

    def grab(key, pat, cast=float):
        mm = re.search(pat, t)
        m[key] = cast(mm.group(1)) if mm else None
    grab('contact_ep', r'接触段/时长: (\d+) 段', int)
    grab('contact_s', r'接触段/时长: \d+ 段 / ([\d.]+) s')
    grab('stuck', r'卡死事件（[^）]*）: \*\*(\d+)\*\*', int)
    grab('stuck_cmd', r'其中"想动却动不了\(cmd非0\)": \*\*(\d+)\*\*', int)
    grab('stuck_max_s', r'最长 ([\d.]+) s')
    grab('stall_s', r'停滞总秒数: \*\*([\d.]+) s\*\*')
    grab('wait_s', r'等候选\(waiting_for_rrt_goal\): \*\*([\d.]+) s\*\*')
    grab('wait_max_s', r'等候选\(waiting_for_rrt_goal\): \*\*[\d.]+ s\*\*　最长一段 \*\*([\d.]+) s\*\*')
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('round_dir')
    ap.add_argument('--json', default='')
    a = ap.parse_args()
    d = a.round_dir
    out = {'round': os.path.basename(d)}

    # ---------- S：供给（仲裁器统计行） ----------
    stats = {}
    lp = os.path.join(d, 'launch.log')
    if os.path.exists(lp):
        txt = open(lp, errors='replace').read()
        ms = re.findall(r"统计 \{([^}]*)\}", txt)
        if ms:
            try:
                stats = eval('{' + ms[-1] + '}', {'__builtins__': {}}, {})   # noqa: S307
            except Exception:                       # noqa: BLE001
                stats = {}
        out['plan_fail_iters'] = txt.count('exceeded maximum iterations')
        out['plan_fail_novalid'] = txt.count('no valid path found')
        out['plan_fail_lethal_start'] = txt.count('Starting point in lethal space')
    seen = int(stats.get('seen') or 0)
    published = int(stats.get('published') or 0)
    rejected = sum(int(v) for k, v in stats.items()
                   if k.startswith('rejected_') and isinstance(v, (int, float)))
    repair = int(stats.get('repair_cost') or 0)
    out['S1_cands_seen'] = seen
    out['S3_cands_published'] = published
    # 注意：rejected_* 是"按拍累计"的计数（同一候选会在多拍里被重复计入），
    # 所以它/候选数 会 >1，不是比例，而是"每个候选平均被拒几拍"，用来横向比较松紧。
    out['S4_reject_ticks_per_cand'] = round(rejected / float(seen), 2) if seen else None
    out['S5_repair_ratio'] = round(repair / float(seen), 3) if seen else None
    out['S6_cands_per_dispatch'] = round(seen / float(published), 2) if published else None
    out['reject_detail'] = {k: v for k, v in stats.items() if k.startswith('rejected_')}

    # ---------- C：打分质量（候选探针） ----------
    probes = _jl(os.path.join(d, 'candidate_probe.jsonl'))
    out['C0_probe_rows'] = len(probes)
    if probes:
        uniq = {(round(p['goal'][0] / 0.25), round(p['goal'][1] / 0.25)) for p in probes
                if p.get('goal')}
        out['S2_cands_unique'] = len(uniq)

        def med(key):
            v = [p.get(key) for p in probes if isinstance(p.get(key), (int, float))]
            return st.median(v) if v else None
        out['C1_novelty_2m'] = med('unk_2m')
        out['C2_free_frac_2m'] = med('free_frac_2m')
        ff = [p['free_frac_2m'] for p in probes
              if isinstance(p.get('free_frac_2m'), (int, float))]
        out['C3_repeat_ratio'] = (round(sum(1 for x in ff if x >= 0.90) / float(len(ff)), 3)
                                  if ff else None)
        ds = [p['dist_to_robot'] for p in probes
              if isinstance(p.get('dist_to_robot'), (int, float))]
        out['C4_dispatch_dist_med'] = st.median(ds) if ds else None
        out['C4_dispatch_dist_max'] = max(ds) if ds else None
        out['C5_turn_deg_med'] = med('turn_deg')

    # ---------- G：收获（最终地图 + 里程） ----------
    mp = os.path.join(d, 'map.pgm')
    if os.path.exists(mp):
        w, h, data = _load_pgm(mp)
        yml = {}
        yp = os.path.join(d, 'map.yaml')
        if os.path.exists(yp):
            try:
                import yaml
                yml = yaml.safe_load(open(yp, encoding='utf-8')) or {}
            except Exception:                   # noqa: BLE001
                yml = {}
        res = float(yml.get('resolution') or 0.05)
        ox = float((yml.get('origin') or [0, 0, 0])[0])
        oy = float((yml.get('origin') or [0, 0, 0])[1])
        free = occ = 0
        grids = set()
        for j in range(h):
            base = j * w
            for i in range(w):
                v = data[base + i]
                if v == 205 or v < 0:
                    continue
                if v == 0:
                    occ += 1
                else:
                    free += 1
                if res > 0:
                    gx = int((ox + i * res) / 1.0)
                    gy = int((oy + j * res) / 1.0)
                    grids.add((gx, gy))
        cell = res * res
        out['G1_known_m2'] = round((free + occ) * cell, 1)
        out['G2_free_m2'] = round(free * cell, 1)
        out['G2_occ_m2'] = round(occ * cell, 1)
        out['G6_grids_1m'] = len(grids)

    tl = _jl(os.path.join(d, 'timeline.jsonl'))
    pts = [(r['mx'], r['my']) for r in tl
           if isinstance(r.get('mx'), (int, float)) and isinstance(r.get('my'), (int, float))]
    trav = 0.0
    for i in range(1, len(pts)):
        trav += math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
    out['G3_traverse_m'] = round(trav, 1)
    if tl:
        s0 = tl[0].get('sim') or 0
        s1 = tl[-1].get('sim') or 0
        dur = max(1.0, s1 - s0)
        out['G5_dur_s'] = round(dur, 0)
        if out.get('G1_known_m2'):
            out['G4_gain_per_m'] = round(out['G1_known_m2'] / max(0.1, trav), 3)
            out['G5_gain_per_s'] = round(out['G1_known_m2'] / dur, 4)

    # ---------- A：守门 ----------
    out.update(_digest_metrics(d))
    ev = os.path.join(d, 'evaluate.json')
    if os.path.exists(ev):
        try:
            j = json.load(open(ev, encoding='utf-8'))
            out['A1_verdict'] = j.get('verdict')
            c = j.get('collision') or {}
            out['A3_reverse_m'] = c.get('reverse_m')
        except Exception:                       # noqa: BLE001
            pass

    # ---------- 输出 ----------
    rows = [
        ('S1 候选总数', out.get('S1_cands_seen')),
        ('S2 候选去重数', out.get('S2_cands_unique')),
        ('S3 派发数', out.get('S3_cands_published')),
        ('S4 每候选被拒拍数', out.get('S4_reject_ticks_per_cand')),
        ('S5 修复比例', out.get('S5_repair_ratio')),
        ('S6 供给冗余 seen/pub', out.get('S6_cands_per_dispatch')),
        ('C1 派发时 2m 内未知格(中位)', out.get('C1_novelty_2m')),
        ('C2 派发时 2m 内已知空闲占比(中位)', out.get('C2_free_frac_2m')),
        ('C3 往老地方跑的比例(>=0.90)', out.get('C3_repeat_ratio')),
        ('C4 派发距离 中位/最大 m', '%s / %s' % (
            round(out['C4_dispatch_dist_med'], 2) if out.get('C4_dispatch_dist_med') else '-',
            round(out['C4_dispatch_dist_max'], 2) if out.get('C4_dispatch_dist_max') else '-')),
        ('C5 派发转角 中位 deg', out.get('C5_turn_deg_med')),
        ('G1 已知面积 m²', out.get('G1_known_m2')),
        ('G2 空闲/占据 m²', '%s / %s' % (out.get('G2_free_m2'), out.get('G2_occ_m2'))),
        ('G3 行驶里程 m', out.get('G3_traverse_m')),
        ('G4 每米新增面积 m²/m', out.get('G4_gain_per_m')),
        ('G5 每秒新增面积 m²/s', out.get('G5_gain_per_s')),
        ('G6 探索到的 1m 网格数', out.get('G6_grids_1m')),
        ('A1 验收', out.get('A1_verdict')),
        ('A1 接触段/秒', '%s / %s' % (out.get('contact_ep'), out.get('contact_s'))),
        ('A1 卡死/卡住/最长', '%s / %s / %s' % (out.get('stuck'), out.get('stuck_cmd'),
                                              out.get('stuck_max_s'))),
        ('A1 停滞秒', out.get('stall_s')),
        ('A1 等候选秒(最长)', '%s (%s)' % (out.get('wait_s'), out.get('wait_max_s'))),
        ('A2 规划失败 超迭代/无解/起点致命', '%s / %s / %s' % (
            out.get('plan_fail_iters'), out.get('plan_fail_novalid'),
            out.get('plan_fail_lethal_start'))),
        ('A3 倒车 m', out.get('A3_reverse_m')),
    ]
    print('## %s 候选打分指标' % out['round'])
    print('')
    print('| 指标 | 值 |')
    print('| --- | --- |')
    for k, v in rows:
        print('| %s | %s |' % (k, '-' if v is None else v))
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump(out, f, ensure_ascii=False, indent=1)
    print('')
    print('JSON ' + json.dumps({k: v for k, v in out.items() if not k.startswith('reject_detail')},
                               ensure_ascii=False))


if __name__ == '__main__':
    main()
