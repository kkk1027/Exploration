#!/usr/bin/env python3
"""analyze_candidates.py —— 候选质量与"卡死"关联的离线统计（探针数据用）。

输入：一批 logs/round_*/（需要 launch.log + supervisor.jsonl + watchdog.jsonl + map.pgm/vision_grid.pgm）
做法：
  1) 从 launch.log 抽**所有**派发目标（含 source=exploration_manager/rrt/home）
  2) 从 map.pgm ∪ vision_grid.pgm 建统一障碍掩码 → 距离场 → 在目标点算复杂度特征：
       vis_occ_r2 / narrow_r2 / free_r2 / comp_r2 / esc_dirs
  3) 从 supervisor.jsonl（giveup_40s、unit_start）与 watchdog.jsonl（stall）抽"卡死事件"
  4) 把每个事件按"事件时机器人位置 ↔ 目标点"最近邻归因（<=4 m），统计各特征的卡死率
输出：终端表格 + 建议阈值。
"""
import argparse
import glob
import json
import math
import os
import re
import sys
from collections import defaultdict

import numpy as np

W = '/home/kbf/stemm_exploration_sim_ws'


def load_pgm(p):
    d = open(p, 'rb').read(); i = 0; toks = []
    while len(toks) < 4:
        while d[i:i + 1].isspace():
            i += 1
        j = i
        while not d[j:j + 1].isspace():
            j += 1
        toks.append(d[i:j]); i = j
    i += 1
    w, h = int(toks[1]), int(toks[2])
    return np.frombuffer(d[i:i + w * h], dtype=np.uint8).reshape(h, w), w, h


def load_meta(p):
    t = open(p).read()
    ox, oy = map(float, re.search(r'origin: \[([-\d.]+), ([-\d.]+)', t).groups())
    res = float(re.search(r'resolution: ([\d.]+)', t).group(1))
    return ox, oy, res


def build_field(round_dir):
    mp = os.path.join(round_dir, 'map.pgm'); vp = os.path.join(round_dir, 'vision_grid.pgm')
    if not os.path.exists(mp):
        return None
    img, w, h = load_pgm(mp); ox, oy, res = load_meta(os.path.join(round_dir, 'map.yaml'))
    blocked = img == 0
    if os.path.exists(vp):
        try:
            vimg, vw, vh = load_pgm(vp); vox, voy, vres = load_meta(os.path.join(round_dir, 'vision_grid.yaml'))
            occ = np.argwhere(vimg == 0)
            if occ.size:
                wx = vox + (occ[:, 1] + 0.5) * vres
                wy = voy + (occ[:, 0] + 0.5) * vres
                ci = ((wx - ox) / res).astype(int); cj = ((wy - oy) / res).astype(int)
                m = (ci >= 0) & (ci < w) & (cj >= 0) & (cj < h)
                blocked[cj[m], ci[m]] = True
        except Exception:
            pass
    from scipy import ndimage
    dist = ndimage.distance_transform_edt(~blocked) * res
    return dist, ox, oy, res, w, h


def features(field, gx, gy, radius=2.0):
    dist, ox, oy, res, w, h = field
    ci, cj = int((gx - ox) / res), int((gy - oy) / res)
    if not (0 <= ci < w and 0 <= cj < h):
        return None
    k = max(1, int(radius / res))
    i0, i1 = max(0, ci - k), min(w, ci + k + 1); j0, j1 = max(0, cj - k), min(h, cj + k + 1)
    win = dist[j0:j1, i0:i1]
    narrow = int((win < 0.35).sum()); free = int((win >= 0.30).sum())
    # 连通域
    comp = 0
    free_mask = win >= 0.30
    si, sj = ci - i0, cj - j0
    if 0 <= si < free_mask.shape[1] and 0 <= sj < free_mask.shape[0] and free_mask[sj, si]:
        seen = np.zeros_like(free_mask, dtype=bool); seen[sj, si] = True
        q = [(sj, si)]; comp = 1
        while q and comp < 4000:
            y, x = q.pop()
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < free_mask.shape[0] and 0 <= nx < free_mask.shape[1] \
                            and not seen[ny, nx] and free_mask[ny, nx]:
                        seen[ny, nx] = True; q.append((ny, nx)); comp += 1
    esc = 0
    step = max(1, int(0.10 / res))
    for ang in [a * math.pi / 4.0 for a in range(8)]:
        ok = True
        for d in range(step, int(0.60 / res) + 1, step):
            x = ci + int(round(math.cos(ang) * d)); y = cj + int(round(math.sin(ang) * d))
            if not (0 <= x < w and 0 <= y < h) or dist[y, x] < 0.35:
                ok = False; break
        esc += 1 if ok else 0
    return {'narrow_r2': narrow, 'free_r2': free, 'comp_r2': comp, 'esc_dirs': esc,
            'occ_frac_r2': 1.0 - free / float(win.size)}


def parse_dispatches(log):
    out = []
    for line in open(log, encoding='utf-8', errors='ignore'):
        m = re.search(r'navigation goal dispatched: source=(\w+), target=\(([-\d.]+), ([-\d.]+)\), '
                      r'robot=\(([-\d.]+), ([-\d.]+), yaw=([-\d.]+)deg\)', line)
        if m:
            out.append({'src': m.group(1), 'gx': float(m.group(2)), 'gy': float(m.group(3)),
                        'rx': float(m.group(4)), 'ry': float(m.group(5))})
    return out


def parse_events(round_dir):
    ev = []
    p = os.path.join(round_dir, 'supervisor.jsonl')
    if os.path.exists(p):
        for line in open(p, encoding='utf-8', errors='ignore'):
            try:
                r = json.loads(line)
            except Exception:
                continue
            if r.get('type') in ('giveup_40s', 'unit_start'):
                pos = r.get('pos')
                if pos is None:
                    try:
                        nav = json.loads(r.get('nav') or '{}')
                        px = (nav.get('pose') or {}).get('x'); py = (nav.get('pose') or {}).get('y')
                        pos = [px, py]
                    except Exception:
                        pos = None
                if pos and pos[0] is not None:
                    ev.append({'kind': r['type'], 'x': pos[0], 'y': pos[1],
                               'sev': 3 if r['type'] == 'giveup_40s' else 1})
    p = os.path.join(round_dir, 'watchdog.jsonl')
    if os.path.exists(p):
        for line in open(p, encoding='utf-8', errors='ignore'):
            try:
                r = json.loads(line)
            except Exception:
                continue
            if r.get('type') == 'stall' and r.get('pos'):
                ev.append({'kind': 'stall', 'x': r['pos'][0], 'y': r['pos'][1], 'sev': 2})
    return ev


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--glob', default='logs/round_*')
    ap.add_argument('--include', default='')
    a = ap.parse_args()
    dirs = [d for d in sorted(glob.glob(os.path.join(W, a.glob)), key=os.path.getmtime)
            if a.include in os.path.basename(d)]
    rows = []
    for d in dirs:
        field = build_field(d)
        if field is None:
            continue
        disp = parse_dispatches(os.path.join(d, 'launch.log'))
        evs = parse_events(d)
        for e in evs:
            best, bd = None, 1e9
            for i, g in enumerate(disp):
                dist = math.hypot(g['gx'] - e['x'], g['gy'] - e['y'])
                if dist < bd:
                    best, bd = i, dist
            if best is not None and bd <= 4.0:
                disp[best].setdefault('events', []).append(e['sev'])
                e['attributed'] = True
        for g in disp:
            f = features(field, g['gx'], g['gy'])
            if not f:
                continue
            f.update({'round': os.path.basename(d).split('_')[-1], 'src': g['src'],
                      'goal': (round(g['gx'], 2), round(g['gy'], 2)),
                      'sev': sum(g.get('events', []))})
            rows.append(f)
    if not rows:
        print('没有可用数据'); return 1
    print('样本（派发目标数）:', len(rows), ' 来自', len(dirs), '轮')
    bad = [r for r in rows if r['sev'] > 0]
    print('其中被归因到卡死/放弃/静止事件的目标:', len(bad),
          f'({100*len(bad)/len(rows):.0f}%)')
    print()
    feats = ['esc_dirs', 'comp_r2', 'narrow_r2', 'occ_frac_r2', 'free_r2']
    print('%-13s %18s %18s' % ('特征', '安全目标中位数', '卡死目标中位数'))
    for f in feats:
        a1 = sorted(r[f] for r in rows if r['sev'] == 0)
        a2 = sorted(r[f] for r in bad)
        med1 = a1[len(a1)//2] if a1 else float('nan')
        med2 = a2[len(a2)//2] if a2 else float('nan')
        print('%-13s %18.2f %18.2f' % (f, med1, med2))
    print()
    print('按 esc_dirs（可逃逸方向数）分桶的卡死率：')
    for lo, hi in ((0, 0), (1, 2), (3, 4), (5, 8)):
        sub = [r for r in rows if lo <= r['esc_dirs'] <= hi]
        if not sub:
            continue
        b = sum(1 for r in sub if r['sev'] > 0)
        print('  esc_dirs=%d..%d  样本 %3d  卡死 %2d (%3.0f%%)' % (lo, hi, len(sub), b, 100*b/len(sub)))
    print()
    print('按 comp_r2（连通自由格数）分桶：')
    for lo, hi in ((0, 100), (101, 400), (401, 1200), (1201, 99999)):
        sub = [r for r in rows if lo <= r['comp_r2'] <= hi]
        if not sub:
            continue
        b = sum(1 for r in sub if r['sev'] > 0)
        print('  comp_r2=%5d..%-5d 样本 %3d  卡死 %2d (%3.0f%%)' % (lo, hi, len(sub), b, 100*b/len(sub)))
    print()
    print('按 occ_frac_r2（2m 内障碍占比）分桶：')
    for lo, hi in ((0, .05), (.05, .10), (.10, .20), (.20, 1.0)):
        sub = [r for r in rows if lo <= r['occ_frac_r2'] < hi]
        if not sub:
            continue
        b = sum(1 for r in sub if r['sev'] > 0)
        print('  occ=%.2f..%.2f 样本 %3d  卡死 %2d (%3.0f%%)' % (lo, hi, len(sub), b, 100*b/len(sub)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
