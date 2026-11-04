#!/usr/bin/env python3
"""score_round_map.py —— 用世界真值给**每轮存出的地图**打分（建图记忆性/准确性）。

背景：`session_driver` 每轮结束会存 `map.pgm/.yaml`。既有 digest 只看"覆盖格数"，
无法回答"图里记的东西对不对、漏了什么"。这里用 Gazebo 世界的真值 2D 足迹
（`harness/gt_footprints.py` 产物）逐格比对，给出四个可解释的指标：

  precision  地图占据格中，真值上确有其物（≤tol）的比例   ← 1−幽灵率，衡量"记忆是否记错"
  recall     真值实体格（在已探索范围内）被地图标为占据的比例 ← 衡量"记忆是否漏"
  ghost      地图占据格但真值上没有的比例 = 1 − precision
  free_area  地图里"已知空闲"的格数（0.05 m 格）×0.0025 = m²   ← 探索广度
  known_ratio 已知格 / 全部格（含未知）

行序约定（本仓库踩过多次坑，写死在这里）：
  * 地图 PGM（nav2/map_saver 约定）：**行 0 = 图上方 = 最大 y** ⇒ y = oy + (h-1-row+0.5)*res
    （2026-11-04 修好 session_driver 后存出的图就是这一种）
  * 真值位图 gt_grid.npy：**行 0 = 最小 y**（已用静态地图校验到 100% 重合）

用法：
  python3 harness/score_round_map.py --glob 'logs/round_*_m5r*' [--tol 0.20] [--out logs/map_scores.json]
"""
import argparse
import glob
import json
import os
import sys

import numpy as np
from PIL import Image

try:
    import yaml
except Exception:                                            # noqa: BLE001
    yaml = None


def read_pgm(path):
    with open(path, 'rb') as f:
        toks = []
        while len(toks) < 4:
            line = f.readline()
            if not line:
                break
            if line.startswith(b'#'):
                continue
            toks += line.split()
        w, h, mx = int(toks[1]), int(toks[2]), int(toks[3])
        if mx > 255:
            raise SystemExit(f'{path}: 只支持 8bit PGM')
        img = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    return img


def read_yaml(path):
    with open(path, encoding='utf-8') as f:
        txt = f.read()
    if yaml is not None:
        d = yaml.safe_load(txt)
        ox, oy, _ = d['origin']
        return dict(image=os.path.join(os.path.dirname(path), d['image']),
                    res=float(d['resolution']), ox=float(ox), oy=float(oy))
    import re
    m = dict(re.findall(r'^\s*(\w+):\s*(.+)$', txt, re.M))
    ox, oy, _ = [float(x) for x in m['origin'].strip('[]').split(',')]
    return dict(image=os.path.join(os.path.dirname(path), m['image'].strip()),
                res=float(m['resolution']), ox=ox, oy=oy)


def occ_pts(pgm, meta, mask, flip_row):
    h = pgm.shape[0]
    r, c = np.nonzero(mask)
    x = meta['ox'] + (c + 0.5) * meta['res']
    y = meta['oy'] + (((h - 1 - r) if flip_row else r) + 0.5) * meta['res']
    return np.stack([x, y], 1)


def nn_frac(P, Q, tol, chunk=4096):
    """P 中每点到 Q 最近距离 ≤ tol 的比例（Q 为空则全 0）。"""
    if len(P) == 0:
        return 0.0
    if len(Q) == 0:
        return 0.0
    best = np.full(len(P), 1e9)
    for i in range(0, len(Q), chunk):
        q = Q[i:i + chunk]
        best = np.minimum(best, ((P[:, None, :] - q[None, :, :]) ** 2).sum(-1).min(1))
    return float((np.sqrt(best) <= tol).mean())


def load_gt(gt_dir):
    meta = json.load(open(os.path.join(gt_dir, 'gt_grid.json'), encoding='utf-8'))
    arr = np.load(os.path.join(gt_dir, 'gt_grid.npy'))
    res, (ox, oy) = meta['res'], meta['origin'][:2]
    h = arr.shape[0]

    def pts(bit):
        r, c = np.nonzero(arr & bit)
        return np.stack([ox + (c + 0.5) * res, oy + (r + 0.5) * res], 1)

    return dict(solid=pts(8), low=pts(2), lowmask=(arr & 2) > 0,
                res=res, ox=ox, oy=oy, h=h, w=arr.shape[1])


def score_round(d, gt, tol):
    mp = os.path.join(d, 'map.pgm')
    my = os.path.join(d, 'map.yaml')
    if not (os.path.exists(mp) and os.path.exists(my)):
        return None
    meta = read_yaml(my)
    img = meta['image']
    if not os.path.exists(img):
        # 轮目录里 yaml 的 image 名指向 explore_latest.pgm，但快照被 round.sh 存成 map.pgm
        alt = os.path.join(d, 'map.pgm')
        if os.path.exists(alt):
            img = alt
        else:
            print(f'  ! {os.path.basename(d)}: 找不到地图图件（{meta["image"]}）')
            return None
    pgm = read_pgm(img)
    # ★ 行序自检：老轮次（2026-11-04 修 session_driver 之前）存出的图是 bottom-up，
    #   新轮次是 nav2 约定 top-down。这里两种都算一次，取 precision 高者并记录，
    #   避免把"历史镜像图"误判成"建图精度差"。
    occ_top = occ_pts(pgm, meta, pgm < 50, flip_row=True)
    occ_bot = occ_pts(pgm, meta, pgm < 50, flip_row=False)
    p_top = nn_frac(occ_top, gt['solid'], tol)
    p_bot = nn_frac(occ_bot, gt['solid'], tol)
    row_origin = 'top' if p_top >= p_bot else 'bottom'
    flip_row = row_origin == 'top'
    occ = occ_pts(pgm, meta, pgm < 50, flip_row=flip_row)
    free = occ_pts(pgm, meta, pgm > 250, flip_row=flip_row)   # nav2: 254=自由
    unk = int(((pgm > 150) & (pgm < 250)).sum())           # 205=未知
    n_all = pgm.size
    prec = nn_frac(occ, gt['solid'], tol)
    # recall：只看"已探索范围"内的真值格（用地图的已知区做掩码）
    gx = ((gt['solid'][:, 0] - meta['ox']) / meta['res']).astype(int)
    if flip_row:   # top-down：row = (h-1) - (y-oy)/res
        gy = ((meta['oy'] + pgm.shape[0] * meta['res'] - gt['solid'][:, 1]) / meta['res']).astype(int)
    else:          # bottom-up：row = (y-oy)/res
        gy = ((gt['solid'][:, 1] - meta['oy']) / meta['res']).astype(int)
    inside = (gx >= 0) & (gx < pgm.shape[1]) & (gy >= 0) & (gy < pgm.shape[0])
    known_mask = np.zeros_like(pgm, dtype=bool)
    known_mask[pgm <= 250] = True
    sel = inside.copy()
    sel[inside] = known_mask[gy[inside], gx[inside]]
    rec = nn_frac(gt['solid'][sel], occ, tol) if sel.any() else 0.0   # 方向：真值→地图
    # 真值里"会挡车"的低矮带在已探索区的覆盖率（视觉层最该补的部分）
    lows = gt['low']
    lx = ((lows[:, 0] - meta['ox']) / meta['res']).astype(int)
    if flip_row:
        ly = ((meta['oy'] + pgm.shape[0] * meta['res'] - lows[:, 1]) / meta['res']).astype(int)
    else:
        ly = ((lows[:, 1] - meta['oy']) / meta['res']).astype(int)
    lin = (lx >= 0) & (lx < pgm.shape[1]) & (ly >= 0) & (ly < pgm.shape[0])
    lsel = lin.copy()
    lsel[lin] = known_mask[ly[lin], lx[lin]]
    rec_low = nn_frac(lows[lsel], occ, tol) if lsel.any() else 0.0
    return dict(round=os.path.basename(d), row_origin=row_origin,
                occ_cells=int(len(occ)), free_cells=int(len(free)),
                unknown_cells=unk, known_ratio=round(1 - unk / n_all, 4),
                free_area_m2=round(len(free) * (meta['res'] ** 2), 2),
                precision=round(prec, 4), ghost_rate=round(1 - prec, 4),
                recall=round(rec, 4), recall_low=round(rec_low, 4),
                gt_known_cells=int(sel.sum()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--glob', default='logs/round_*')
    ap.add_argument('--gt', default='logs/gt')
    ap.add_argument('--tol', type=float, default=0.20)
    ap.add_argument('--out', default='logs/map_scores.json')
    a = ap.parse_args()
    gt = load_gt(a.gt)
    dirs = sorted(glob.glob(a.glob))
    rows = []
    for d in dirs:
        r = score_round(d, gt, a.tol)
        if r:
            rows.append(r)
            print(f"{r['round'][-28:]:30s} 占据={r['occ_cells']:5d} "
                  f"自由={r['free_cells']:6d}({r['free_area_m2']:6.1f}m²) "
                  f"精度={r['precision'] * 100:5.1f}% 幽灵率={r['ghost_rate'] * 100:4.1f}% "
                  f"召回={r['recall'] * 100:5.1f}% 低矮带召回={r['recall_low'] * 100:5.1f}% "
                  f"已知比={r['known_ratio'] * 100:5.1f}% 行序={r['row_origin']}", flush=True)
    if rows:
        json.dump(rows, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
        def med(k):
            v = [r[k] for r in rows if r.get(k) is not None]
            return float(np.median(v)) if v else float('nan')
        print('\n===== 中位（%d 轮）=====' % len(rows))
        print(f"  占据格中位      = {med('occ_cells'):.0f}")
        print(f"  自由面积中位    = {med('free_area_m2'):.2f} m²")
        print(f"  精度中位        = {med('precision') * 100:.1f}%   （1−幽灵率）")
        print(f"  幽灵率中位      = {med('ghost_rate') * 100:.1f}%")
        print(f"  召回中位        = {med('recall') * 100:.1f}%")
        print(f"  低矮带召回中位  = {med('recall_low') * 100:.1f}%")
        print(f"  已知比中位      = {med('known_ratio') * 100:.1f}%")
        print(f'  输出 {a.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
