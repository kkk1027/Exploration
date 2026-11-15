#!/usr/bin/env python3
"""对比「房子内部未探区域」的几种定义，找出房型无关的那个。

问题背景
--------
NCR = 可通行已知 / (可通行已知 + 房子内可达未知)

分母里的"房子内未知"必须把**室外**排除掉。当前实现用"与地图四边连通的未知
算作室外"，实测会误吞房子里的未探房间（通过墙上尚未建图的缝隙连到图边），
导致 NCR 虚高（0.9373 vs 凸包下的 0.7771）。

候选定义
--------
A  convex         已知区域凸包。L 形/凹形房型会把凹角外的室外算进来。
B  closing@k      对 known 做闭运算（核 k 格）再 fill_holes：
                   把门洞/墙缝封上，被围住的空腔 = 房子内部。
C  dilate@N       已知区域膨胀 N 格。
D  border         与地图四边连通的未知算室外（当前实现，作为对照）。

为什么优先 B
-----------
闭运算的物理含义是"把小于核宽度的缺口封上"，与房型无关：
无论 L 形、十字形还是多房间，只要墙体缺口小于核直径，房子轮廓就能闭合，
室外未知不会被围住。A 依赖整体凸性，D 依赖"外面一定连到图边"。

用法
----
  python3 harness/compare_room_definitions.py            # 全部方法
  python3 harness/compare_room_definitions.py --dirs ... # 指定轮次
"""
import argparse
import glob
import os
import statistics as st
import sys
import time

import numpy as np
from scipy import ndimage
from scipy.spatial import ConvexHull, Delaunay

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'harness'))
from probe_ncr import load_pgm, load_map_yaml          # noqa: E402

UNKNOWN = 205


def load_masks(d):
    pix, w, h = load_pgm(os.path.join(d, 'map.pgm'))
    if pix is None:
        return None
    meta = load_map_yaml(os.path.join(d, 'map.yaml'))
    res = meta.get('res', 0.05)
    free = (pix != UNKNOWN) & (pix > 0)
    occ = (pix == 0)
    unk = (pix == UNKNOWN)
    return free, occ, unk, res


def navigable_known(free, res, inscribed=0.235):
    dist = ndimage.distance_transform_edt(free)
    return free & (dist * res >= inscribed)


def inside_convex(known):
    ys, xs = np.nonzero(known)
    if len(xs) < 4:
        return np.zeros_like(known)
    pts = np.column_stack([xs, ys])
    try:
        hull = ConvexHull(pts)
    except Exception:                                  # noqa: BLE001
        return np.zeros_like(known)
    delau = Delaunay(pts[hull.vertices])
    h, w = known.shape
    gy, gx = np.mgrid[0:h, 0:w]
    flat = np.column_stack([gx.ravel(), gy.ravel()])
    return (delau.find_simplex(flat) >= 0).reshape(h, w)


def inside_closing(known, k):
    """闭运算封缺口 + 填孔洞，得到房子轮廓。"""
    if k <= 1:
        return known.copy()
    st_el = np.ones((k, k), dtype=bool)
    closed = ndimage.binary_closing(known, structure=st_el)
    return ndimage.binary_fill_holes(closed)


def inside_dilate(known, n):
    if n <= 0:
        return known.copy()
    st_el = np.ones((3, 3), dtype=bool)
    return ndimage.binary_dilation(known, structure=st_el, iterations=n)


def inside_border(unk):
    """与图边连通的未知算室外 ⇒ 返回 in-border-exterior 的补集。"""
    lab, _n = ndimage.label(unk)
    edge = set()
    for e in (lab[0, :], lab[-1, :], lab[:, 0], lab[:, -1]):
        edge.update(int(v) for v in e if v)
    ext = np.isin(lab, list(edge)) if edge else np.zeros_like(unk)
    return ext


def evaluate(d):
    masks = load_masks(d)
    if masks is None:
        return None
    free, occ, unk, res = masks
    known = free | occ
    nav = navigable_known(free, res)
    nav_n = int(nav.sum())
    out = {'dir': os.path.basename(d)}

    def ncr_of(inside_unknown):
        denom = nav_n + inside_unknown
        return (nav_n / denom) if denom > 0 else None

    t0 = time.time()
    inside = inside_convex(known)
    out['convex'] = (ncr_of(int((unk & inside).sum())),
                     int((unk & inside).sum()),
                     (time.time() - t0) * 1000)

    for k in (3, 5, 7, 9, 11, 15, 21):
        t0 = time.time()
        ins = inside_closing(known, k)
        out['closing%d' % k] = (ncr_of(int((unk & ins).sum())),
                                int((unk & ins).sum()),
                                (time.time() - t0) * 1000)

    for n in (10, 20, 30, 40):
        t0 = time.time()
        ins = inside_dilate(known, n)
        out['dilate%d' % n] = (ncr_of(int((unk & ins).sum())),
                               int((unk & ins).sum()),
                               (time.time() - t0) * 1000)

    t0 = time.time()
    ext = inside_border(unk)
    out['border'] = (ncr_of(int((unk & ~ext).sum())),
                     int((unk & ~ext).sum()),
                     (time.time() - t0) * 1000)

    out['_nav'] = nav_n
    out['_unk'] = int(unk.sum())
    return out


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument('--dirs', nargs='*')
    ap.add_argument('--pattern', default='p14vU')
    ap.add_argument('--n', type=int, default=10, help='取样轮数')
    args = ap.parse_args(argv[1:])

    dirs = args.dirs
    if not dirs:
        dirs = sorted(glob.glob(os.path.join(
            ROOT, 'logs', 'round_*_%s*' % args.pattern)))[:args.n]
    if not dirs:
        print('没有轮次')
        return 2

    keys = (['convex'] + ['closing%d' % k for k in (3, 5, 7, 9, 11, 15, 21)]
            + ['dilate%d' % n for n in (10, 20, 30, 40)] + ['border'])
    rows = []
    for d in dirs:
        r = evaluate(d)
        if r:
            rows.append(r)
    if not rows:
        return 1

    print('## 各定义下的 NCR（%d 轮最终地图）\n' % len(rows))
    hdr = '%-12s' % '轮次' + ''.join('%-11s' % k[:10] for k in keys)
    print(hdr)
    print('-' * len(hdr))
    for r in rows:
        line = '%-12s' % r['dir'].split('_', 2)[-1][:11]
        for k in keys:
            v = r[k][0]
            line += '%-11s' % (('%.4f' % v) if v is not None else '-')
        print(line)

    print('\n## 汇总\n')
    print('%-12s %-9s %-9s %-9s %-10s %-9s' % (
        '定义', '中位', '最小', '最大', '极差', '耗时ms'))
    print('-' * 62)
    for k in keys:
        vs = [r[k][0] for r in rows if r[k][0] is not None]
        ms = [r[k][2] for r in rows]
        if not vs:
            continue
        print('%-12s %-9.4f %-9.4f %-9.4f %-10.4f %-9.1f' % (
            k, st.median(vs), min(vs), max(vs),
            max(vs) - min(vs), st.median(ms)))

    print('\n## 判读要点\n')
    print('- **极差小** = 指标稳定，不同轮次之间可比。')
    print('- **中位偏低** = 分母包含了更多"房子里没探的地方"（更严格）。')
    print('- **border** 是当前实现，作为对照；它给出的值最高说明它漏算了。')
    print('- **convex** 在 L 形房型上会把凹角外算进来（分母偏大 ⇒ NCR 偏低）。')
    print('- **closing@k** 的 k 需小于门宽（否则把门也封上，房子里未探的房间')
    print('  变成不可达 ⇒ 分母偏小 ⇒ NCR 虚高）；需大于墙缝（否则漏出去）。')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
