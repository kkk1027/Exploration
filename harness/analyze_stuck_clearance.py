#!/usr/bin/env python3
"""analyze_stuck_clearance.py —— 核对 P0-B："车宽仍小于最窄处宽度，却卡死"。

用法: python3 harness/analyze_stuck_clearance.py [--glob 'logs/round_*']

做法：对每一轮，从 digest.md 里取"卡住(cmd非0)"事件的位置，在**该轮存档地图**上
算"到最近占据格的距离"（纯 BFS，不依赖 scipy —— 本机 scipy 与 numpy 2.x 不兼容），
自由宽度 = 2×净空，然后分三档判读：
  ★比车宽还窄(0.47 m) / 窄于通行判据(0.90 m) / ≥通行判据（能过却卡住）
局限性：存档地图是**轮末**的，事件发生时刻该区域可能是未知（地图上表现为障碍）。
"""
import argparse
import glob
import json
import os
import re
from collections import deque

import numpy as np
import yaml
from PIL import Image

CAR_W = 0.47
NEED = 0.90


def bfs_clearance(occ, res, cap=2.0):
    H, W = occ.shape
    INF = 1e9
    d = np.full((H, W), INF, dtype=np.float32)
    q = deque()
    for j in range(H):
        for i in np.nonzero(occ[j])[0]:
            d[j, i] = 0.0
            q.append((j, i))
    while q:
        j, i = q.popleft()
        base = d[j, i] + res
        if base > cap:
            continue
        for dj, di in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nj, ni = j + dj, i + di
            if 0 <= nj < H and 0 <= ni < W and d[nj, ni] > base:
                d[nj, ni] = base
                q.append((nj, ni))
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--glob', default='logs/round_*')
    a = ap.parse_args()
    rows = []
    for d in sorted(glob.glob(a.glob)):
        dg, mp, yp = (os.path.join(d, 'digest.md'), os.path.join(d, 'map.pgm'),
                      os.path.join(d, 'map.yaml'))
        if not (os.path.exists(dg) and os.path.exists(mp) and os.path.exists(yp)):
            continue
        txt = open(dg, errors='ignore').read()
        evs = re.findall(r'\|\s*\d+\s*\|[^|]+\|[^|]+\|\s*\d+\s*\|\s*\(([-\d.]+),([-\d.]+)\)'
                         r'\s*\|\s*([^|]+)\|', txt)
        if not evs:
            continue
        y = yaml.safe_load(open(yp))
        res = y['resolution']
        ox, oy, _ = y['origin']
        img = np.array(Image.open(mp))
        H, W = img.shape
        field = bfs_clearance(img < 100, res)
        for x, yy, typ in evs:
            if '卡住' not in typ:
                continue
            x, yy = float(x), float(yy)
            i, j = int((x - ox) / res), H - 1 - int((yy - oy) / res)
            if not (0 <= i < W and 0 <= j < H):
                continue
            rows.append((os.path.basename(d).split('_', 2)[-1], x, yy, 2 * float(field[j, i])))
    print(f'{"轮次":22s} {"卡住位置":20s} {"自由宽度":>9s}  判读')
    n_narrow = n_ok = n_mid = 0
    for tag, x, yy, w in rows:
        if w < CAR_W:
            v = '★比车宽还窄'; n_narrow += 1
        elif w < NEED:
            v = '窄于通行判据'; n_mid += 1
        else:
            v = '≥通行判据（能过却卡住）'; n_ok += 1
        print(f'{tag:22s} ({x:6.2f},{yy:6.2f})  {w:7.2f} m  {v}')
    print(f'\n共 {len(rows)} 个卡住点：<车宽 {n_narrow}；<通行判据 {n_mid}；'
          f'≥通行判据 {n_ok}')
    print('局限：地图是轮末存档，事件时刻未探明区域会表现为障碍。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
