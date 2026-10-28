#!/usr/bin/env python3
"""overlay_vision.py —— 把视觉层累积栅格叠到静态地图上，出一张能直接看的 PNG。

用途（2026-10-28 鬼影取证）：
  数字能说明"有 85% 的视觉格离静态障碍 >0.3 m"，但"到底是鬼影还是地图里没有的真物"
  必须看图。约定与项目既有结论一致：
    * 深灰 = 静态地图占据；浅灰 = 已知空闲；白 = 未知
    * 红  = 只在视觉层标记、且离静态障碍 > iso_m（鬼影候选 / 低矮障碍候选）
    * 绿  = 视觉层标记且贴合静态障碍（一致，可信）
    * 蓝十字 = 机器人位姿（地图原点）
  同时打印：视觉格数、绿/红计数、红格到静态障碍的距离分位数。

用法：
  python3 harness/overlay_vision.py --vision /tmp/navonly_vision.pgm \
      --map /home/kbf/stemm_sim_ws/maps/explore_latest.pgm --out /tmp/ghost_overlay.png
"""
import argparse
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

# PGM 行 0 在图上方、origin 在左下 ⇒ map 帧 y = oy + (h-1-row)*res
# （本项目已踩过的镜像坑，两个栅格都必须按这条换）


def read_pgm(path):
    with open(path, 'rb') as f:
        tok = []
        while len(tok) < 4:
            line = f.readline()
            if line.startswith(b'#'):
                continue
            tok += line.split()
        w, h, maxv = int(tok[1]), int(tok[2]), int(tok[3])
        img = np.frombuffer(f.read(w * h * (2 if maxv > 255 else 1)),
                            dtype=np.uint8 if maxv <= 255 else np.uint16)
    return img.reshape(h, w), maxv


def read_yaml(path):
    d = {}
    with open(path, encoding='utf-8') as f:
        for line in f:
            if ':' in line:
                k, v = line.split(':', 1)
                d[k.strip()] = v.strip()
    ox, oy, _ = [float(x) for x in d['origin'].strip('[]').split(',')]
    return dict(res=float(d['resolution']), ox=ox, oy=oy,
                image=os.path.join(os.path.dirname(path), d['image']))


def to_map_cells(pgm, meta, is_occ, flip_row=True):
    """返回 map 帧坐标 (x,y) 数组。

    flip_row=True ：PGM 行 0 在图上方（nav2/map_saver 约定）
    flip_row=False：PGM 行 0 在 y=miny（vision_obstacle_grid.dump_grid 的约定）
    """
    h, w = pgm.shape
    rows, cols = np.nonzero(is_occ(pgm))
    x = meta['ox'] + (cols + 0.5) * meta['res']
    y = (meta['oy'] + (h - rows - 0.5) * meta['res']) if flip_row \
        else (meta['oy'] + (rows + 0.5) * meta['res'])
    return np.stack([x, y], axis=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--vision', default='/tmp/navonly_vision.pgm')
    ap.add_argument('--vision-yaml', default='/tmp/navonly_vision.yaml')
    ap.add_argument('--map', default='/home/kbf/stemm_sim_ws/maps/explore_latest.pgm')
    ap.add_argument('--map-yaml', default='/home/kbf/stemm_sim_ws/maps/explore_latest.yaml')
    ap.add_argument('--iso-m', type=float, default=0.30)
    ap.add_argument('--out', default='/tmp/ghost_overlay.png')
    ap.add_argument('--scale', type=int, default=8)
    a = ap.parse_args()

    vm, mm = read_yaml(a.vision_yaml), read_yaml(a.map_yaml)
    vpgm, _ = read_pgm(a.vision)
    mpgm, _ = read_pgm(a.map)

    # 静态地图：<50 占据（PGM: 深色=占据）
    spts = to_map_cells(mpgm, mm, lambda p: p < 50, flip_row=True)
    srowscols = np.nonzero(mpgm < 50)

    # 视觉栅格；0 未知/空闲 100/254 占据 → 这里 dump 出来的约定：>0 视为标记
    # 视觉落盘是 trinary：0=占用、205=未知、254=自由 ⇒ 只有 0 是标记
    vpts = to_map_cells(vpgm, vm, lambda p: p == 0, flip_row=False)
    print(f'[overlay] 静态占据格={len(spts)}  视觉标记格={len(vpts)}')

    if len(vpts) == 0:
        print('[overlay] 视觉栅格是空的，退出')
        return 2

    dmin = np.full(len(vpts), 1e9)
    for i in range(0, len(spts), 2048):
        s = spts[i:i + 2048]
        d = np.sqrt(((vpts[:, None, :] - s[None, :, :]) ** 2).sum(-1))
        dmin = np.minimum(dmin, d.min(1))
    near = dmin <= a.iso_m
    print(f'[overlay] 与静态障碍一致(<= {a.iso_m} m) = {int(near.sum())}  '
          f'孤立(> {a.iso_m} m) = {int((~near).sum())} '
          f'({(~near).mean() * 100:.1f}%)')
    print('[overlay] 孤立格距离分位: '
          + '  '.join(f'p{q}={np.percentile(dmin[~near], q):.2f}m'
                      for q in (10, 50, 90)) if (~near).any() else '')

    # 画布：以静态地图的世界范围为底
    mh, mw = mpgm.shape
    W, H = mw, mh
    canvas = np.full((H, W, 3), 255, dtype=np.uint8)
    yy, xx = np.mgrid[0:H, 0:W]
    # 空闲/未知底图
    canvas[mpgm >= 50] = (225, 225, 225)
    canvas[mpgm < 50] = (90, 90, 90)

    def w2px(p):
        col = np.round((p[:, 0] - mm['ox']) / mm['res'] - 0.5).astype(int)
        row = np.round((mh - 1) - ((p[:, 1] - mm['oy']) / mm['res'] - 0.5)).astype(int)
        ok = (col >= 0) & (col < W) & (row >= 0) & (row < H)
        return col[ok], row[ok]

    c, r = w2px(vpts[near])
    canvas[r, c] = (0, 160, 0)
    c, r = w2px(vpts[~near])
    canvas[r, c] = (220, 0, 0)

    img = Image.fromarray(canvas).resize((W * a.scale, H * a.scale), Image.NEAREST)
    d = ImageDraw.Draw(img)
    # 机器人（地图原点）
    col = (0.0 - mm['ox']) / mm['res'] - 0.5
    row = (mh - 1) - ((0.0 - mm['oy']) / mm['res'] - 0.5)
    px, py = col * a.scale, row * a.scale
    d.line([(px - 8, py), (px + 8, py)], fill=(0, 0, 255), width=2)
    d.line([(px, py - 8), (px, py + 8)], fill=(0, 0, 255), width=2)
    d.ellipse([px - 6, py - 6, px + 6, py + 6], outline=(0, 0, 255), width=2)
    img.save(a.out)
    print(f'[overlay] 写出 {a.out}  ({W * a.scale}x{H * a.scale})  绿=一致 红=孤立 深灰=静态占据')
    return 0


if __name__ == '__main__':
    sys.exit(main())
