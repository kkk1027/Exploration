#!/usr/bin/env python3
"""pick_spawn.py —— 按"可达性/净空"从真实地图上重定起点（出生点）。

为什么需要它（2026-11-14 实测）
-------------------------------
`p14vF` 三轮的物理接触全部发生在 sim≈23（出生后第一个目标刚下发），
用最终 `map.pgm` 做距离变换后可以定量说明：

| 位置 | 到最近占用格 | 车体外接半径 |
|---|---|---|
| 出生点 (0,0) 地图系 | **0.400 m** | 0.370（base_footprint 系）/ 0.397（Nav2 footprint） |
| 接触位姿 (-0.047,0.127) | **0.350 m** | 同上 |

即：出生点只剩 3 cm 余量，车一动 0.14 m 净空就掉到 0.350 m < 0.370 m，
车角必然扫进墙里。这与控制器参数无关，是**起点本身选错了**。

本工具在最新一轮地图上，以"出生点为圆心的搜索域"内找净空最大的若干格，
并把地图坐标换算回 **世界坐标**（地图系原点 = 出生位姿），供 `round.sh`
的 `POSES` 使用。

用法
----
    python3 harness/pick_spawn.py                       # 用最新一轮
    python3 harness/pick_spawn.py --round <dir>
    python3 harness/pick_spawn.py --radius 1.5 --n 5 --min-clear 1.0
    python3 harness/pick_spawn.py --emit-shell          # 输出 POSES=( ... ) 行

退出码：0 成功；2 找不到满足阈值的点；3 输入不可用。
"""
import argparse
import glob
import math
import os
import sys

try:
    import numpy as np
except Exception:                                    # noqa: BLE001
    print('✗ 需要 numpy', file=sys.stderr)
    sys.exit(3)

try:
    from scipy import ndimage
except Exception:                                    # noqa: BLE001
    ndimage = None


def read_pgm(path):
    """读 P5/P2 PGM，返回 (H, W) uint8 数组。"""
    with open(path, 'rb') as f:
        raw = f.read()
    idx = 0
    tokens = []
    while len(tokens) < 4:
        while idx < len(raw) and raw[idx:idx + 1].isspace():
            idx += 1
        if raw[idx:idx + 1] == b'#':
            while idx < len(raw) and raw[idx:idx + 1] != b'\n':
                idx += 1
            continue
        start = idx
        while idx < len(raw) and not raw[idx:idx + 1].isspace():
            idx += 1
        tokens.append(raw[start:idx])
    magic = tokens[0].decode()
    w, h, _maxv = int(tokens[1]), int(tokens[2]), int(tokens[3])
    idx += 1
    if magic == 'P5':
        img = np.frombuffer(raw[idx:idx + w * h], dtype=np.uint8).reshape(h, w)
    else:                                            # P2 明码
        vals = raw[idx:].split()
        img = np.array([int(v) for v in vals[:w * h]], dtype=np.uint8).reshape(h, w)
    return img


def read_map_yaml(path):
    """极简 map.yaml 解析：只需要 resolution 与 origin。"""
    res = None
    ox = oy = None
    for line in open(path, encoding='utf-8', errors='replace'):
        s = line.strip()
        if s.startswith('resolution:'):
            res = float(s.split(':', 1)[1])
        elif s.startswith('origin:'):
            body = s.split(':', 1)[1].strip().strip('[]')
            parts = [p for p in body.replace(',', ' ').split() if p]
            ox, oy = float(parts[0]), float(parts[1])
    return res, ox, oy


def parse_check(round_dir, name):
    """从 report/evaluate 里捞本轮的出生位姿（世界系）。"""
    import json
    cand = [
        os.path.join(round_dir, 'evaluate.json'),
        os.path.join(round_dir, 'report.stdout'),
    ]
    pose = None
    for p in cand:
        if not p.endswith('.json') or not os.path.exists(p):
            continue
        try:
            d = json.load(open(p, encoding='utf-8'))
        except Exception:                            # noqa: BLE001
            continue

        def walk(o):
            nonlocal pose
            if isinstance(o, dict):
                if o.get('check') == 'pose' and 'x' in o:
                    pose = (float(o['x']), float(o['y']),
                            float(o.get('yaw', 0.0)))
                for v in o.values():
                    walk(v)
            elif isinstance(o, list):
                for v in o:
                    walk(v)
        walk(d)
    return pose


def parse_pose_from_report(round_dir):
    """从 harness.log 的 "本轮随机起点：x=.. y=.. yaw=.." 行取出生位姿。"""
    import re
    p = os.path.join(round_dir, 'harness.log')
    if not os.path.exists(p):
        return None
    pat = re.compile(r'随机起点：x=(-?[\d.]+)\s+y=(-?[\d.]+)\s+yaw=(-?[\d.]+)')
    for line in open(p, encoding='utf-8', errors='replace'):
        m = pat.search(line)
        if m:
            return (float(m.group(1)), float(m.group(2)), float(m.group(3)))
    return None


def latest_round(base):
    dirs = sorted(glob.glob(os.path.join(base, 'logs', 'round_*')))
    dirs = [d for d in dirs if os.path.exists(os.path.join(d, 'map.pgm'))]
    return dirs[-1] if dirs else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--round', default=None, help='round 目录（默认取最新含 map.pgm 的）')
    ap.add_argument('--base', default=os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))), help='工作空间根')
    ap.add_argument('--radius', type=float, default=1.5,
                    help='以原点（本轮出生点）为圆心的搜索半径，米')
    ap.add_argument('--n', type=int, default=5, help='输出几个候选')
    ap.add_argument('--min-clear', type=float, default=1.00,
                    help='最小净空（到最近占用格），米。车体外接 0.370，建议 >= 0.8')
    ap.add_argument('--spread', type=float, default=0.30,
                    help='候选之间的最小间距，米（避免输出一堆相邻重复点）')
    ap.add_argument('--emit-shell', action='store_true',
                    help='输出可直接粘进 round.sh 的 POSES=( ... ) 行')
    a = ap.parse_args()

    round_dir = a.round or latest_round(a.base)
    if not round_dir or not os.path.isdir(round_dir):
        print('✗ 找不到含 map.pgm 的 round 目录', file=sys.stderr)
        return 3
    pgm = os.path.join(round_dir, 'map.pgm')
    yml = os.path.join(round_dir, 'map.yaml')
    if not (os.path.exists(pgm) and os.path.exists(yml)):
        print(f'✗ {round_dir} 缺 map.pgm / map.yaml', file=sys.stderr)
        return 3
    res, ox, oy = read_map_yaml(yml)
    if not res or res <= 0.0:
        print('✗ map.yaml 里没有合法 resolution', file=sys.stderr)
        return 3

    img = read_pgm(pgm)
    h, w = img.shape
    # trinary：占用格像素值低（0）。保守取 <65。
    occ = img < 65
    if ndimage is not None:
        dist = ndimage.distance_transform_edt(~occ) * res
    else:
        print('✗ 需要 scipy.ndimage', file=sys.stderr)
        return 3

    # 地图系 → 世界系：地图原点 = 本轮出生位姿
    spawn = parse_check(round_dir, 'pose') or parse_pose_from_report(round_dir)
    if spawn is None:
        print('⚠ 本轮出生位姿未知，按 map 原点 =(0,0,yaw=0) 换算', file=sys.stderr)
        spawn = (0.0, 0.0, 0.0)
    sx, sy, syaw = spawn

    def map_to_world(mx_, my_):
        c, s = math.cos(syaw), math.sin(syaw)
        return (sx + mx_ * c - my_ * s, sy + mx_ * s + my_ * c)

    # 收集候选（地图坐标系，以原点为圆心）
    cands = []
    r_lim = a.radius
    for row in range(h):
        my_ = oy + (h - 1 - row) * res          # map.yaml origin 在左下角
        if abs(my_) > r_lim:
            continue
        for col in range(w):
            mx_ = ox + col * res
            if math.hypot(mx_, my_) > r_lim:
                continue
            if occ[row, col]:
                continue
            cands.append((float(dist[row, col]), mx_, my_))
    cands.sort(key=lambda t: (-t[0], math.hypot(t[1], t[2])))

    picked = []
    for c, mx_, my_ in cands:
        if c < a.min_clear:
            break
        if any(math.hypot(mx_ - px, my_ - py) < a.spread for _, px, py in picked):
            continue
        picked.append((c, mx_, my_))
        if len(picked) >= a.n:
            break

    print(f'[pick_spawn] round = {os.path.basename(round_dir)}')
    print(f'  地图 {w}x{h} @ {res:.3f} m  占用格 {int(occ.sum())}')
    print(f'  出生位姿(世界) x={sx:.2f} y={sy:.2f} yaw={syaw:.4f}')
    print(f'  搜索半径 {a.radius:.2f} m  最小净空 {a.min_clear:.2f} m')
    print()
    if not picked:
        print('✗ 搜索域内没有满足净空阈值的格子（可放宽 --min-clear 或加大 --radius）',
              file=sys.stderr)
        return 2

    # 出生点自身净空，便于对比
    c0 = None
    if 0 <= int((0 - ox) / res) < w:
        row0 = h - 1 - int((0 - oy) / res)
        col0 = int((0 - ox) / res)
        if 0 <= row0 < h:
            c0 = float(dist[row0, col0])
    print(f'  当前位置（原点）净空 = {c0:.3f} m' if c0 is not None else '  当前位置净空 = ?')
    print('  候选（按净空降序）：')
    poses = []
    for i, (c, mx_, my_) in enumerate(picked, 1):
        wx, wy = map_to_world(mx_, my_)
        poses.append((wx, wy))
        print('    %d) 地图(%+.2f,%+.2f) 净空=%.3f m 距原点=%.2f m  '
              '→ 世界(x=%.2f, y=%.2f)'
              % (i, mx_, my_, c, math.hypot(mx_, my_), wx, wy))
    print()
    if a.emit_shell:
        # 朝向沿用本轮出生 yaw，保持"面朝同一方向"的可比性
        items = ' '.join('"%.4f %.4f %.4f"' % (wx, wy, syaw) for wx, wy in poses)
        print('POSES=(%s)' % items)
    return 0


if __name__ == '__main__':
    sys.exit(main())
