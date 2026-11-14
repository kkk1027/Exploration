#!/usr/bin/env python3
"""contact_forensics.py —— 把"物理接触"从日志现象变成可判定的几何结论。

为什么需要它
------------
只看 probe.jsonl 里的 `type=contact` 只能知道"什么时候、在哪儿、撞了谁"，
无法回答"为什么安全判据没拦住"。本工具用**本轮最终地图**做欧氏距离变换，
对每一次接触算三个几何量：

  clear_at_contact  该位姿到最近占用格的距离
  circumscribed     车体外接半径（base_footprint 系 = hypot(0.29,0.23)=0.370）
  deficit           circumscribed − clear_at_contact
                    > 0 ⇒ 车体必然与障碍重叠，这就是"接触"的物理定义
  spawn_clear       出生点净空（用来判断"起点本身选错了"这一类）

用法
----
    python3 harness/contact_forensics.py                # 最新一轮
    python3 harness/contact_forensics.py logs/round_xxx
    python3 harness/contact_forensics.py --all          # 所有含 map.pgm 的轮
"""
import argparse
import glob
import json
import math
import os
import sys

try:
    import numpy as np
    from scipy import ndimage
except Exception:                                    # noqa: BLE001
    print('✗ 需要 numpy + scipy', file=sys.stderr)
    sys.exit(3)

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# URDF 实读（wheeltec_two_wheel_model/_t_classic.urdf.xacro）
BODY_X_MIN, BODY_X_MAX, BODY_Y_HALF = -0.16, 0.29, 0.23
CIRCUM = math.hypot(BODY_X_MAX, BODY_Y_HALF)         # 0.370
NAV2_CIRCUM = math.hypot(0.32, 0.235)                # 0.397（footprint 含 padding）


def read_pgm(path):
    with open(path, 'rb') as f:
        raw = f.read()
    idx, tokens = 0, []
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
    w, h = int(tokens[1]), int(tokens[2])
    idx += 1
    return np.frombuffer(raw[idx:idx + w * h], dtype=np.uint8).reshape(h, w)


def read_map_yaml(path):
    res = ox = oy = None
    for line in open(path, encoding='utf-8', errors='replace'):
        s = line.strip()
        if s.startswith('resolution:'):
            res = float(s.split(':', 1)[1])
        elif s.startswith('origin:'):
            parts = [p for p in s.split(':', 1)[1].strip()
                     .strip('[]').replace(',', ' ').split() if p]
            ox, oy = float(parts[0]), float(parts[1])
    return res, ox, oy


def spawn_pose(round_dir):
    import re
    p = os.path.join(round_dir, 'harness.log')
    if not os.path.exists(p):
        return None
    pat = re.compile(r'随机起点：x=(-?[\d.]+)\s+y=(-?[\d.]+)\s+yaw=(-?[\d.]+)')
    for line in open(p, encoding='utf-8', errors='replace'):
        m = pat.search(line)
        if m:
            return tuple(float(m.group(i)) for i in (1, 2, 3))
    return None


def analyse(round_dir):
    pgm = os.path.join(round_dir, 'map.pgm')
    yml = os.path.join(round_dir, 'map.yaml')
    probe = os.path.join(round_dir, 'probe.jsonl')
    if not (os.path.exists(pgm) and os.path.exists(yml) and os.path.exists(probe)):
        return None
    res, ox, oy = read_map_yaml(yml)
    if not res or res <= 0:
        return None
    img = read_pgm(pgm)
    h, w = img.shape
    occ = img < 65
    dist = ndimage.distance_transform_edt(~occ) * res

    def clearance(x, y):
        col = int((x - ox) / res)
        row = h - 1 - int((y - oy) / res)       # map.yaml origin 在左下角
        if not (0 <= row < h and 0 <= col < w):
            return None
        return float(dist[row, col])

    contacts = []
    for line in open(probe, encoding='utf-8', errors='replace'):
        try:
            r = json.loads(line)
        except Exception:                        # noqa: BLE001
            continue
        if r.get('type') != 'contact':
            continue
        if r.get('src') not in ('chassis', 'wheel'):
            continue
        pos = r.get('pos') or [None, None]
        if pos[0] is None:
            continue
        nav = {}
        try:
            nav = json.loads(r.get('nav') or '{}')
        except Exception:                        # noqa: BLE001
            pass
        c = clearance(pos[0], pos[1])
        contacts.append({
            'sim': r.get('sim'), 'src': r.get('src'),
            'pair': (r.get('pair') or '').split('|')[-1][:56],
            'pos': pos, 'clear': c,
            'mode': nav.get('mode'), 'state': nav.get('navigation_state'),
            'cmd': r.get('cmd'),
            'front': nav.get('front_min_range'),
            'left': nav.get('left_min_range'),
            'right': nav.get('right_min_range'),
        })
    sp = spawn_pose(round_dir)
    spawn_clear = None
    if sp is not None:
        spawn_clear = clearance(0.0, 0.0)        # 地图系原点 = 出生位姿
    return {
        'round': os.path.basename(round_dir),
        'contacts': contacts,
        'spawn': sp, 'spawn_clear': spawn_clear,
    }


def report(info):
    if info is None:
        return
    n = len(info['contacts'])
    print('=' * 78)
    print('%s   物理接触帧 = %d' % (info['round'], n))
    if info['spawn_clear'] is not None:
        flag = ''
        if info['spawn_clear'] < CIRCUM:
            flag = '   ← 出生点净空 < 车体外接 %.3f，起手就会碰' % CIRCUM
        elif info['spawn_clear'] < CIRCUM + 0.10:
            flag = '   ← 出生点只剩 %.3f m 余量（危险）' % (
                info['spawn_clear'] - CIRCUM)
        print('  出生点净空 %.3f m（车体外接 %.3f / Nav2 %.3f）%s'
              % (info['spawn_clear'], CIRCUM, NAV2_CIRCUM, flag))
    if not n:
        print('  ✓ 无物理接触')
        return
    # 首个接触帧最有价值
    first = info['contacts'][0]
    print('  首次接触：sim=%s  src=%s' % (first['sim'], first['src']))
    print('    对象     %s' % first['pair'])
    print('    位姿     (%.3f, %.3f)  净空=%s m'
          % (first['pos'][0], first['pos'][1],
             ('%.3f' % first['clear']) if first['clear'] is not None else '?'))
    if first['clear'] is not None:
        deficit = CIRCUM - first['clear']
        verdict = ('车体必然重叠 %.3f m ⇒ 几何上已经接触' % deficit
                   if deficit > 0 else
                   '净空仍够（%.3f m 余量）⇒ 接触由局部姿态/角点造成' % (-deficit))
        print('    判定     %s' % verdict)
    print('    manager  mode=%s state=%s' % (first['mode'], first['state']))
    print('    cmd      %s' % (first['cmd'],))
    print('    测距     front=%s left=%s right=%s'
          % (first['front'], first['left'], first['right']))
    # 按对象/状态聚类
    from collections import Counter
    c1 = Counter((x['src'], x['mode'], x['state'], x['pair']) for x in info['contacts'])
    print('  分布：')
    for k, v in c1.most_common(6):
        print('    %-6s %-14s %-26s %-56s %d'
              % (k[0], k[1], k[2], k[3], v))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('round', nargs='?', default=None)
    ap.add_argument('--all', action='store_true')
    a = ap.parse_args()
    if a.all:
        dirs = sorted(glob.glob(os.path.join(BASE, 'logs', 'round_*')))
        for d in dirs:
            info = analyse(d)
            if info is not None:
                report(info)
        return 0
    d = a.round
    if d is None:
        cand = [x for x in sorted(glob.glob(os.path.join(BASE, 'logs', 'round_*')))
                if os.path.exists(os.path.join(x, 'map.pgm'))]
        if not cand:
            print('✗ 找不到含 map.pgm 的 round', file=sys.stderr)
            return 3
        d = cand[-1]
    info = analyse(d)
    if info is None:
        print('✗ %s 缺 map.pgm / map.yaml / probe.jsonl' % d, file=sys.stderr)
        return 3
    report(info)
    return 0


if __name__ == '__main__':
    sys.exit(main())
