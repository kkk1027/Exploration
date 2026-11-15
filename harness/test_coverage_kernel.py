#!/usr/bin/env python3
"""coverage_kernel 单元测试（P2 覆盖率终止判据内核）。

覆盖点
------
A. OccupancyGrid / PGM 两套输入适配，语义必须一致
B. PGM 行序（row = h-1-my）—— 2026-11-15 踩过一次，锁住
C. 可通行已知：净空过滤（未知按障碍算，保守）
D. 可达未知：允许穿未知、被墙隔开的不算
E. 外部未知剔除（这是指标能不能用的关键）
F. ConvergenceTracker：连续确认 + 停滞兜底 + 记账区分
G. 边界：起点在地图外 / 空图 / 分母为 0
H. 单调性：探明格数增加时 NCR 不应下降（同一起点）
I. 性能：不超出预算

运行
----
  python3 harness/test_coverage_kernel.py
"""
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
# 内核归属 stemm_nav2_manager —— 这是依赖方向的正确一侧：
# stemm_cartographer_exploration 是它的子类包（cartographer_manager 继承
# StemmNav2Manager），反向 import 会循环。
REAL_PKG = '/home/kbf/wheeltec_ros2/src/stemm_nav2_manager'
sys.path.insert(0, REAL_PKG)

from stemm_nav2_manager.coverage_kernel import (   # noqa: E402
    ConvergenceTracker, coverage_stats, is_converged,
    masks_from_occupancy_grid, masks_from_pgm,
)

PASS = 0
FAIL = 0
FAILS = []


def check(name, cond, detail=''):
    global PASS, FAIL
    if cond:
        PASS += 1
        print('  ✓ %s' % name)
    else:
        FAIL += 1
        FAILS.append(name)
        print('  ✗ %s %s' % (name, detail))


def blank(h, w, value=-1):
    """构造 OccupancyGrid 风格栅格。"""
    return np.full((h, w), value, dtype=np.int16)


def grid_of(rows):
    """用字符画构造栅格：'.'=free(0)  '#'=occupied(100)  '?'=unknown(-1)

    行按「自上而下」书写（第 0 行 = 地图最高 y），符合人看图习惯；
    内部会翻转成 OccupancyGrid 顺序（第 0 行 = 最低 y）。
    """
    h = len(rows)
    w = len(rows[0])
    arr = blank(h, w)
    for r, line in enumerate(rows):
        for c, ch in enumerate(line):
            arr[h - 1 - r, c] = 0 if ch == '.' else (100 if ch == '#' else -1)
    return arr, w, h


def stats(rows, rx=None, ry=None, res=0.05, ox=0.0, oy=0.0, inscribed=0.0):
    """用字符画跑一次 coverage_stats；默认把机器人放在图中心。"""
    arr, w, h = grid_of(rows)
    free, unknown = masks_from_occupancy_grid(arr, w, h)
    if rx is None:
        rx = ox + (w // 2) * res
    if ry is None:
        ry = oy + (h // 2) * res
    return coverage_stats(free, unknown, res, ox, oy, rx, ry, inscribed)


print('\n=== A. 两套输入适配一致性 ===')

# 同一个几何，分别用 OccupancyGrid 与 PGM 表达，结果必须一致
rows = [
    '........',
    '........',
    '..####..',
    '........',
    '........',
]
arr, w, h = grid_of(rows)
free_g, unk_g = masks_from_occupancy_grid(arr, w, h)

# PGM：0=occ 205=unknown 254=free，且行自下而上（与 occupancy 相反）
pgm = np.full((h, w), 254, dtype=np.uint8)
pgm[arr != 0] = 205
pgm[arr == 100] = 0
free_p, unk_p = masks_from_pgm(pgm)
check('A1 两套适配的 free_mask 完全一致', np.array_equal(free_g, free_p))
check('A2 两套适配的 unknown_mask 完全一致', np.array_equal(unk_g, unk_p))

print('\n=== B. PGM 行序（row = h-1-my） ===')

# PGM 第 0 行（像素最上面一行）对应世界最高 y。
# 构造只有最上一行是空闲的图，检查翻转后落在正确的世界位置。
h2, w2 = 10, 10
pgm2 = np.full((h2, w2), 205, dtype=np.uint8)
pgm2[0, :] = 254                       # 像素第 0 行 = 世界最高 y
free2, unk2 = masks_from_pgm(pgm2)
check('B1 翻转后空闲行落在索引 h-1（世界最高 y）',
      bool(free2[h2 - 1, :].all()), 'free2 最高行为 %s' % free2[h2 - 1, :].all())
check('B2 翻转后第 0 行（世界最低 y）仍是未知',
      bool(unk2[0, :].all()))

# 用真实 PGM 再验一次：已知角与实际文件一致
real = ('/home/kbf/stemm_exploration_sim_ws/logs/'
        'round_20261114-194944_p14vS_far1/map.pgm')
if os.path.exists(real):
    sys.path.insert(0, ROOT)
    try:
        from probe_ncr import load_pgm                  # noqa: E402
        pix, rw, rh = load_pgm(real)
        f_real, u_real = masks_from_pgm(pix)
        check('B3 真实 PGM 适配后形状正确', f_real.shape == (rh, rw))
        # 相对覆盖率自洽：空闲 + 未知 + 占据 = 全图
        occ = int((~f_real & ~u_real).sum())
        check('B4 三类互斥且覆盖全图',
              int(f_real.sum()) + int(u_real.sum()) + occ == rw * rh)
    except Exception as exc:                        # noqa: BLE001
        check('B3 真实 PGM 可读取', False, str(exc))

print('\n=== C. 可通行已知的净空过滤 ===')

# 一条 1 格宽的走廊：内切半径 0 时应全部算可通行；
# 内切半径 > 1 格时应为 0（因为它贴着未知/墙）
rows = [
    '#########',
    '.........',
    '#########',
]
s0 = stats(rows, inscribed=0.0)
s1 = stats(rows, inscribed=0.20)
check('C1 内切半径 0 时可通行已知 > 0', s0['navigable_known'] > 0,
      'got %s' % s0['navigable_known'])
check('C2 1 格宽走廊在内切 0.20 m（4 格）时被全部排除',
      s1['navigable_known'] == 0, 'got %s' % s1['navigable_known'])

# 开阔区：内切 0.20 m 仍应有大量可通行格
rows = [
    '.........',
    '.........',
    '.........',
    '.........',
    '.........',
]
s = stats(rows, inscribed=0.20)
check('C3 开阔区在内切 0.20 m 下仍保留可通行格',
      s['navigable_known'] > 0, 'got %s' % s['navigable_known'])

print('\n=== D. 可达未知 ===')

# 左右两块未知，中间被墙完全隔开；机器人左侧
rows = [
    '???##???',
    '...##...',
    '???##???',
]
arr, w, h = grid_of(rows)
free, unknown = masks_from_occupancy_grid(arr, w, h)
res = 0.05
# 机器人放在左侧空闲区
rx = 1 * res
ry = 1 * res
d = coverage_stats(free, unknown, res, 0.0, 0.0, rx, ry, 0.0)
check('D1 被墙隔开的右侧未知不计入可达未知',
      d['reach_unknown'] < int(unknown.sum()),
      'reach=%s total_unknown=%s' % (d['reach_unknown'], int(unknown.sum())))
check('D2 起点连通域有效', d['reach_ok'] is True)

print('\n=== E. 外部未知剔除（指标可用性的关键） ===')

# 外圈全是未知（模拟房子外部），内部一个封闭房间全是空闲
rows = [
    '???????',
    '?.....?',
    '?.....?',
    '?.....?',
    '???????',
]
s = stats(rows, inscribed=0.0)
ext = s['reach_unknown_exterior']
check('E1 存在外部未知', ext > 0, 'ext=%s' % ext)
check('E2 外部未知几乎等于全部可达未知（此例无内部未知）',
      s['reach_unknown_interior'] == 0,
      'interior=%s' % s['reach_unknown_interior'])
check('E3 原始 NCR 明显低于内部 NCR',
      s['ncr'] is not None and s['ncr_interior'] is not None
      and s['ncr_interior'] > s['ncr'],
      'ncr=%s interior=%s' % (s['ncr'], s['ncr_interior']))
check('E4 全空闲房间内部 NCR == 1.0', abs(s['ncr_interior'] - 1.0) < 1e-9,
      'got %s' % s['ncr_interior'])

# 内部也有未知：内部 NCR 必须 < 1
rows = [
    '???????',
    '?.....?',
    '?...?.?',
    '?.....?',
    '???????',
]
s = stats(rows, inscribed=0.0)
check('E5 内部有未探区时内部 NCR < 1.0', s['ncr_interior'] < 1.0,
      'got %s' % s['ncr_interior'])
check('E6 内部未知被正确计入 reach_unknown_interior',
      s['reach_unknown_interior'] > 0,
      'got %s' % s['reach_unknown_interior'])

print('\n=== F. ConvergenceTracker ===')

t = ConvergenceTracker(threshold=0.95, confirm_n=3, stall_sec=0.0)
r = [t.update(0.96, 0.0), t.update(0.96, 1.0), t.update(0.96, 2.0)]
check('F1 连续 3 次达标才收敛',
      [x[0] for x in r] == [False, False, True], str([x[0] for x in r]))
check('F2 收敛原因记为 ncr_threshold', r[-1][2] == 'ncr_threshold')
check('F3 非停滞', r[-1][1] is False)

t = ConvergenceTracker(threshold=0.95, confirm_n=3)
seq = [t.update(0.96, 0.0), t.update(0.90, 1.0), t.update(0.96, 2.0),
       t.update(0.96, 3.0), t.update(0.96, 4.0)]
check('F4 中断后重新计数（第 3 次达标才收敛）',
      [x[0] for x in seq] == [False, False, False, False, True],
      str([x[0] for x in seq]))

t = ConvergenceTracker(threshold=0.99, confirm_n=3, stall_sec=10.0,
                       stall_delta=0.002)
# NCR 完全不动 ⇒ 10 s 后停滞收敛
outs = [t.update(0.90, float(i)) for i in range(0, 12)]
converged_at = [i for i, o in enumerate(outs) if o[0]]
check('F5 停滞兜底会触发', bool(converged_at), '未触发')
if converged_at:
    check('F6 停滞触发时间约在 10 s 后（容差 1 s）',
         10 <= converged_at[0] <= 11, 'at %s' % converged_at[0])
    check('F7 停滞记账与真收敛区分', outs[converged_at[0]][1] is True)
    check('F8 原因为 ncr_stalled', outs[converged_at[0]][2] == 'ncr_stalled')

# ★ 对照：NCR 缓慢但持续爬升（每步 0.003 > stall_delta）⇒ **不该**判停滞。
#   这是「更晚收敛」的语义保障：还在进步就不许停。
t = ConvergenceTracker(threshold=0.99, confirm_n=3, stall_sec=5.0,
                       stall_delta=0.002)
slow_climb = [t.update(0.90 + 0.003 * i, float(i)) for i in range(0, 15)]
check('F13 缓慢但持续爬升时不判停滞（更晚收敛的语义保障）',
      not any(o[0] for o in slow_climb),
      str([o for o in slow_climb if o[0]][:2]))

t = ConvergenceTracker(threshold=0.99, confirm_n=3, stall_sec=10.0)
t.update(0.80, 0.0)
t.update(0.85, 5.0)            # 涨了 0.05 > delta ⇒ 停滞计时重置
out = t.update(0.85, 12.0)
check('F9 有实质进展时停滞计时重置（12 s 未触发）', out[0] is False,
      str(out))

t = ConvergenceTracker(threshold=0.95, confirm_n=3)
out = t.update(None, 0.0)
check('F10 NCR 不可用时不收敛且不崩', out == (False, False, 'ncr_unavailable'))

check('F11 is_converged 对 None 返回 False', is_converged(None, 0.95) is False)
check('F12 is_converged 边界相等算收敛', is_converged(0.95, 0.95) is True)

print('\n=== G. 边界条件 ===')

rows = ['.....', '.....', '.....']
arr, w, h = grid_of(rows)
free, unknown = masks_from_occupancy_grid(arr, w, h)
# 起点远在地图外
g = coverage_stats(free, unknown, 0.05, 0.0, 0.0, 999.0, 999.0, 0.0)
check('G1 起点在图外时 start_in_bounds=False', g['start_in_bounds'] is False)
check('G2 起点在图外不会崩，且返回裁剪后的索引',
      isinstance(g['start_cell'], tuple))

# 全未知
arr = blank(5, 5)
free, unknown = masks_from_occupancy_grid(arr, 5, 5)
g = coverage_stats(free, unknown, 0.05, 0.0, 0.0, 0.1, 0.1, 0.0)
check('G3 全未知时 ncr 为 None（分母为 0 或起点无效）',
      g['ncr'] is None or g['ncr'] == 0.0, 'got %s' % g['ncr'])

# 全占据
arr = blank(5, 5, 100)
free, unknown = masks_from_occupancy_grid(arr, 5, 5)
g = coverage_stats(free, unknown, 0.05, 0.0, 0.0, 0.1, 0.1, 0.0)
check('G4 全占据时不崩', g['navigable_known'] == 0)

print('\n=== H. 单调性 ===')

# 同一起点，逐步把未知变空闲，NCR 不应下降
rows_base = [
    '???????',
    '?.....?',
    '?...?.?',
    '?.....?',
    '???????',
]
prev = None
ok_mono = True
detail = []
for k in range(0, 4):
    rows = list(rows_base)
    if k >= 1:
        rows[2] = '?...?..' if k == 1 else (
            '?...?..' if k == 2 else '?...?..')
    if k == 3:
        rows = [
            '???????',
            '?.....?',
            '?.....?',
            '?.....?',
            '???????',
        ]
    s = stats(rows, inscribed=0.0)
    v = s['ncr_interior']
    if prev is not None and v is not None and prev is not None and v < prev - 1e-9:
        ok_mono = False
        detail.append('k=%d %s < %s' % (k, v, prev))
    prev = v
check('H1 探明格增加时内部 NCR 单调不降', ok_mono, ' '.join(detail))

print('\n=== I. 性能 ===')

h3, w3 = 400, 240                     # 比实测地图（387x238）略大
rng = np.random.default_rng(7)
arr = np.full((h3, w3), -1, dtype=np.int16)
arr[50:350, 40:200] = 0
arr[100:110, 60:180] = 100
free, unknown = masks_from_occupancy_grid(arr, w3, h3)
t0 = time.time()
n = 20
for _ in range(n):
    coverage_stats(free, unknown, 0.05, -1.9, -11.0, 1.4, -1.2, 0.235)
ms = (time.time() - t0) / n * 1000.0
check('I1 单次耗时 < 40 ms（实测通常 8~12 ms）', ms < 40.0, '%.1f ms' % ms)
print('    实测 %.1f ms' % ms)

print('\n' + '=' * 60)
print('coverage_kernel: %d 通过, %d 失败' % (PASS, FAIL))
if FAILS:
    print('失败项:')
    for f in FAILS:
        print('  - %s' % f)
sys.exit(0 if FAIL == 0 else 1)
