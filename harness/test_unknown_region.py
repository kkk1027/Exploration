#!/usr/bin/env python3
"""P1/P2 回归测试：未知连通分量特征 + 候选池快照探针。

背景（2026-11-11，用户批准的"候选落点倾向"改造前置准备）：
  候选点永远站在已知/未知交界线上，它的**自身距离**没有"区域"含义。
  要回答"这个候选打开的未知区有多大、在哪个方向、离车多远"，必须先做
  未知格连通分量（8 邻域），再把候选归属到它旁边**面积最大**的分量上。

本测试锁定六条性质：
  A 分量划分正确：两块不相邻的未知区 → 两个分量
  B 归属规则：候选旁边同时有大小两块时，归到**大的**那块
  C 噪声过滤：只挨着 <ureg_min_cells 的小分量时，归属为 0（不参与排序）
  D 距离三元组自洽：near <= cen <= far，且都 <= 地图对角线
  E 前沿最远距离只统计"紧邻已知空闲"的未知格（不能取到孤立未知块）
  F 候选池快照落盘：一行一拍、行内含 rank/dist/reg_* /far_edge_m
  H 未知厚度场：大块中心厚、小/薄块薄（连通分量退化后的主区域特征）

用法：python3 harness/test_unknown_region.py
"""
import json
import os
import sys
import tempfile
import time
from types import SimpleNamespace

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARB = os.path.join(ROOT, 'src', 'stemm_cartographer_exploration_sim',
                   'scripts', 'frontier_arbiter')

FREE, OCC, UNK = 0, 100, -1


def _load():
    """把无扩展名的 frontier_arbiter 当模块载入（只取纯方法，不初始化节点）。"""
    src = open(ARB, encoding='utf-8').read()
    start = src.index('    # ================= P1（2026-11-11）未知区连通分量')
    end = src.index('    # ---- 工具 ----')
    body = src[start:end]
    ns = {'np': np, 'os': os, 'json': json, 'time': time}
    exec(compile('class _Shim:\n    def __init__(self):\n        pass\n' + body,  # noqa: S102
                 '<shim>', 'exec'), ns)
    return ns['_Shim']


def _make_map(w, h, res, ox, oy, data):
    info = SimpleNamespace(resolution=res, width=w, height=h,
                           origin=SimpleNamespace(
                               position=SimpleNamespace(x=ox, y=oy)))
    return SimpleNamespace(info=info, data=list(data))


def _blank(w, h, fill=FREE):
    return np.full((h, w), fill, dtype=np.int16)


def _shim(cls, m, pose=(0.0, 0.0), **kw):
    o = cls()
    o.map = m
    o.pose = pose
    o._map_seq = 1
    o._ureg_key = None
    o._ureg_field = None
    o._ureg_pts = {}
    o._pool_tick = 0
    o.cands = []
    o.stats = {}
    o.ureg_enabled = kw.get('ureg_enabled', True)
    o.ureg_radius = kw.get('ureg_radius', 0.5)
    o.ureg_min_cells = kw.get('ureg_min_cells', 40)
    o.pool_probe_n = kw.get('pool_probe_n', 12)
    o.pool_probe_log = kw.get('pool_probe_log', '')
    o.strategy_phase = 'fill'
    o._known_key = None
    o._known_n = 0
    return o


def _scene():
    """120x120 @5cm = 6x6 m。两块未知：大的 30x30（1.5 m²）、小的 4x4（0.04 m²）。"""
    w = h = 120
    arr = _blank(w, h, FREE)
    arr[20:50, 20:50] = UNK          # 大块：900 格 = 2.25 m²
    arr[90:94, 90:94] = UNK          # 小块：16 格，低于 min_cells=40
    arr[10:14, 100:104] = OCC
    res, ox, oy = 0.05, 0.0, 0.0
    return _make_map(w, h, res, ox, oy, arr.ravel()), arr, (w, h, res, ox, oy)


def _c2w(i, j, res=0.05, ox=0.0, oy=0.0):
    """格坐标 → 世界坐标（取格中心）。"""
    return ox + (i + 0.5) * res, oy + (j + 0.5) * res


def run():
    ok = 0
    bad = []
    cls = _load()
    m, arr, (w, h, res, ox, oy) = _scene()

    # ---- A 分量划分 ----
    o = _shim(cls, m)
    f = o.unknown_region_field()
    if f is not None and f['n'] >= 2:
        ok += 1
        print('  A ✓ 未知分量数 = %d（>=2）' % f['n'])
    else:
        bad.append('A 分量划分：%r' % (None if f is None else f['n']))

    sizes = f['sizes'] if f else []
    big = int(np.argmax(sizes)) + 1 if len(sizes) else 0

    # ---- B 归属规则：候选在大块边缘（旁边同时有大小块不可能，故分两点验证） ----
    # 大块左下角外侧（已知格，但 0.5 m 内有大块未知）
    px, py = _c2w(18, 18)
    rid_big = o.unknown_region_at(px, py)
    if rid_big == big and rid_big > 0:
        ok += 1
        print('  B ✓ 大块旁候选 → 分量 %d（最大块）' % rid_big)
    else:
        bad.append('B 归属：期望 %d 实得 %d' % (big, rid_big))

    # 两块同时出现在半径内时优先大的：构造候选位于两块之间且半径覆盖两者
    o2 = _shim(cls, m, ureg_radius=5.0)
    px2, py2 = _c2w(70, 70)
    rid_both = o2.unknown_region_at(px2, py2)
    if rid_both == big:
        ok += 1
        print('  B ✓ 半径覆盖两块时取面积最大者（%d）' % rid_both)
    else:
        bad.append('B 取大块：期望 %d 实得 %d' % (big, rid_both))

    # ---- C 噪声过滤 ----
    o3 = _shim(cls, m, ureg_radius=0.15)
    sx, sy = _c2w(91, 91)
    rid_small = o3.unknown_region_at(sx, sy)
    if rid_small == 0:
        ok += 1
        print('  C ✓ 仅挨着小分量（16 格 < 40）→ 归属 0')
    else:
        bad.append('C 噪声过滤：期望 0 实得 %d' % rid_small)

    # ---- D 距离三元组自洽 ----
    o4 = _shim(cls, m, pose=(1.0, 1.0))
    span = o4.region_span(big, pose=(1.0, 1.0))
    diag = np.hypot(w * res, h * res)
    if span is not None:
        near, cen, far = span
        if near <= cen + 1e-9 and cen <= far + 1e-9 and far <= diag + 1e-6:
            ok += 1
            print('  D ✓ 距离 near=%.2f cen=%.2f far=%.2f <= 对角线 %.2f'
                  % (near, cen, far, diag))
        else:
            bad.append('D 三元组不自洽：%r' % (span,))
    else:
        bad.append('D region_span 返回 None')

    # ---- E 前沿最远距离只算"紧邻已知空闲"的未知格 ----
    arr_e = _blank(w, h, FREE)
    arr_e[30:40, 30:40] = UNK        # 前沿块（紧邻空闲）
    arr_e[100:110, 100:110] = UNK    # 孤立块（四周被占据围死）
    arr_e[95:115, 95:115] = OCC
    arr_e[100:110, 100:110] = UNK
    me = _make_map(w, h, res, ox, oy, arr_e.ravel())
    o5 = _shim(cls, me, pose=(0.0, 0.0))
    far_edge = o5.unknown_edge_far()
    # 孤立块中心 ≈ (5.25,5.25)，前沿块最远角 ≈ (2.0,2.0)
    if far_edge is not None and far_edge < 4.0:
        ok += 1
        print('  E ✓ 前沿最远 %.2f m（排除了被围死的孤立块 ≈7.4 m）' % far_edge)
    else:
        bad.append('E 前沿最远：%r（应 <4 m）' % (far_edge,))

    # ---- F 候选池快照 ----
    tmp = tempfile.NamedTemporaryFile(suffix='.jsonl', delete=False)
    tmp.close()
    o6 = _shim(cls, m, pose=(0.5, 0.5), pool_probe_log=tmp.name, pool_probe_n=3)
    o6.cands = [(1.0, 1.0, 0.0)] * 7
    scored = [(900.0, -1.5, 1.10, 1.10),
              (800.0, -2.0, 1.20, 1.20),
              (700.0, -3.0, 1.30, 1.30),
              (600.0, -4.0, 1.40, 1.40)]
    raw_of = {(1.10, 1.10): 1000.0, (1.20, 1.20): 900.0,
              (1.30, 1.30): 800.0, (1.40, 1.40): 700.0}
    o6.probe_pool(scored, raw_of)
    rows = [json.loads(x) for x in open(tmp.name, encoding='utf-8')]
    r0 = rows[0] if rows else {}
    keys = {'tick', 'robot', 'n_pool', 'n_scored', 'n_regions', 'far_edge_m', 'rows'}
    need = {'rank', 'x', 'y', 'dist', 'u_eff', 'key', 'reg', 'reg_m2',
            'reg_near', 'reg_cen', 'reg_far', 'depth'}
    if (len(rows) == 1 and keys.issubset(r0)
            and len(r0['rows']) == 3
            and need.issubset(r0['rows'][0])
            and r0['n_pool'] == 7 and r0['n_scored'] == 4):
        ok += 1
        print('  F ✓ 快照 1 行 / 3 名 / 字段齐全（n_pool=7 n_scored=4）')
    else:
        bad.append('F 快照字段不全：%r' % (r0,))
    os.unlink(tmp.name)

    # ---- H 未知厚度场：大块中心厚、小/薄块薄 ----
    arr_h = _blank(w, h, FREE)
    arr_h[20:50, 20:50] = UNK        # 大块 30x30 格 = 1.5 m 见方
    arr_h[100:104, 40:44] = UNK      # 小块 4x4 格 = 0.2 m 见方
    mh = _make_map(w, h, res, ox, oy, arr_h.ravel())
    o8 = _shim(cls, mh)
    d_big = o8.unknown_depth_at(*_c2w(35, 35), radius=0.5)
    d_small = o8.unknown_depth_at(*_c2w(101, 41), radius=0.5)
    if (d_big is not None and d_small is not None
            and d_big > d_small * 3 and 0.2 <= d_big <= 1.2):
        ok += 1
        print('  H ✓ 未知厚度：大块 %.2f m >> 小块 %.2f m' % (d_big, d_small))
    else:
        bad.append('H 厚度场：big=%r small=%r' % (d_big, d_small))

    # ---- G 关闭开关后必须完全退化 ----
    o7 = _shim(cls, m, ureg_enabled=False)
    if o7.unknown_region_field() is None and o7.unknown_region_at(1.0, 1.0) == 0:
        ok += 1
        print('  G ✓ ureg_enabled=False → 特征全关（调用方退回旧行为）')
    else:
        bad.append('G 关闭开关无效')

    print()
    total = ok + len(bad)
    if bad:
        print('❌ %d/%d 通过' % (ok, total))
        for b in bad:
            print('   -', b)
        return 1
    print('✅ %d/%d 全部通过' % (ok, total))
    return 0


if __name__ == '__main__':
    sys.exit(run())
