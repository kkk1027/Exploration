#!/usr/bin/env python3
"""S2 回归测试：`_path_lateral_widths` 的"沿法向两侧求和"必须满足三个性质。

背景（2026-11-11）：
  旧实现 width = 2 × 到最近障碍距离，只在"两侧对称"时成立。而前沿点永远站在
  已知/未知交界线上，典型形状是
      [已知墙] ←0.45 m→ [候选] ←开阔未知 4 m→
  此时旧值 0.90 < corridor_hard(0.92) 就被判死，而物理上另一侧有 4 m。

本测试锁定三条性质（前两条是新行为，第三条是安全约束）：
  A 对称走廊：新值 ≈ 旧值（不能凭空放大，否则会放过真窄缝）
  B 单侧障碍、另一侧开阔：新值 显著大于 旧值（这是要修的 bug）
  C 任意场景：新值 >= 旧值（旧值是理论下界，修完只能变宽不能变窄）

用法：python3 harness/test_lateral_width.py
"""
import importlib.util
import os
import sys

import numpy as np


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARB = os.path.join(ROOT, 'src', 'stemm_cartographer_exploration_sim',
                   'scripts', 'frontier_arbiter')


def _load():
    """把无扩展名的 frontier_arbiter 当模块载入（只取纯函数，不初始化节点）。"""
    src = open(ARB, encoding='utf-8').read()
    # 砍掉 rclpy 依赖：只保留 ast 出 _path_lateral_widths 需要的部分太重，
    # 这里用 exec 抽取单方法（方法体不依赖 ROS 类型）。
    start = src.index('    def _path_lateral_widths(self, field, path, idxs,')
    end = src.index('    def path_and_width(self, x, y):')
    body = src[start:end]
    ns = {'np': np}
    exec(compile('class _Shim:\n' + body, '<shim>', 'exec'), ns)  # noqa: S102
    return ns['_Shim']


def _make_field(w, h, res, ox, oy, blocked_rows, blocked_cols=()):
    """构造 (dist_m, ox, oy, res, w, h, unknown) —— 与 clearance_field() 同构。"""
    from scipy import ndimage
    blocked = np.zeros((h, w), dtype=bool)
    for r in blocked_rows:
        blocked[r, :] = True
    for c in blocked_cols:
        blocked[:, c] = True
    dist = ndimage.distance_transform_edt(~blocked) * res
    unknown = np.zeros((h, w), dtype=bool)
    return (dist, ox, oy, res, w, h, unknown)


def _horiz_path(w, row, res, ox, oy, n=40):
    """在 costmap 格 row 上取一条水平直线路径（格索引 list）。"""
    cols = np.linspace(4, w - 5, n).astype(int)
    return [row * w + int(c) for c in cols]


def main():
    fails = []
    res = 0.05
    ox = oy = 0.0
    shim = _load()
    obj = shim()
    obj.corridor_hard = 0.92

    def check(name, cond, detail=''):
        if cond:
            print(f'  PASS  {name}')
        else:
            print(f'  FAIL  {name}  {detail}')
            fails.append(name)

    # ---- A 对称走廊：两侧都有墙，间距 2.0 m，路径走中线 ----
    print('A 对称走廊（间距 2.0 m，路径居中）')
    w, h = 100, 60
    # 第 5 行与第 45 行是墙 ⇒ 中线在 (5+45)/2 = 25 行
    field = _make_field(w, h, res, ox, oy, blocked_rows=(5, 45))
    path = _horiz_path(w, 25, res, ox, oy)
    lat = obj._path_lateral_widths(field, path, list(range(len(path))),
                                   res, ox, oy, w, h)
    check('返回值非 None', lat is not None)
    if lat:
        widths = lat[0]
        old = [2.0 * field[0][p // w, p % w] for p in path]
        new_med = float(np.median(widths))
        old_med = float(np.median(old))
        check('新值 ≈ 旧值（对称场景不放大）',
              abs(new_med - old_med) <= 0.12,
              f'new={new_med:.2f} old={old_med:.2f}')
        check('对称场景实测宽度 ≈ 2.0 m',
              abs(new_med - 2.0) <= 0.15, f'new={new_med:.2f}')
        print(f'        中位：新 {new_med:.2f} m / 旧 {old_med:.2f} m')

    # ---- B 单侧障碍、另一侧开阔：要修的正是这个 ----
    print('B 单侧墙 + 另一侧开阔（造出"旧值被 corridor_hard 误杀"的几何）')
    w, h = 100, 100
    # 第 0 行（y=0.025 m）是墙，路径走第 9 行（y=0.475）
    #   → 到最近障碍 0.45 m ⇒ 旧值 0.90 < corridor_hard 0.92 ⇒ **旧实现判死**
    #   → 新值 0.45 + 0.92(截断) ≈ 1.37 ⇒ 通过
    field = _make_field(w, h, res, ox, oy, blocked_rows=(0,))
    path = _horiz_path(w, 9, res, ox, oy)
    lat = obj._path_lateral_widths(field, path, list(range(len(path))),
                                   res, ox, oy, w, h)
    check('返回值非 None', lat is not None)
    if lat:
        widths = lat[0]
        old = [2.0 * field[0][p // w, p % w] for p in path]
        new_med = float(np.median(widths))
        old_med = float(np.median(old))
        print(f'        中位：新 {new_med:.2f} m / 旧 {old_med:.2f} m'
              f'（旧 {old_med:.2f} < 0.92 ⇒ 旧实现判死）')
        check('旧值确实会被误杀（< corridor_hard）', old_med < 0.92,
              f'old={old_med:.2f}')
        check('新值显著大于旧值', new_med >= old_med + 0.30,
              f'new={new_med:.2f} old={old_med:.2f}')
        check('新值足以越过 corridor_hard', new_med >= 0.92,
              f'new={new_med:.2f}')

    # ---- C 安全约束：新值永远 >= 旧值 ----
    print('C 安全约束（随机障碍，逐点断言 新值 >= 旧值）')
    rng = np.random.default_rng(20261111)
    worst = None
    for trial in range(6):
        w, h = 120, 120
        rows = tuple(rng.choice(np.arange(3, 117), size=6, replace=False).tolist())
        cols = tuple(rng.choice(np.arange(3, 117), size=6, replace=False).tolist())
        field = _make_field(w, h, res, ox, oy, blocked_rows=rows, blocked_cols=cols)
        # 挑一行明显不是墙的位置当路径
        row = int(rng.integers(3, 117))
        path = _horiz_path(w, row, res, ox, oy)
        lat = obj._path_lateral_widths(field, path, list(range(len(path))),
                                       res, ox, oy, w, h)
        if lat is None:
            continue
        for k, p in enumerate(path):
            old = 2.0 * field[0][p // w, p % w]
            gap = lat[0][k] - old
            if worst is None or gap < worst:
                worst = gap
    check('新值始终 >= 旧值（含 0.02 m 浮点余量）',
          worst is not None and worst >= -0.02,
          f'最小差值={worst:.4f} m')
    print(f'        全部采样点中 (新-旧) 最小差值 = {worst:.4f} m')

    print()
    if fails:
        print(f'FAILED {len(fails)} 项：{fails}')
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
