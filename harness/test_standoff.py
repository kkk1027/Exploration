#!/usr/bin/env python3
"""test_standoff.py —— 返航前"挪一步"兜底的离线自测（<2 秒，不依赖 ROS）。

来历（2026-10-24）：最主要的残余失败模式是**探索时间到时车停在贴障位置**，
车端 manager 以 `current pose is too close to occupied /map cells` /
`front obstacle too close` 拒绝派返航，3 次重试 6 秒耗尽（272 轮历史里 21 轮命中）。

修法：结束建图**之前**判断"是否停得太贴障"，是就先派一个 ≤0.6 m 的接管点让车挪开。
这条路径会在每轮结尾自动跑，判错就会把车往墙里送，所以必须离线焊住：

  A 空旷处（净空足）        → 不挪（返回 None）
  B 贴墙 + 身后有空         → 必须给出接管点，且落点在自由格、离车 ≤0.6 m
  C 四面皆堵（1 m 内无路）  → 不挪（返回 None）
  D 无地图                  → 不挪
  E 已经到了次数上限        → 不挪
  F 开关关闭                → 不挪
  G 接管点自身净空必须够（不能把车送到另一个死角）
"""
import ast
import math
import sys

import numpy as np

SRC = ('/home/kbf/stemm_exploration_sim_ws/src/stemm_cartographer_exploration_sim/'
       'scripts/session_driver')
RES = 0.05


class Logger:
    def __init__(self):
        self.lines = []

    def warn(self, m):
        self.lines.append(m)

    def info(self, m):
        self.lines.append(m)


class Info:
    def __init__(self, res, w, h, ox, oy):
        self.resolution = res
        self.width = w
        self.height = h

        class P:
            pass
        p = P()
        p.x, p.y, p.z = ox, oy, 0.0
        self.origin = type('O', (), {'position': p})()


class MapMsg:
    def __init__(self, grid, res, ox, oy):
        self.data = grid.astype(np.int8).flatten().tolist()
        self.info = Info(res, grid.shape[1], grid.shape[0], ox, oy)


class FakeSelf:
    def __init__(self, grid=None, pose=(0.0, 0.0)):
        self.map_msg = None
        if grid is not None:
            n = grid.shape[0]
            self.map_msg = MapMsg(grid, RES, -n * RES / 2.0, -n * RES / 2.0)
        self.pose_xy = pose
        self.standoff_enabled = True
        self.standoff_trigger = 0.45
        self.standoff_goal = 0.60
        self.standoff_max = 2
        self._standoff_count = 0
        self.log = Logger()

    def get_logger(self):
        return self.log


def extract(names):
    src = open(SRC, encoding='utf-8').read()
    tree = ast.parse(src)
    out = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name in names:
            out[node.name] = ast.get_source_segment(src, node)
    miss = set(names) - set(out)
    assert not miss, f'源码里找不到方法：{miss}'
    return out


def build(cls):
    seg = extract({'map_clearance', 'pick_standoff', '_pick_standoff_if_tight',
                   '_grid_params', '_cell'})
    ns = {'math': math}
    for name, code in seg.items():
        exec(compile(code, f'<{name}>', 'exec'), ns)
        setattr(cls, name, ns[name])


def grid_open(size=200):
    return np.zeros((size, size), dtype=np.int8)


def grid_wall_behind(size=200):
    """车在中心，正前方（+x）半米处是墙，其余开阔。"""
    g = grid_open(size)
    c = size // 2
    for j in range(size):
        for i in range(c + 5, c + 8):         # 约 +0.25~0.35 m 处一堵竖墙
            g[j, i] = 100
    return g


def grid_boxed(size=200):
    """车被围在 0.3 m 的小格子里（四面皆堵）。"""
    g = grid_open(size)
    c = size // 2
    r = 6
    g[c - r:c + r + 1, c - r] = 100
    g[c - r:c + r + 1, c + r] = 100
    g[c - r, c - r:c + r + 1] = 100
    g[c + r, c - r:c + r + 1] = 100
    return g


def main():
    build(FakeSelf)
    fails = []

    def check(name, cond, detail=''):
        print(f'{"✓" if cond else "✗"} {name}{("  " + detail) if detail else ""}')
        if not cond:
            fails.append(name)

    def free_at(f, x, y):
        g = f._grid_params()
        _d, res, w, h, ox, oy = g
        return 0 <= f._cell(g, int((x - ox) / res), int((y - oy) / res)) < 50

    # A 空旷处：净空远大于阈值 → 不该挪
    f = FakeSelf(grid_open())
    clr = f.map_clearance(0.0, 0.0)
    goal = f._pick_standoff_if_tight()
    check('A 空旷处不挪', goal is None, f'净空={clr}（应≈扫描半径 1.2）目标={goal}')

    # B 贴墙：正前方 0.4 m 有墙（净空 < 0.35）→ 必须给出接管点
    f = FakeSelf(grid_wall_behind())
    clr = f.map_clearance(0.0, 0.0)
    goal = f._pick_standoff_if_tight()
    ok = (goal is not None and free_at(f, goal[0], goal[1])
          and math.hypot(goal[0], goal[1]) <= 0.62)
    check('B 贴墙时给出合法接管点', ok,
          f'净空={clr} 目标={None if goal is None else (round(goal[0],2), round(goal[1],2))}')

    # G 接管点自身净空要够（不能把车送进另一个死角）
    if goal is not None:
        gclr = f.map_clearance(goal[0], goal[1], scan_m=0.45)
        check('G 接管点本身净空足够', gclr is not None and gclr >= 0.30, f'接管点净空={gclr}')

    # C 四面皆堵：1 m 内无路 → 不该挪
    f = FakeSelf(grid_boxed(), pose=(0.0, 0.0))
    goal = f._pick_standoff_if_tight()
    check('C 四面皆堵不挪', goal is None, f'目标={goal}')

    # D 无地图
    f = FakeSelf(None)
    check('D 无地图不挪', f._pick_standoff_if_tight() is None)

    # E 次数上限
    f = FakeSelf(grid_wall_behind())
    f._standoff_count = f.standoff_max
    check('E 达到上限不再挪', f._pick_standoff_if_tight() is None)

    # F 开关关闭
    f = FakeSelf(grid_wall_behind())
    f.standoff_enabled = False
    check('F 开关关闭不挪', f._pick_standoff_if_tight() is None)

    # H 净空恰好等于阈值 → 不挪（边界）
    f = FakeSelf(grid_wall_behind())
    f.standoff_trigger = round(f.map_clearance(0.0, 0.0), 3)
    check('H 净空==阈值不挪（边界）', f._pick_standoff_if_tight() is None,
          f'阈值={f.standoff_trigger}')

    if fails:
        for x in fails:
            print(f'  ✗ {x}')
        print('结果：FAIL')
        return 1
    print('结果：PASS（8 项断言全部通过）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
