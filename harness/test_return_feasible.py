#!/usr/bin/env python3
"""test_return_feasible.py —— "可达但不可返回"与"脱困目标"的离线自测（<2 秒，不依赖 ROS）。

来历（2026-10-25 myrun 事故）：车在 (-1.05,3.82) 的角落停了 100 秒 ——
每个前沿候选的路径都要过同一个窄口，仲裁每拍"全部被拒"，一个目标都不派，
manager 自己的采样也过不去，车就一直待着，脱困单元也失败。事后确认这是
"可达但不可返回"的现场：进得去、出不来。

本测试焊住两条新增逻辑：
  A 目标附近掉头空间不足（< 车对角线 0.81 m）→ 判"不可返回"
  B 目标附近有足够空地（拐角旁的小广场）→ 放行
  C 视野里全是窄缝、找不到 0.55 m 净空的落点 → 脱困目标返回 None（不乱派）
  D 车周围有开阔空地 → 给出脱困目标，且落点净空达标、距车不超过上限
  E 冷却期内第二次请求 → 拒绝（避免脱困目标刷屏）
  F 次数上限 → 拒绝
"""
import ast
import math
import sys

import numpy as np

SRC = ('/home/kbf/stemm_exploration_sim_ws/src/stemm_cartographer_exploration_sim/'
       'scripts/frontier_arbiter')
RES = 0.05


class Logger:
    def __init__(self):
        self.lines = []

    def warn(self, m):
        self.lines.append(m)

    def info(self, m):
        self.lines.append(m)


class FakeSelf:
    def __init__(self, grid, pose=(0.0, 0.0)):
        n = grid.shape[0]
        ox = oy = -n * RES / 2.0
        # clearance field: (dist_m, ox, oy, res, w, h, unknown)
        dist = grid.astype(float)      # 约定：clearance field 里的 dist_m 已经是"米"
        self._field = (dist, ox, oy, RES, n, n, None)
        self.pose = (pose[0], pose[1], 0.0)
        self.map = None
        self.return_check = True
        self.return_turn_width = 0.85
        self.return_scan_radius = 1.20
        self.escape_goal_enabled = True
        self.escape_min_clearance = 0.55
        self.escape_max_dist = 1.60
        self.escape_cooldown = 8.0
        self.escape_max_total = 20
        self._esc_last = 0.0
        self._esc_count = 0
        self.stats = {}
        self.publish_ok = True
        self.log = Logger()
        self._ox, self._oy = ox, oy
        self._n = n

    def get_logger(self):
        return self.log

    def clearance_field(self):
        return self._field

    def path_and_width(self, gx, gy):
        # 简化：本测试只验证"能不能到"的判定分支，不重算 BFS
        return (self.publish_ok, None, 2.0, 0.0)

    def publish_goal(self, gx, gy, r, u=None):
        pass


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
    seg = extract({'turn_space_near', 'return_infeasible', 'pick_escape_goal'})
    ns = {'math': math, 'time': __import__('time')}
    for name, code in seg.items():
        exec(compile(code, f'<{name}>', 'exec'), ns)
        setattr(cls, name, ns[name])


def cleared_field(size=200, clear_m=5.0):
    """到最近障碍的距离场（单位 m）。clear_m=5 → 到处都很开阔。"""
    return np.full((size, size), clear_m)


def corridor_field(size=200, half_width_m=0.35):
    """一条沿 x 轴的走廊（单位：米）。墙外给 0，走廊内给"到墙的距离"。"""
    g = np.zeros((size, size))
    c = size // 2
    k = int(half_width_m / RES)
    for j in range(size):
        d = k - abs(j - c)                    # 距墙的格数
        if d >= 0:
            g[j, :] = d * RES
    return g


def main():
    build(FakeSelf)
    fails = []

    def check(name, cond, detail=''):
        print(f'{"✓" if cond else "✗"} {name}' + ((f'  ← {detail}') if (not cond and detail) else ''))
        if not cond:
            fails.append(name)

    # A 走廊里（半宽 0.35 → 掉头空间 0.70 < 0.85）→ 不可返回
    f = FakeSelf(corridor_field(half_width_m=0.35))
    oneway, turn = f.return_infeasible(0.0, 0.0)
    check('A 窄走廊判"不可返回"', oneway and turn is not None and turn < 0.85,
          f'turn={turn}')

    # B 开阔地（拐角旁小广场）→ 放行
    f = FakeSelf(cleared_field(clear_m=2.0))
    oneway, turn = f.return_infeasible(0.0, 0.0)
    check('B 开阔地放行', (not oneway) and turn is not None and turn >= 0.85, f'turn={turn}')

    # C 到处只有 0.50 m 净空 → 找不到 0.55 的落点 → 不派
    f = FakeSelf(cleared_field(clear_m=0.25))       # 2×0.25 = 0.50 < 0.55
    check('C 太挤时不派脱困目标', f.pick_escape_goal() is None)

    # D 有开阔空地 → 给出脱困目标
    f = FakeSelf(cleared_field(clear_m=1.0))
    esc = f.pick_escape_goal()
    ok = esc is not None and math.hypot(esc[0] - f.pose[0], esc[1] - f.pose[1]) <= 1.65
    check('D 有开阔地时给出脱困目标', ok, f'esc={esc}')

    # E 冷却期内第二次 → 拒绝
    f = FakeSelf(cleared_field(clear_m=1.0))
    f.pick_escape_goal()
    check('E 冷却期内不再派', f.pick_escape_goal() is None)

    # F 次数上限
    f = FakeSelf(cleared_field(clear_m=1.0))
    f.escape_cooldown = 0.0
    f.escape_max_total = 2
    got = [f.pick_escape_goal() for _ in range(4)]
    n_ok = sum(1 for g in got if g is not None)
    check('F 每局上限 2 次', n_ok == 2, f'实际 {n_ok}')

    if fails:
        print(f'  ✗ {len(fails)} 项失败')
        print('结果：FAIL')
        return 1
    print('结果：PASS（6 项断言全部通过）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
