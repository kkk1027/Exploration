#!/usr/bin/env python3
"""test_fallback_obs.py —— V2-② 保底观察点的功能自测（<2 秒，不依赖 ROS）。

来历：2026-10-24 加"没候选就派开阔观察点"这条兜底路径。它只在
"所有候选都被拒"时才跑，而那种状态在真机里难复现，一旦写错（比如
scored 的元组解包写错、净空场为空）就会在卡死那一刻崩，等于把兜底变
成新的崩溃点。这里用合成净空场把它跑通：

  case A 空旷场地  → 必须派出点，且落在开阔处
  case B 全线贴墙（净空全场 <0.30）→ 必须返回 None（宁可不派）
  case C 无净空场   → 必须返回 None
  case D 冷却/次数上限 → 超限后必须返回 None
"""
import ast
import math
import sys

import numpy as np

SRC = ('/home/kbf/stemm_exploration_sim_ws/src/stemm_cartographer_exploration_sim/'
       'scripts/frontier_arbiter')
RES = 0.05


def extract(names):
    """把指定方法从仲裁器源码里抠出来（不 import ROS）。"""
    src = open(SRC, encoding='utf-8').read()
    tree = ast.parse(src)
    out = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name in names:
            out[node.name] = ast.get_source_segment(src, node)
    miss = set(names) - set(out)
    assert not miss, f'源码里找不到方法：{miss}'
    return out


class FakeSelf:
    """只实现 pick_safe_observation / region_complexity 依赖的最小状态。"""

    def __init__(self, field):
        self._field = field
        self.pose = (0.0, 0.0, 0.0)
        self.probe_radius = 2.0
        self.probe_narrow_r = 0.35
        self.probe_escape_r = 0.6
        self.max_cand = 8
        self.fallback_enabled = True
        self.fallback_min_esc = 5
        self.fallback_min_comp = 1000
        self.fallback_max_dist = 3.0
        self.fallback_cooldown = 10.0
        self.fallback_max_total = 30
        self._fb_last = 0.0
        self._fb_count = 0
        # 末段偏置（本测试固定为"不在末段窗口"，窗口行为由 test_tail_park.py 覆盖）
        self.tail_min_esc = 5
        self.tail_penalty_value = 900.0

    def clearance_field(self):
        return self._field

    def vision_occ_mask(self):
        return None

    def pull_back(self, x, y):
        return (x, y)

    def in_tail_window(self):
        return False


def build_methods(cls):
    seg = extract({'pick_safe_observation', 'region_complexity'})
    ns = {'math': math, 'time': __import__('time')}
    for name, code in seg.items():
        exec(compile(code, f'<{name}>', 'exec'), ns)
        setattr(cls, name, ns[name])


def make_field(kind, size=200):
    """合成净空场：(dist_m, ox, oy, res, w, h, unknown)。原点在场中心。"""
    ox = oy = -size * RES / 2.0
    if kind == 'open':
        d = np.full((size, size), 5.0, dtype=float)
    elif kind == 'wall':
        d = np.full((size, size), 0.10, dtype=float)          # 全场贴墙
    else:
        return None
    return (d, ox, oy, RES, size, size, None)


def main():
    FakeSelf.pick_safe_observation = None
    build_methods(FakeSelf)
    fails = []

    # case A：空旷场地，候选在 2 m 处 → 应派出点，且离车 0.8~3.0 m
    f = FakeSelf(make_field('open'))
    obs = f.pick_safe_observation([(2.0, 0.0), (1.5, 1.0), (2.5, -1.0)])
    if obs is None:
        fails.append('A 空旷场地竟然没派出点')
    else:
        dist = math.hypot(obs[0] - f.pose[0], obs[1] - f.pose[1])
        print(f'A 空旷场地 → 派点 ({obs[0]:.2f},{obs[1]:.2f})，距车 {dist:.2f} m，计数 {f._fb_count}')
        if not (0.8 <= dist <= 3.0):
            fails.append(f'A 派点距离 {dist:.2f} m 越界')
        if f._fb_count != 1:
            fails.append(f'A 计数应为 1，实际 {f._fb_count}')

    # case A2：冷却期内再来一次 → 必须拒绝
    obs2 = f.pick_safe_observation([(2.0, 0.0)])
    print(f'A2 冷却期内 → {"拒绝（正确）" if obs2 is None else f"仍派出 {obs2}（错误）"}')
    if obs2 is not None:
        fails.append('A2 冷却没生效')

    # case B：全场贴墙 → 必须返回 None
    f = FakeSelf(make_field('wall'))
    obs = f.pick_safe_observation([(2.0, 0.0), (1.5, 1.0)])
    print(f'B 全场贴墙 → {"拒绝（正确）" if obs is None else f"派出 {obs}（错误）"}')
    if obs is not None:
        fails.append('B 贴墙场景仍派点')

    # case C：净空场缺失 → 必须返回 None
    f = FakeSelf(None)
    obs = f.pick_safe_observation([(2.0, 0.0)])
    print(f'C 无净空场 → {"拒绝（正确）" if obs is None else f"派出 {obs}（错误）"}')
    if obs is not None:
        fails.append('C 无净空场仍派点')

    # case D：次数上限
    f = FakeSelf(make_field('open'))
    f.fallback_max_total = 2
    f.fallback_cooldown = 0.0                              # 关冷却，只测上限
    got = [f.pick_safe_observation([(2.0, 0.0)]) for _ in range(5)]
    n_ok = sum(1 for g in got if g is not None)
    print(f'D 上限 2 次 → 连派 5 次成功 {n_ok} 次（期望 2）')
    if n_ok != 2:
        fails.append(f'D 次数上限应为 2，实际 {n_ok}')

    # case E：太远 / 太近的候选都应被跳过
    f = FakeSelf(make_field('open'))
    obs = f.pick_safe_observation([(9.0, 0.0), (0.2, 0.0)])
    print(f'E 只有过远/过近候选 → {"拒绝（正确）" if obs is None else f"派出 {obs}（错误）"}')
    if obs is not None:
        fails.append('E 距离筛选失效')

    if fails:
        for x in fails:
            print(f'  ✗ {x}')
        print('结果：FAIL')
        return 1
    print('结果：PASS（5 组、7 项断言全部通过）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
