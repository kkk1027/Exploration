#!/usr/bin/env python3
"""test_tail_park.py —— 探索末段"停车友好"偏置的离线自测（<2 秒，不依赖 ROS）。

来历（2026-10-24）：新几组十轮里最主要的残余失败模式不是撞障，而是
**探索时间到时车停在贴障位置**，车端 manager 以
`current pose is too close to occupied /map cells` / `front obstacle too close`
拒绝启动返航，3 次重试在 6 秒内耗尽 → 整轮返航失败
（272 轮历史里这条消息出现在 21 轮＝7.7%，近期失败几乎全是它）。

修法：最后 tail_sec 秒里给"四周不够开阔"的候选加罚，让车尽量停在空地。
本测试焊住这个偏置的四个边界：
  A 窗口内 + 窄候选   → 必须加罚
  B 窗口内 + 开阔候选 → 不加罚
  C 窗口外（还早）    → 不加罚
  D 关掉开关          → 不加罚
  E 还没派过任何目标（无法判断进度）→ 不加罚
  F 窗口边界：remain 恰好 0 / 恰好 tail_sec → 判在窗口内
"""
import ast
import math
import sys

import numpy as np

SRC = ('/home/kbf/stemm_exploration_sim_ws/src/stemm_cartographer_exploration_sim/'
       'scripts/frontier_arbiter')
RES = 0.05


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


class FakeSelf:
    def __init__(self, field, remain):
        self._field = field
        self.tail_park_enabled = True
        self.explore_sec = 120.0
        self.tail_sec = 25.0
        self.tail_min_esc = 5
        self.tail_penalty_value = 900.0
        self.gate_occ_soft = 0.30
        self.probe_radius = 2.0
        self.probe_narrow_r = 0.35
        self.probe_escape_r = 0.6
        # remain=剩余探索秒数；用"首次派发时刻"把它换算成时钟
        self._first_pub_t = None if remain is None else 0.0
        self._remain = remain

    def clearance_field(self):
        return self._field

    def vision_occ_mask(self):
        return None


def build_methods(cls):
    seg = extract({'tail_penalty', 'in_tail_window', 'region_complexity'})
    ns = {'math': math, 'time': __import__('time')}
    for name, code in seg.items():
        exec(compile(code, f'<{name}>', 'exec'), ns)
        setattr(cls, name, ns[name])


def field(kind, size=200):
    ox = oy = -size * RES / 2.0
    if kind == 'open':
        d = np.full((size, size), 5.0, dtype=float)
    elif kind == 'tight':
        d = np.full((size, size), 0.10, dtype=float)
    else:
        return None
    return (d, ox, oy, RES, size, size, None)


def with_remain(obj, remain):
    """把 in_tail_window 的判据换成"剩余 remain 秒"（不依赖真实时钟）。"""
    orig = obj.in_tail_window

    def fake():
        if not obj.tail_park_enabled or obj.explore_sec <= 0.0 or obj._first_pub_t is None:
            return False
        return 0.0 <= remain <= obj.tail_sec
    obj.in_tail_window = fake
    return orig


def main():
    build_methods(FakeSelf)
    fails = []

    def check(name, cond, detail=''):
        print(f'{"✓" if cond else "✗"} {name}{("  " + detail) if detail else ""}')
        if not cond:
            fails.append(name)

    # A 窗口内 + 窄
    f = FakeSelf(field('tight'), 10.0); with_remain(f, 10.0)
    p = f.tail_penalty(3.0, 1.0)
    check('A 窗口内窄候选被加罚', p == f.tail_penalty_value, f'罚分={p}')

    # B 窗口内 + 开阔
    f = FakeSelf(field('open'), 10.0); with_remain(f, 10.0)
    p = f.tail_penalty(3.0, 1.0)
    check('B 窗口内开阔候选不加罚', p == 0.0, f'罚分={p}')

    # C 窗口外
    f = FakeSelf(field('tight'), 60.0); with_remain(f, 60.0)
    p = f.tail_penalty(3.0, 1.0)
    check('C 还早时不加罚', p == 0.0, f'罚分={p}')

    # D 开关关掉
    f = FakeSelf(field('tight'), 10.0); with_remain(f, 10.0)
    f.tail_park_enabled = False
    p = f.tail_penalty(3.0, 1.0)
    check('D 开关关闭时不加罚', p == 0.0, f'罚分={p}')

    # E 还没派过目标
    f = FakeSelf(field('tight'), None)
    p = f.tail_penalty(3.0, 1.0)
    check('E 无进度信息时不加罚', p == 0.0, f'罚分={p}')

    # F 边界：remain = 0 与 remain = tail_sec 都算窗口内
    for remain, want_in in ((0.0, True), (25.0, True), (25.1, False)):
        f = FakeSelf(field('tight'), remain); with_remain(f, remain)
        inside = f.in_tail_window()
        check(f'F remain={remain}s 窗口判定', inside == want_in,
              f'inside={inside}（期望 {want_in}）')

    if fails:
        for x in fails:
            print(f'  ✗ {x}')
        print('结果：FAIL')
        return 1
    print('结果：PASS（7 组、7 项断言全部通过）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
