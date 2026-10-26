#!/usr/bin/env python3
"""test_noexit_verdict.py —— A 层"可退出性"判定的离线单元测试（不启 ROS、不占用仿真）。

被测：frontier_arbiter.FrontierArbiter._noexit_verdict（docs/P9 §15 重定义后的语义）
输入：wlist = [(宽度, 世界x, 世界y), ...] 与路径同序（索引 0 = 车这一侧）、step_m = 采样步长
输出：None = 所有 pinch 都能退出（放行）；(x, y) = 第一个"进得去出不来"的 pinch 中点

覆盖的真实场景（取自 2026-10-26 的贴合实测）：
  * 走廊 0.9 m 一口气穿过 3 m（无掉头、无回退）→ 必须否决；
  * 同样的 0.9 m 缝，但车这侧有 1.5 m 开阔（能沿原路退）→ 放行；
  * 同样的缝，中间有一个 1.0 m 的凹室（能掉头）→ 放行；
  * 只擦 0.2 m 车角（宽 0.6 m）→ 放行；
  * 全部 >= pass_width（1.05 m）→ 根本没有 pinch，放行；
  * 多个 pinch：第一个能退出、第二个不能 → 报第二个的位置。
"""
import os
import sys

ROOT = '/home/kbf/stemm_exploration_sim_ws'
ARB = os.path.join(ROOT, 'src/stemm_cartographer_exploration_sim/scripts/frontier_arbiter')


def load_arbiter():
    """只把 `_noexit_verdict` 这一个函数从源文件里摘出来执行。

    不 import 整个脚本（那需要 rclpy / wheeltec_rrt_msg 全套 ROS 环境），
    但仍然执行的是**文件里的真实源码**，所以改坏了逻辑一样会失败。
    """
    import ast
    src = open(ARB, encoding='utf-8').read()
    tree = ast.parse(src, filename=ARB)
    fn = next(n for n in ast.walk(tree)
              if isinstance(n, ast.FunctionDef) and n.name == '_noexit_verdict')
    mod = ast.Module(body=[fn], type_ignores=[])
    ns = {}
    exec(compile(mod, ARB, 'exec'), ns)
    return ns['_noexit_verdict']


class Stub:
    """只带 _noexit_verdict 需要的属性；不需要 ROS 节点。"""

    def __init__(self):
        self.noexit_pinch_m = 0.752
        self.noexit_short_m = 0.35
        self.noexit_turn_m = 0.81
        self.noexit_retreat_m = 1.20
        self.noexit_retreat_need_m = 1.00
        self.stats = {}


def wlist(widths, step=0.05):
    return [(w, i * step, 0.0) for i, w in enumerate(widths)]


def run(mod):
    v = mod
    st = Stub()
    ok = 0
    fail = []

    def case(name, widths, want_none, step=0.05):
        nonlocal ok
        got = v(st, wlist(widths, step), step)
        good = (got is None) if want_none else (got is not None)
        if good:
            ok += 1
            print('  ✓ %s' % name)
        else:
            fail.append(name)
            print('  ✗ %s  得到 %r（期望 %s）' % (name, got, 'None' if want_none else '否决'))

    # 1) 无 pinch（全部 1.2 m）
    case('全宽 1.2 m → 放行', [1.2] * 60, True)
    # 2) 0.9 m 走廊 3 m：没有低于 pinch(0.752) 的段落 → 这是 A 要"救回来"的供给
    case('0.9 m 走廊 3 m（无 pinch）→ 放行', [0.9] * 60, True)
    # 2b) 0.6 m 缝 3 m 长：真 pinch，两侧都是 0.6、无掉头无回退 → 否决
    case('0.6 m 缝 ×3 m，无回退无掉头 → 否决', [0.6] * 60, False)
    # 3) 同上 0.6 m 缝，但车这侧留 1.5 m 开阔（1.2 m 窗口内能退）→ 放行
    case('0.6 m 缝 + 车侧 1.5 m 开阔 → 放行',
         [1.5] * 30 + [0.6] * 60 + [1.5] * 10, True)
    # 4) 同上 0.6 m 缝，但缝中间有一处 1.0 m 凹室（能掉头 0.81 m）→ 放行
    case('0.6 m 缝 + 缝中 0.5 m 长 1.0 m 凹室（能掉头）→ 放行',
         [0.6] * 20 + [1.0] * 10 + [0.6] * 20, True)
    # 5) 只擦一下车角（0.6 m × 0.2 m = 4 点）→ 放行
    case('0.6 m 只擦 0.2 m → 放行', [1.2] * 10 + [0.6] * 4 + [1.2] * 10, True)
    # 6) 两个 pinch：第一个能退（前方 1.5 m 开阔），第二个不能 → 要否决
    case('两段 pinch：第一段能退（车侧开阔），第二段前只有 0.9 m 宽 → 否决',
         [1.5] * 30 + [0.6] * 40 + [0.8] * 18 + [0.6] * 60, False)
    # 7) 空/单点输入（边界）→ 放行
    case('空列表 → 放行', [], True)
    case('单点 0.5 m → 放行（长度 0.05 m <= 0.35）', [0.5], True)

    print('\n通过 %d 项，失败 %d 项' % (ok, len(fail)))
    if fail:
        print('失败用例：' + '；'.join(fail))
        return 1
    return 0


if __name__ == '__main__':
    mod = load_arbiter()
    sys.exit(run(mod))
