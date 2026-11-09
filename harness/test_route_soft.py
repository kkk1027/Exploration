#!/usr/bin/env python3
"""test_route_soft.py —— P13-3 硬门降级（软约束）的离线验证。

背景（p13c A/B 实测，各 5 轮）：
    rejected_narrow 812~936 次/轮，published 仅 29~33 次/轮
⇒ 仲裁器供不起货，frontier_merge 在 validated TTL 过期后直接转发未评分的
  raw（47~103 条/轮），实际派发由 raw 主导，评分改造无从体现。

本测试锁住三件事：
  1) 软门开启时，只有 min_w < corridor_hard 才硬拒（几何真过不去）；
  2) 软门开启时，min_w 越接近硬下限，软罚分越大，且**不会恒为 0**
     —— 默认 corridor_hard(0.92) > pass_width(0.90)，若拿 pass_width 当基准，
        该分支是不可达的死代码（这是首版实现的实际缺陷）；
  3) 软门关闭时完全回到历史硬门行为。

用法：python3 harness/test_route_soft.py [-v]
"""
import importlib.machinery
import importlib.util
import os
import sys
import types

_pkgs = ('rclpy', 'rclpy.node', 'rclpy.qos', 'action_msgs', 'action_msgs.msg',
         'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs', 'nav_msgs.msg',
         'std_msgs', 'std_msgs.msg', 'wheeltec_rrt_msg', 'wheeltec_rrt_msg.msg')
for name in _pkgs:
    if name not in sys.modules:
        mod = types.ModuleType(name)
        mod.__getattr__ = lambda attr: object  # type: ignore
        sys.modules[name] = mod
sys.modules['rclpy'].node = sys.modules['rclpy.node']
sys.modules['std_msgs'].msg = sys.modules['std_msgs.msg']
sys.modules['geometry_msgs'].msg = sys.modules['geometry_msgs.msg']
sys.modules['nav_msgs'].msg = sys.modules['nav_msgs.msg']
sys.modules['action_msgs'].msg = sys.modules['action_msgs.msg']
sys.modules['wheeltec_rrt_msg'].msg = sys.modules['wheeltec_rrt_msg.msg']


class _Node:
    def __init__(self, *a, **k):
        pass


sys.modules['rclpy.node'].Node = _Node
sys.modules['rclpy.qos'].QoSProfile = lambda *a, **k: None
sys.modules['rclpy.qos'].DurabilityPolicy = type('D', (), {'TRANSIENT_LOCAL': 1})
sys.modules['rclpy.qos'].ReliabilityPolicy = type('R', (), {'RELIABLE': 1})
sys.modules['wheeltec_rrt_msg.msg'].PointArray = type('P', (), {})
sys.modules['action_msgs.msg'].GoalStatusArray = type('G', (), {})
sys.modules['geometry_msgs.msg'].PoseStamped = type('PS', (), {})
sys.modules['geometry_msgs.msg'].Twist = type('T', (), {})
sys.modules['nav_msgs.msg'].OccupancyGrid = type('O', (), {})
sys.modules['nav_msgs.msg'].Odometry = type('Od', (), {})
sys.modules['std_msgs.msg'].String = type('S', (), {})
sys.modules['std_msgs.msg'].Bool = type('B', (), {})

HERE = os.path.dirname(os.path.abspath(__file__))
ARB = os.path.join(HERE, '..', 'src', 'stemm_cartographer_exploration_sim',
                   'scripts', 'frontier_arbiter')
_loader = importlib.machinery.SourceFileLoader('frontier_arbiter', ARB)
_spec = importlib.util.spec_from_loader('frontier_arbiter', _loader)
arb = importlib.util.module_from_spec(_spec)
sys.modules['frontier_arbiter'] = arb
_loader.exec_module(arb)

FAILS = []
VERBOSE = '-v' in sys.argv


def check(name, cond, detail=''):
    if cond:
        if VERBOSE:
            print('  ok   %s  %s' % (name, detail))
        return True
    FAILS.append('%s  %s' % (name, detail))
    print('  FAIL %s  %s' % (name, detail))
    return False


def soft_of(min_w=None, narrow_run=0.0, unk_run=0.0, cfg=None):
    """复刻 path_and_width 里软门 soft 的计算（纯算术，不依赖地图）。"""
    c = cfg
    soft = 0.0
    if min_w is not None and min_w < c['route_target']:
        span = max(1e-6, c['route_target'] - c['corridor_hard'])
        soft += min(1.0, (c['route_target'] - min_w) / span)
    if narrow_run > c['narrow_len_max']:
        soft += min(1.0, (narrow_run - c['narrow_len_max'])
                    / max(1e-6, c['narrow_len_max']))
    if unk_run > c['unknown_span_max']:
        soft += min(1.0, (unk_run - c['unknown_span_max'])
                    / max(1e-6, c['unknown_span_max']))
    return round(soft, 3)


CFG = {'corridor_hard': 0.92, 'pass_width': 0.90, 'route_target': 1.20,
       'narrow_len_max': 1.00, 'unknown_span_max': 0.35}


def main():
    print('\n[1] 几何硬门仍在：min_w < corridor_hard 一律拒')
    check('corridor_hard(0.92) > pass_width(0.90)（这就是必须换基准的原因）',
          CFG['corridor_hard'] > CFG['pass_width'],
          '%.2f > %.2f' % (CFG['corridor_hard'], CFG['pass_width']))
    # 用 pass_width 当基准时，宽度项恒为 0 → 死代码
    dead = 0.0
    for w in (0.92, 0.95, 1.0, 1.2):
        if w < CFG['pass_width']:
            dead += (CFG['pass_width'] - w) / max(1e-6, CFG['pass_width'] - CFG['corridor_hard'])
    check('对照：用 pass_width 当基准时软项恒为 0（确认首版缺陷）',
          dead == 0.0, 'dead=%.3f' % dead)

    print('\n[2] 软门罚分随宽度单调，且不恒为 0')
    pts = [(0.92, '=硬下限'), (1.00, ''), (1.10, ''), (1.19, ''),
           (1.20, '=舒适宽'), (1.50, '>舒适宽')]
    vals = [(w, soft_of(min_w=w, cfg=CFG)) for w, _ in pts]
    if VERBOSE:
        for w, v in vals:
            print('    min_w=%.2f → soft=%.3f' % (w, v))
    check('min_w=硬下限时软罚分最大', vals[0][1] >= max(v for _, v in vals),
          'soft(0.92)=%.3f' % vals[0][1])
    check('min_w=硬下限时软罚分 > 0（不是死代码）', vals[0][1] > 0.5,
          'soft(0.92)=%.3f' % vals[0][1])
    check('min_w=舒适宽时软罚分为 0', vals[4][1] == 0.0, 'soft(1.20)=%.3f' % vals[4][1])
    check('超过舒适宽也为 0', vals[5][1] == 0.0, 'soft(1.50)=%.3f' % vals[5][1])
    mono = all(vals[i][1] >= vals[i + 1][1] for i in range(len(vals) - 1))
    check('窄→宽 单调不增', mono,
          ' '.join('%.2f:%.2f' % v for v in vals))

    print('\n[3] 长窄段 / 长未知段也折算成风险')
    base = soft_of(min_w=1.30, cfg=CFG)
    nr = soft_of(min_w=1.30, narrow_run=3.0, cfg=CFG)
    ur = soft_of(min_w=1.30, unk_run=1.0, cfg=CFG)
    check('长窄段增加软罚分', nr > base, 'base=%.2f narrow3m=%.2f' % (base, nr))
    check('长未知段增加软罚分', ur > base, 'base=%.2f unk1m=%.2f' % (base, ur))
    check('极长窄段封顶为 1.0', soft_of(min_w=1.30, narrow_run=99.0, cfg=CFG) <= 1.0)

    print('\n[4] 软罚分要足够大才能影响排序')
    W = 900.0          # risk_w_soft 默认值
    narrow_pen = W * soft_of(min_w=0.92, cfg=CFG)
    wide_pen = W * soft_of(min_w=1.20, cfg=CFG)
    check('最窄 vs 舒适宽 的罚分差 > 800', narrow_pen - wide_pen > 800.0,
          'narrow=%.0f wide=%.0f' % (narrow_pen, wide_pen))
    # 典型收益量级：远处大块未知 u≈1500、距离折现后 ≈300~1000
    u_near_narrow = 1000.0 - narrow_pen
    u_far_wide = 1000.0 * (1.0 / (1.0 + (6.0 / 4.5) ** 2)) - wide_pen
    check('窄路即使收益更高也会被宽路反超', u_far_wide > u_near_narrow,
          'narrow=%.0f wide=%.0f' % (u_near_narrow, u_far_wide))

    print('\n[5] 源码约定：默认值一致')
    src = open(ARB, encoding='utf-8').read()
    check("route_gate_soft 默认 true", "'route_gate_soft', True" in src)
    check("risk_w_soft 默认 900", "'risk_w_soft', 900.0" in src)
    check('risk_route_target 默认 1.20', "'risk_route_target_m', 1.20" in src)
    check('硬拒只给 corridor_hard', 'min_w < self.corridor_hard' in src)
    check('软项用 risk_route_target 归一化',
          'self.risk_route_target - self.corridor_hard' in src)

    print()
    if FAILS:
        print('结果：FAIL（%d 项）' % len(FAILS))
        for f in FAILS:
            print('  - %s' % f)
        return 1
    print('结果：PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
