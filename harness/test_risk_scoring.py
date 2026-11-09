#!/usr/bin/env python3
"""test_risk_scoring.py —— P13-1 派发风险评分的离线验证（不需要 ROS / Gazebo）。

验证的是**取值方向与量级**是否符合 `docs/P13_基线.md` 的实测结论：

  1) 距离折现单调：越远越小，且 p(0)=1、p(risk_dist_half)=0.5；
  2) 折现值与基线取消率曲线同一量级（<3 m 应远高于 >6 m）；
  3) 折现是**乘在收益上的**，不是小罚分 —— 远处候选只有信息量显著更大才该胜出；
     这一条如果退化成"加一个小罚分"，就会重新出现"81% 派发 >4 m"的老问题；
  4) 沿路风险：路径越窄、未知占比越高 → 风险越大；
  5) 关闭开关时所有项退化为中性值（risk_enabled=False 必须完全等价旧行为）。

用法：python3 harness/test_risk_scoring.py [-v]
"""
import importlib.machinery
import importlib.util
import os
import sys
import types

# ---- 和 test_scoring.py 同一套空壳：模块顶部 import rclpy，但构造节点才需要真实 ROS ----
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

Arbiter = arb.Arbiter
FAILS = []
VERBOSE = '-v' in sys.argv


class Cfg:
    """只带风险评分需要的属性的轻量替身（避免构造真实 ROS 节点）。"""

    risk_enabled = True
    risk_dist_half = 4.5        # 与 Arbiter 默认值一致（三点拟合基线）
    risk_route_target = 1.20    # 舒福宽度，不能取 pass_width(0.90)，否则本项恒为 0
    risk_w_route = 150.0

    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


def bind(func, obj):
    return func.__get__(obj, type(obj))


def check(name, cond, detail=''):
    if cond:
        if VERBOSE:
            print('  ok   %s  %s' % (name, detail))
        return True
    FAILS.append('%s  %s' % (name, detail))
    print('  FAIL %s  %s' % (name, detail))
    return False


def near(a, b, tol=1e-6):
    return abs(a - b) <= tol


def main():
    c = Cfg()
    disc = bind(Arbiter.risk_discount, c)
    route = bind(Arbiter.risk_route, c)

    print('\n[1] 距离折现：端点与单调性')
    check('p(0)=1', near(disc(0.0), 1.0), 'p0=%.4f' % disc(0.0))
    check('p(half)=0.5', near(disc(4.5), 0.5), 'p4.5=%.4f' % disc(4.5))
    ds = [0.0, 1.0, 2.0, 3.0, 4.0, 6.0, 9.0, 12.0, 16.0]
    ps = [disc(d) for d in ds]
    check('严格单调递减', all(ps[i] > ps[i + 1] for i in range(len(ps) - 1)),
          ' '.join('%.2f:%.3f' % (d, p) for d, p in zip(ds, ps)))

    print('\n[2] 折现曲线与基线取消率同量级')
    # 基线（剔除幻影）：<3 m 19%、4~6 m 60%、>6 m 81% 取消
    # ⇒ 期望"还能跑完"的比例约 0.81 / 0.40 / 0.19
    for d, want in ((2.0, 0.81), (5.0, 0.40), (9.0, 0.19)):
        p = disc(d)
        check('p(%g m) 在基线区间内' % d, abs(p - want) <= 0.10,
              'p=%.3f want≈%.2f' % (p, want))

    print('\n[3] 折现是乘在收益上的（这是本设计的核心）')
    # 近处、信息一般 vs 远处、信息很大
    near_u, far_u = 1000.0, 3000.0
    near_d, far_d = 2.0, 9.0
    s_near = near_u * disc(near_d)
    s_far = far_u * disc(far_d)
    check('近处(2m,u=1000) 胜过 远处(9m,u=3000)', s_near > s_far,
          'near=%.0f far=%.0f' % (s_near, s_far))
    # 远处信息量必须显著更大才能翻盘
    s_far2 = 6000.0 * disc(far_d)
    check('远处信息量翻倍后才能胜出', s_far2 > s_near,
          'far2=%.0f near=%.0f' % (s_far2, s_near))
    # 若退化成"小罚分"（线性减 35*d），远处的 3000 仍会赢；本设计必须让近处赢
    linear_near = near_u - 35.0 * near_d
    linear_far = far_u - 35.0 * 9.0
    check('对照：线性小罚分挡不住远处（说明折现必要）', linear_far > linear_near,
          'linear near=%.0f far=%.0f' % (linear_near, linear_far))

    print('\n[4] 沿路风险：窄 / 未知')
    # 目标 1.20 m：>=1.20 应为 0，低于它应单调上升
    r_wide, _ = route(min_w=1.30, unk=0.0)
    r_at, _ = route(min_w=1.20, unk=0.0)
    r_edge, _ = route(min_w=0.95, unk=0.0)
    r_floor, _ = route(min_w=0.90, unk=0.0)
    r_tight, _ = route(min_w=0.70, unk=0.0)
    check('净宽达目标时风险为 0',
          near(r_wide, 0.0, 1e-9) and near(r_at, 0.0, 1e-9),
          'w1.30=%.1f w1.20=%.1f' % (r_wide, r_at))
    check('低于目标后单调上升', r_edge < r_floor < r_tight,
          'w0.95=%.1f w0.90=%.1f w0.70=%.1f'
          % (r_edge, r_floor, r_tight))
    check('硬门阈值 0.90 本身不为 0（说明与 pass_width 解耦）',
          r_floor > 0.0, 'w0.90=%.1f' % r_floor)
    r_u0, _ = route(min_w=1.30, unk=0.0)
    r_u5, _ = route(min_w=1.30, unk=0.5)
    r_u1, _ = route(min_w=1.30, unk=1.0)
    check('未知占比越高风险越大', r_u0 < r_u5 < r_u1,
          'unk0=%.1f unk0.5=%.1f unk1=%.1f' % (r_u0, r_u5, r_u1))
    check('min_w=None 时不报错', route(None, None)[0] >= 0.0)

    print('\n[5] 开关关闭时完全中性')
    off = Cfg(risk_enabled=False)
    disc_off = bind(Arbiter.risk_discount, off)
    route_off = bind(Arbiter.risk_route, off)
    check('risk_enabled=False → p=1（所有距离）',
          all(near(disc_off(d), 1.0) for d in (0.0, 3.0, 6.0, 12.0, 99.0)))
    check('risk_enabled=False → 沿路风险=0', near(route_off(0.3, 1.0)[0], 0.0))
    # 折现关闭后排序必须退化回原始收益，等价旧行为
    check('关闭后排序等价原始收益',
          near(1000.0 * disc_off(2.0), 1000.0)
          and near(3000.0 * disc_off(9.0), 3000.0))

    print('\n[6] 参数敏感性')
    tight = Cfg(risk_dist_half=3.0)
    loose = Cfg(risk_dist_half=9.0)
    d_t = bind(Arbiter.risk_discount, tight)
    d_l = bind(Arbiter.risk_discount, loose)
    check('half 越小惩罚越重', d_t(6.0) < disc(6.0) < d_l(6.0),
          'half3=%.3f half4.5=%.3f half9=%.3f'
          % (d_t(6.0), disc(6.0), d_l(6.0)))

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
