#!/usr/bin/env python3
"""test_scoring.py —— 前沿收益函数的离线验证（不需要 ROS 节点、不需要 Gazebo，秒级返回）。

它直接 import frontier_arbiter 里的纯函数打分内核，用合成占据栅格验证"取值方向"是否符合
用户要的探索策略：
  1) 大块未知 ≫ 薄条未知 ≫ 已探完只剩碎角；
  2) 同样大小但更近的候选略优先；
  3) 被罚（黑名单/受阻/撞过）的候选会掉下去；
  4) 前沿宽度小（碎角）会降权但不为 0（留到最后细化）。

用法：harness/test_scoring.py [-v]
"""
import importlib.util
import os
import sys

# 打分内核是纯函数，但模块顶部会 import rclpy（节点本身不构造）。
# 没source ROS 时给个空壳，保证离线测试在任何 shell 里都能跑。
try:                                    # pragma: no cover
    import rclpy  # noqa: F401
except Exception:                       # pragma: no cover
    import types
    for name in ('rclpy', 'rclpy.node', 'rclpy.qos', 'action_msgs', 'action_msgs.msg',
                 'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs', 'nav_msgs.msg',
                 'std_msgs', 'std_msgs.msg', 'wheeltec_rrt_msg', 'wheeltec_rrt_msg.msg'):
        mod = types.ModuleType(name)
        mod.__getattr__ = lambda attr: object  # type: ignore
        sys.modules.setdefault(name, mod)
    sys.modules['rclpy'].node = sys.modules['rclpy.node']
    class _N:  # 占位
        def __init__(self, *a, **k):
            pass
    sys.modules['rclpy.node'].Node = _N
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

HERE = os.path.dirname(os.path.abspath(__file__))
ARB = os.path.join(HERE, '..', 'src', 'stemm_cartographer_exploration_sim', 'scripts', 'frontier_arbiter')

import importlib.machinery
_loader = importlib.machinery.SourceFileLoader('frontier_arbiter', ARB)
spec = importlib.util.spec_from_loader('frontier_arbiter', _loader)
arb = importlib.util.module_from_spec(spec)
sys.modules['frontier_arbiter'] = arb
spec.loader.exec_module(arb)

CFG = {
    'info_radius': 1.0,
    'block_cap': 6000,
    'block_radius': 6.0,
    'width_radius': 1.5,
    'min_width': 6,
    'width_cap': 8,
    'w_block': 1.5,
    'w_width': 40.0,
    'w_dist': 0.5,
    'ratio_floor': 0.35,
    'small_frontier_factor': 0.4,
}


def make_grid(w=120, h=120, res=0.05, fill=0):
    """以 (0,0) 为左下角、全自由/未知的栅格。"""
    return {'res': res, 'w': w, 'h': h, 'ox': 0.0, 'oy': 0.0, 'data': [fill] * (w * h)}


def fill_rect(g, x0, y0, x1, y1, value):
    x1 = min(x1, g['w'])
    y1 = min(y1, g['h'])
    for j in range(max(0, y0), y1):
        base = j * g['w']
        for i in range(max(0, x0), x1):
            g['data'][base + i] = value


def cell(g, x, y):
    return arb.to_cell(g, x, y)


def main():
    verbose = '-v' in sys.argv
    fails = []

    def check(name, cond, detail=''):
        print(('  ✓ ' if cond else '  ✗ ') + name + (f'  {detail}' if detail else ''))
        if not cond:
            fails.append(name)

    # ---- 场景：3x3 m 已知自由房间，墙外是未知/占据，构造三种典型"待探索口" ----
    g = make_grid(200, 200, 0.05, fill=100)      # 先全部占据
    fill_rect(g, 70, 70, 130, 130, 0)            # 房间 x,y ∈ [3.5, 6.5) m 自由
    # A 大块未知：右侧 x>=6.5 是一整片未知（与自由区直接相邻）
    fill_rect(g, 130, 40, 200, 200, -1)
    # B 一缝到底：顶部伸进墙里、2 格宽、2 m 长的死角未知缝（只有 2 个自由格与它相邻）
    fill_rect(g, 80, 130, 82, 170, -1)
    # C 小坑碎角：房间内部 2x2 格（10x10 cm）未知小坑
    fill_rect(g, 80, 80, 82, 82, -1)

    pose = (5.0, 5.0)
    cands = [(6.4, 5.0), (4.0, 6.4), (3.95, 3.95)]
    ranked = arb.rank_candidates(g, cands, CFG, pose=pose)
    names = {(6.4, 5.0): 'A 大块未知', (4.0, 6.4): 'B 一缝到底', (3.95, 3.95): 'C 小坑碎角'}
    print('\n[1] 排序（大块未知应当第一）')
    for r in ranked:
        tag = names.get((r['x'], r['y']), '?')
        print('    %-14s score=%8.2f  unknown=%3d block=%4d width=%3d ratio=%.2f'
              % (tag, r['score'], r['unknown'], r['block'], r['width'], r['ratio']))
    # 真实系统里 2x2 小坑会先被"未知格数 >= min_unknown_cells(30)"挡掉，
    # 这里单独验证"过了规模门槛之后"的排序：大块未知 > 一缝到底。
    min_unknown = 30
    passed = [r for r in ranked if r['unknown'] >= min_unknown]
    check('过了规模门槛的候选里：大块未知 > 一缝到底',
          len(passed) >= 2 and passed[0]['x'] == 6.4 and passed[1]['y'] == 6.4,
          f'passed={[(r["x"], r["y"]) for r in passed]}')
    dropped = [r for r in ranked if r['unknown'] < min_unknown]
    check('小坑碎角被规模门槛挡掉（unknown < 30）',
          len(dropped) == 1 and abs(dropped[0]['x'] - 3.95) < 1e-6,
          f"unknown={dropped[0]['unknown'] if dropped else 'n/a'}")
    check('大块未知的 block 达到上限（说明它连到一大片未知）',
          ranked[0]['block'] >= CFG['block_cap'])

    # ---- [2] 同样块大小，近的略优先 ----
    g2 = make_grid(160, 160, 0.05, fill=-1)
    fill_rect(g2, 40, 40, 120, 120, 0)
    near, far = (4.0, 5.8), (4.0, 5.8)
    r2 = arb.rank_candidates(g2, [near, far], CFG, pose=(4.0, 4.0))
    d1 = arb.score_one(g2, 4.0, 5.8, 1.8, CFG)
    d2 = arb.score_one(g2, 4.0, 5.8, 9.0, CFG)
    print('\n[2] 距离惩罚')
    check('更近的候选分更高', d1['score'] > d2['score'],
          f"near={d1['score']:.1f} far={d2['score']:.1f}")

    # ---- [3] 惩罚生效 ----
    s0 = arb.score_one(g, 6.4, 5.0, 1.0, CFG, penalty=0.0)
    s1 = arb.score_one(g, 6.4, 5.0, 1.0, CFG, penalty=800.0)
    print('\n[3] 惩罚（黑名单/受阻/撞过）')
    check('惩罚越大分越低', s1['score'] < s0['score'],
          f"p0={s0['score']:.1f} p800={s1['score']:.1f}")

    # ---- [4] 碎角降权但不为负无穷 ----
    small = arb.score_one(g, 4.0, 6.4, 1.0, CFG)
    print('\n[4] 碎角处理')
    check('窄缝 width 很小（< min_width）', small['width'] < CFG['min_width'],
          f"width={small['width']} unknown={small['unknown']} block={small['block']}")
    cfg_off = dict(CFG, small_frontier_factor=1.0)
    wide = arb.score_one(g, 4.0, 6.4, 1.0, cfg_off)
    check('降权确实生效（关掉降权后同候选分更高）', wide['score'] > small['score'],
          f"off={wide['score']:.1f} on={small['score']:.1f}")
    check('碎角分数仍为有限值（降权而非禁用）', small['score'] == small['score'])

    # ---- [5] 已探完区域（ratio 低）应当低于同尺寸未知区 ----
    g3 = make_grid(200, 200, 0.05, fill=0)     # 全已知自由
    fill_rect(g3, 150, 150, 200, 200, -1)      # 远处一大块未知（7.5 m 外）
    known_area = arb.score_one(g3, 2.5, 2.5, 1.0, CFG)               # 已探完的空地
    unknown_area = arb.score_one(g3, 7.4, 7.4, 1.0, CFG)             # 未知块边缘
    print('\n[5] 已探完 vs 未知大部')
    check('未知大块 > 已探完区域', unknown_area['score'] > known_area['score'],
          f"unknown={unknown_area['score']:.1f} known={known_area['score']:.1f}")

    # ---- [6] 两级打分（预筛 + 完整）与全量完整打分结论一致 ----
    many = [(6.4, 5.0), (4.0, 6.4), (3.95, 3.95), (5.0, 5.0), (6.0, 4.0), (4.5, 5.5)]
    full = arb.rank_candidates(g, many, CFG, pose=pose)
    two = arb.rank_candidates(g, many, CFG, pose=pose, prefilter_n=4)
    print('\n[6] 两级打分')
    check('预筛后的第 1 名与全量一致',
          full and two and (full[0]['x'], full[0]['y']) == (two[0]['x'], two[0]['y']),
          f"full0=({full[0]['x']},{full[0]['y']}) two0=({two[0]['x']},{two[0]['y']})" if full and two else '')

    print()
    if fails:
        print(f'结果：FAIL（{len(fails)} 项）：' + '、'.join(fails))
        return 1
    print('结果：PASS（6 组、11 项断言全部通过）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
