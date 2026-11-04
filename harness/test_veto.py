#!/usr/bin/env python3
"""test_veto.py —— V2-③ 后置 veto 的离线自测（<2 秒，不依赖 ROS）。

来历：2026-10-24 加"对非本节点派发的目标做硬闸门否决"。这条路径直接调
cancel_goal，一旦判空/判反，代价就是**把正在正常执行的（甚至返航的）目标掐掉**——
docs/P7 第 4 节刚因为这个吃过 4 轮返航失败。所以把判定逻辑拆成
veto_probe()，用合成净空场 + 假 nav_state 把每条安全约束单独焊住：

  A 探索期 + 外部目标命中硬闸门      → 必须否决（计数 +1）
  B 目标在 map 原点近旁（返航终点）   → 必须不碰
  C 目标就是自己刚派的点              → 必须不碰
  D mapping_done=True（已在返航）     → 必须不碰
  E 没收到过 nav_state（状态未知）    → 必须不碰（fail-safe）
  F 冷却期内第二次                    → 必须不碰
  G 额度用完（每局 2 次）             → 必须不碰
  H 目标所在区域本来就开阔            → 必须不碰
  I 没有活跃 goal_id（取消会失败）    → 不计数（不消耗额度）
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


class Logger:
    def __init__(self):
        self.lines = []

    def warn(self, m):
        self.lines.append(m)

    def info(self, m):
        self.lines.append(m)


class FakeSelf:
    """只实现 veto_probe / _explore_phase / region_complexity 依赖的最小状态。"""

    def __init__(self, field, nav=None):
        self._field = field
        self.pose = (0.0, 0.0, 0.0)
        self.probe_radius = 2.0
        self.probe_narrow_r = 0.35
        self.probe_escape_r = 0.6
        # veto 参数（与仲裁器默认值一致）
        self.veto_enabled = True
        self.veto_min_esc = 3
        self.veto_occ_max = 0.30
        self.veto_self_r = 0.60
        self.veto_self_sec = 8.0
        self.veto_max = 2
        self.veto_cooldown = 30.0
        self.veto_home_guard = 0.80
        self._veto_last = 0.0
        self._veto_count = 0
        self._veto_prev = None
        self._veto_detail = []
        self.pub_history = []
        self.stats = {}
        self._nav_payload = nav
        self._nav_seen = nav is not None
        self._active_uuid = [b'\x01' * 16]
        self.cancel_ok = True
        # ★ 2026-11-04 P12：inplace 模式默认关（既有用例因此行为不变）
        self.veto_inplace = False
        self.veto_migrate = False
        self.inplace_radius = 2.0
        self.inplace_tries = 24
        self.inplace_max = 3
        self.inplace_cooldown = 3.0
        self.inplace_min_disp = 0.30
        self.inplace_fallback_cancel = True
        self._inplace_last = 0.0
        self._inplace_prev = None
        self.pub_goal_update = None
        self.pub_correction = None
        self.frame = 'map'
        self.yaw = 0.0
        self.corridor_width = 1.05
        self.costmap = None
        self.log = Logger()

    def clearance_field(self):
        return self._field

    def vision_occ_mask(self):
        return None

    def get_logger(self):
        return self.log

    def _do_veto_cancel(self):
        self.cancel_called = True
        return self.cancel_ok

    def _nearest_valid_point(self, gx, gy, radius):
        return getattr(self, 'inplace_reply', None)


def build_methods(cls):
    seg = extract({'veto_probe', '_explore_phase', 'region_complexity'})
    ns = {'math': math, 'time': __import__('time'), 'json': __import__('json')}
    for name, code in seg.items():
        exec(compile(code, f'<{name}>', 'exec'), ns)
        setattr(cls, name, ns[name])


def make_field(kind, size=200):
    ox = oy = -size * RES / 2.0
    if kind == 'open':
        d = np.full((size, size), 5.0, dtype=float)
    elif kind == 'wall':                     # 全场贴墙：esc=0、occ=1.0
        d = np.full((size, size), 0.10, dtype=float)
    else:
        return None
    return (d, ox, oy, RES, size, size, None)


EXPLORE = {'mapping_done': False, 'pending_home_goal': None,
           'return_home_retry_count': 0}


def main():
    build_methods(FakeSelf)
    fails = []

    def check(name, cond, detail=''):
        print(f'{"✓" if cond else "✗"} {name}{("  " + detail) if detail else ""}')
        if not cond:
            fails.append(name)

    # A：探索期 + 外部目标在贴墙区 → 必须否决
    f = FakeSelf(make_field('wall'), nav=EXPLORE)
    r = f.veto_probe(3.0, 1.0, tag='test')
    check('A 硬闸门外部目标被否决', r is not None and f._veto_count == 1,
          f'返回={r} 计数={f._veto_count}')

    # B：原点近旁（返航终点）不碰 —— 即使区域很烂
    f = FakeSelf(make_field('wall'), nav=EXPLORE)
    r = f.veto_probe(0.2, 0.1, tag='test')
    check('B 原点近旁不否决', r is None and f._veto_count == 0, f'返回={r}')

    # C：自己刚派的点不否决
    f = FakeSelf(make_field('wall'), nav=EXPLORE)
    import time as _t
    f.pub_history.append((3.0, 1.0, _t.monotonic()))
    r = f.veto_probe(3.05, 1.02, tag='test')
    check('C 自己派的点不否决', r is None, f'返回={r}')

    # D：已 mapping_done（返航阶段）不碰
    f = FakeSelf(make_field('wall'),
                 nav={'mapping_done': True, 'pending_home_goal': None,
                      'return_home_retry_count': 0})
    r = f.veto_probe(3.0, 1.0, tag='test')
    check('D 返航阶段不否决', r is None, f'返回={r}')

    # E：没收到过 nav_state → fail-safe 不动作
    f = FakeSelf(make_field('wall'), nav=None)
    r = f.veto_probe(3.0, 1.0, tag='test')
    check('E 状态未知时不否决', r is None, f'返回={r}')

    # F：冷却期内第二次不否决
    f = FakeSelf(make_field('wall'), nav=EXPLORE)
    f.veto_probe(3.0, 1.0)
    r2 = f.veto_probe(-3.0, -1.0)          # 不同位置，避开"同目标 20 s"规则
    check('F 冷却期内不再否决', r2 is None and f._veto_count == 1,
          f'返回={r2} 计数={f._veto_count}')

    # G：额度用完（2 次）后不再否决
    f = FakeSelf(make_field('wall'), nav=EXPLORE)
    f.veto_cooldown = 0.0                   # 关冷却，单测额度
    got = [f.veto_probe(3.0 + i, 1.0 + i * 0.7) for i in range(4)]
    n_ok = sum(1 for g in got if g is not None)
    check('G 每局限 2 次', n_ok == 2 and f._veto_count == 2, f'实际 {n_ok} 次')

    # H：开阔处的目标不否决
    f = FakeSelf(make_field('open'), nav=EXPLORE)
    r = f.veto_probe(3.0, 1.0, tag='test')
    check('H 开阔目标不否决', r is None, f'返回={r}')

    # I：取消失败（无活跃 goal）不消耗额度
    f = FakeSelf(make_field('wall'), nav=EXPLORE)
    f.cancel_ok = False
    r = f.veto_probe(3.0, 1.0, tag='test')
    check('I 取消失败不计数', r is None and f._veto_count == 0,
          f'返回={r} 计数={f._veto_count}')

    # J：_explore_phase 对 pending_home_goal / retry_count 也生效
    for nav, why in (({'mapping_done': False, 'pending_home_goal': [0, 0],
                       'return_home_retry_count': 0}, 'pending_home_goal'),
                     ({'mapping_done': False, 'pending_home_goal': None,
                       'return_home_retry_count': 1}, 'return_home_retry_count')):
        f = FakeSelf(make_field('wall'), nav=nav)
        r = f.veto_probe(3.0, 1.0)
        check(f'J {why} 时不否决', r is None, f'返回={r}')

    if fails:
        for x in fails:
            print(f'  ✗ {x}')
        print('结果：FAIL')
        return 1
    print('结果：PASS（10 组、10 项断言全部通过）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
