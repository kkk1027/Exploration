#!/usr/bin/env python3
"""P4/P5 回归测试：先远后近两阶段 + 远目标准入 + 中继降级。

背景（2026-11-11，用户批准的"候选落点倾向"）：
  候选序列内先去"当前位姿下最远的未知格内候选"，再去剩下未探索的未知区 ——
  先探出室内大致框架、再逐步补齐。**只排序，不改候选生成**。

必须锁死的性质：
  I 补齐阶段的排序与改造前**逐位等价**（否则 baseline 不可比）
  J 框架阶段排序 = 距离降序（越远越优先），分数只做并列裁决
  K 阶段切换 (a)+(b)：前沿停滞 + 分量数低 → 框架降为补齐
  L 阶段可回退但带滞回：前沿重新变远才切回框架，且冷却期内不抖
  M 中继点取"已算出那条路径"上 far_relay_m 弧长处、且拒绝坐标不匹配的路径

用法：python3 harness/test_phase_far.py
"""
import math
import os
import sys
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARB = os.path.join(ROOT, 'src', 'stemm_cartographer_exploration_sim',
                   'scripts', 'frontier_arbiter')


def _load():
    src = open(ARB, encoding='utf-8').read()
    start = src.index('    # ================= P4（2026-11-11）先远后近两阶段')
    end = src.index('    def probe_pool(self, scored, raw_of):')
    ns = {'math': math}
    exec(compile('class _Shim:\n    def __init__(self):\n        pass\n'
                 + src[start:end], '<shim>', 'exec'), ns)  # noqa: S102
    return ns['_Shim']


LOGS = []


def _shim(cls, **kw):
    o = cls()
    o.frame_far_first = kw.get('frame_far_first', True)
    o.strategy_phase = 'frame' if o.frame_far_first else 'fill'
    o.frame_depth_min = 0.0
    o.frame_supply_empty_ticks = kw.get('frame_supply_empty_ticks', 3)
    o.frame_stall_ticks = kw.get('frame_stall_ticks', 20)
    o.frame_growth_cells = kw.get('frame_growth_cells', 400)
    o.frame_regions_max = kw.get('frame_regions_max', 6)
    o.frame_hysteresis = kw.get('frame_hysteresis', 20.0)
    o.frame_regrow_cells = kw.get('frame_regrow_cells', 2500)
    o.far_relay_on = kw.get('far_relay_on', True)
    o.far_relay_m = kw.get('far_relay_m', 4.0)
    o.far_relay_min_goal = kw.get('far_relay_min_goal', 1.5)
    o.pose = kw.get('pose', (0.0, 0.0))
    o.stats = {}
    o._phase_t = 0.0
    o._phase_known_best = None
    o._phase_switch_known = None
    o._phase_stall = 0
    o._phase_supply_empty = 0
    o._far_supply = kw.get('far_supply', 0)
    o._phase_forced = False
    o._dispatch_path = kw.get('last_path_w')
    o._dispatch_goal = kw.get('last_path_goal')
    # 可注入的"已探明格数 / 分量数"
    known = kw.get('known', 10000)
    nreg = kw.get('nreg', 2)
    o.known_cells = lambda: known
    o.unknown_region_field = lambda: (None if nreg is None
                                      else {'n': nreg})
    # 中继点合格判据：注入后可控
    o._veto_ok = kw.get('veto_ok', lambda x, y: True)
    o.get_logger = lambda: SimpleNamespace(
        info=lambda m: LOGS.append(('info', m)),
        warn=lambda m: LOGS.append(('warn', m)))
    return o


def run():
    ok = 0
    bad = []
    cls = _load()

    # ---- I 补齐阶段：与改造前逐位等价 ----
    items = [(900.0, -1.5, 1.1, 1.1),
             (900.0, -3.0, 2.2, 2.2),
             (800.0, -6.0, 3.3, 3.3),
             (1200.0, -9.0, 4.4, 4.4)]
    o = _shim(cls, frame_far_first=False)
    got = o._sort_candidates(items)
    want = sorted(items, reverse=True)
    if got == want:
        ok += 1
        print('  I ✓ 补齐阶段排序与 sorted(reverse=True) 完全一致')
    else:
        bad.append('I 补齐阶段不等价：%r != %r' % (got, want))

    # ---- J 框架阶段：越远越优先 ----
    o = _shim(cls, frame_far_first=True)
    got = o._sort_candidates(items)
    dists = [-q[1] for q in got]
    if dists == sorted(dists, reverse=True) and dists[0] == 9.0:
        ok += 1
        print('  J ✓ 框架阶段距离降序：%s' % (['%.1f' % d for d in dists],))
    else:
        bad.append('J 框架阶段未按距离降序：%r' % (dists,))

    # 并列距离时按分数裁决
    tie = [(100.0, -4.0, 1.0, 1.0), (900.0, -4.0, 2.0, 2.0)]
    got = _shim(cls, frame_far_first=True)._sort_candidates(tie)
    if got[0][0] == 900.0:
        ok += 1
        print('  J ✓ 等距时按分数降序做并列裁决')
    else:
        bad.append('J 并列裁决错：%r' % (got,))

    # ---- K 框架 → 补齐：远候选供给耗尽（用户 D2 原意，2026-11-13 改）----
    LOGS.clear()
    o = _shim(cls, known=10000, nreg=2, frame_supply_empty_ticks=3,
              frame_hysteresis=0.0)
    o._far_supply = 0
    for t in (1.0, 2.0, 3.0, 4.0):
        o._far_supply = 0
        o._update_strategy_phase(t)
    if o.strategy_phase == 'fill' and o.stats.get('phase_to_fill_supply') == 1:
        ok += 1
        print('  K ✓ 远候选连续 3 拍为 0 → 切补齐（供给驱动，1 次）')
    else:
        bad.append('K 供给耗尽未切：phase=%s stats=%r'
                   % (o.strategy_phase, o.stats))

    # 远候选一直有 → 不切（即使地图不长）
    o = _shim(cls, known=10000, nreg=2, frame_supply_empty_ticks=3,
              frame_hysteresis=0.0, frame_stall_ticks=99)
    for t in (1.0, 2.0, 3.0, 4.0, 5.0):
        o._far_supply = 4
        o._update_strategy_phase(t)
    if o.strategy_phase == 'frame':
        ok += 1
        print('  K ✓ 远候选一直有（4 个）→ 不切，保持框架')
    else:
        bad.append('K 有货却切了：%s' % o.strategy_phase)

    # 兜底：远候选有但地图长期停滞 → 也切
    o = _shim(cls, known=10000, nreg=2, frame_supply_empty_ticks=99,
              frame_hysteresis=0.0, frame_stall_ticks=3)
    for t in (1.0, 2.0, 3.0, 4.0):
        o._far_supply = 4
        o._update_strategy_phase(t)
    if o.strategy_phase == 'fill' and o.stats.get('phase_to_fill_stall') == 1:
        ok += 1
        print('  K ✓ 兜底：远候选有但地图 3 拍停滞 → 也切补齐')
    else:
        bad.append('K 兜底未生效：phase=%s stats=%r'
                   % (o.strategy_phase, o.stats))

    # ---- L 补齐 → 框架：远候选重新出现 ----
    o = _shim(cls, known=10000, nreg=2, frame_supply_empty_ticks=3,
              frame_hysteresis=0.0)
    for t in (1.0, 2.0, 3.0, 4.0):
        o._far_supply = 0
        o._update_strategy_phase(t)
    switched = o.strategy_phase
    o._far_supply = 2
    o._update_strategy_phase(5.0)
    if switched == 'fill' and o.strategy_phase == 'frame':
        ok += 1
        print('  L ✓ 补齐后远候选重新出现（2 个）→ 切回框架')
    else:
        bad.append('L 未切回框架：%s → %s' % (switched, o.strategy_phase))

    # 滞回：冷却期内不切
    o = _shim(cls, known=10000, nreg=2, frame_supply_empty_ticks=1,
              frame_hysteresis=100.0)
    for t in (1.0, 5.0, 9.0, 13.0):
        o._far_supply = 0
        o._update_strategy_phase(t)
    if o.strategy_phase == 'frame':
        ok += 1
        print('  L ✓ 滞回 100 s 生效：冷却期内不切换')
    else:
        bad.append('L 滞回失效：%s' % o.strategy_phase)

    # ---- M 中继点：取路径上 far_relay_m 弧长处 ----
    # 沿 +x 每 0.5 m 一个点的直线路径，目标 (8,0)
    path = [(0.5 * i, 0.0) for i in range(1, 17)]
    o = _shim(cls, pose=(0.0, 0.0), last_path_w=path,
              last_path_goal=(8.0, 0.0), far_relay_m=4.0,
              far_relay_min_goal=1.5)
    p = o.relay_point(8.0, 0.0)
    if p is not None and abs(p[0] - 4.0) < 0.6 and o.stats.get('far_relay_ok') == 1:
        ok += 1
        print('  M ✓ 中继点 %.2f m（期望 ≈4.0），命中计数 1' % p[0])
    else:
        bad.append('M 中继点错误：%r stats=%r' % (p, o.stats))

    # 坐标不匹配 → 拒绝（防止用到上一个候选的路径）
    o = _shim(cls, pose=(0.0, 0.0), last_path_w=path,
              last_path_goal=(8.0, 0.0))
    if o.relay_point(3.0, 3.0) is None and o.stats.get('far_relay_miss') == 1:
        ok += 1
        print('  M ✓ 目标坐标不匹配 → 拒绝复用旧路径')
    else:
        bad.append('M 未拒绝错配路径：stats=%r' % (o.stats,))

    # 太接近目标的中继点应被跳过（不然等于没改）
    o = _shim(cls, pose=(0.0, 0.0), last_path_w=path,
              last_path_goal=(8.0, 0.0), far_relay_m=4.0,
              far_relay_min_goal=9.0)
    if o.relay_point(8.0, 0.0) is None:
        ok += 1
        print('  M ✓ 中继点离目标不足 9.0 m 时放弃（无实质区别）')
    else:
        bad.append('M 未跳过贴近目标的点')

    print()
    total = ok + len(bad)
    if bad:
        print('❌ %d/%d 通过' % (ok, total))
        for b in bad:
            print('   -', b)
        return 1
    print('✅ %d/%d 全部通过' % (ok, total))
    return 0


if __name__ == '__main__':
    sys.exit(run())
