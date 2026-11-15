#!/usr/bin/env python3
"""覆盖率终止判据的门槛顺序与语义断言（P2）。

为什么需要这些断言
------------------
2026-11-15 实测踩到两次同类错误，都是"门槛没生效但看起来在跑"：

1. 回放脚本只比 `ncr >= theta`，忘了 manager 还有 `min_explore_sec` 与
   `min_known_cells` 两道门槛 ⇒ 得出"θ=0.98 也在 sim=16 收敛、漏探 61%"
   的荒谬结论，当时误以为是 manager 判错。

2. 前门带那次：只降 `front_stop_distance` 不降 `front_emergency_distance`，
   判定顺序倒挂 ⇒ `front_blocked` 永远到不了 5 ⇒ 保护静默失效。

共同点：**门槛之间的先后顺序和不变式，错了不会报错，只会静默失效。**
所以要用静态断言把顺序钉住。

运行
----
  python3 harness/test_coverage_gates.py
"""
import os
import re
import sys

MGR = ('/home/kbf/wheeltec_ros2/src/stemm_nav2_manager/'
       'stemm_nav2_manager/manager.py')
KERNEL = ('/home/kbf/wheeltec_ros2/src/stemm_nav2_manager/'
          'stemm_nav2_manager/coverage_kernel.py')

PASS = 0
FAIL = 0
FAILS = []


def check(name, cond, detail=''):
    global PASS, FAIL
    if cond:
        PASS += 1
        print('  ✓ %s' % name)
    else:
        FAIL += 1
        FAILS.append(name)
        print('  ✗ %s %s' % (name, detail))


def read(p):
    with open(p, encoding='utf-8') as f:
        return f.read()


mgr = read(MGR)
kern = read(KERNEL)


def block(src, marker, length=2500):
    i = src.find(marker)
    return '' if i < 0 else src[i:i + length]


print('\n=== A. 门槛必须存在且默认值可回退 ===')

for pname, default in (
        ('coverage_enabled', 'True'),
        ('coverage_auto_terminate', 'False'),
        ('coverage_threshold', '0.97'),
        ('coverage_confirm_n', '3'),
        ('coverage_stall_sec', '0.0'),
        ('coverage_min_explore_sec', '60.0'),
        ('coverage_min_known_cells', '0'),
):
    check('A.%s 声明且默认 %s' % (pname, default),
          "declare_parameter('%s', %s)" % (pname, default) in mgr)

check('A.stall_sec 默认 0 = 关闭停滞判据（用户 2026-11-15 决定）',
      "declare_parameter('coverage_stall_sec', 0.0)" in mgr,
      '用户要求"暂时只看覆盖率"，停滞条件必须默认关')
check('A.terminate 默认 False = 只观测（manager 不自己结束）',
      "declare_parameter('coverage_auto_terminate', False)" in mgr)
check('A.两个节点的终止开关名字必须不同（防双发）',
      "declare_parameter('coverage_terminate'" not in mgr,
      'manager 若也用 coverage_terminate，会与 session_driver 各发一次结束请求')

print('\n=== B. 门槛顺序：先时间、再面积、最后覆盖率 ===')

seg = block(mgr, 'ncr = c.get(\'ncr_interior\')', 2000)
check('B.1 三道门槛的判定链存在',
      'min_explore_sec' in seg and 'min_known_cells' in seg)
i_t = seg.find('elapsed < self.coverage_min_explore_sec')
i_k = seg.find('known_total < self.coverage_min_known_cells')
i_c = seg.find('_coverage_tracker.update')
check('B.2 时间门槛在面积门槛之前', 0 <= i_t < i_k, 'i_t=%d i_k=%d' % (i_t, i_k))
check('B.3 面积门槛在覆盖率判定之前', 0 <= i_k < i_c, 'i_k=%d i_c=%d' % (i_k, i_c))
check('B.4 门槛不满足时明确不收敛（不是"跳过不判"）',
      seg.count('converged, stalled, reason = False, False') >= 2)
check('B.5 起点不可达时不收敛',
      'start_not_reachable' in seg)

print('\n=== C. known_total 必须是"已知总面积"而非"可通行面积" ===')

check('C.1 known_total = free_all + occupied_all',
      re.search(r'known_total\s*=\s*int\(c\.get\(\'free_all\'\)\s*or\s*0\)\s*'
                r'\+\s*int\(c\.get\(\'occupied_all\'\)\s*or\s*0\)', seg) is not None,
      '若用 nav_known，门槛会随内切半径变化，语义漂移')
check('C.2 内核确实输出 free_all / occupied_all',
      "'free_all'" in kern and "'occupied_all'" in kern)
check('C.3 known_total 与门槛写进发布消息（便于离线复核）',
      "'known_total': known_total" in seg and
      "'known_total_floor'" in seg)

print('\n=== D. 内核的 NCR 定义不得被悄悄改回虚高版本 ===')

kseg = block(kern, 'def coverage_stats', 4500)
check('D.1 外部未知按"与图边连通"判定',
      'lab_u[0, :]' in kseg and 'edge_ids' in kseg,
      '这是房型无关的判据（2026-11-15 实测选定）')
check('D.2 ncr_interior 才是判据用值',
      "out['ncr_interior']" in kseg and 'reach_int' in kseg)
check('D.3 净空用 EDT 且把未知当障碍（保守）',
      'ndimage.distance_transform_edt(~blocked)' in kseg,
      '写成 edt(free) 会让未知处被当成可通行 ⇒ 分子虚高')
check('D.4 不可达时 ncr 为 None 而不是 0 或 1',
      re.search(r"out\['ncr'\]\s*=\s*\(navigable_known\s*/\s*float\(denom\)\)\s*"
                r"if denom\s*>\s*0\s*else\s*None", kseg) is not None)

print('\n=== E. 与"更晚收敛"要求的一致性 ===')

check('E.1 存在最短探索时间门槛',
      'coverage_min_explore_sec' in mgr)
check('E.2 存在已知面积门槛（防"还不知道自己不知道"）',
      'coverage_min_known_cells' in mgr,
      '没有它时 p14vW 在 sim=42 就 NCR=1.0000、比实际早 100 秒')
check('E.3 auto_terminate 由独立开关控制（可单独回退到定时器）',
      'auto_terminate_enabled' in seg)

print('\n' + '=' * 60)
print('coverage_gates: %d 通过, %d 失败' % (PASS, FAIL))
if FAILS:
    print('失败项:')
    for f in FAILS:
        print('  - %s' % f)
sys.exit(0 if FAIL == 0 else 1)
