#!/usr/bin/env python3
"""返航零接触回归测试（2026-11-13 C/E 修复）。

背景（p14v6_ab 实测失败链）：

    sim 236  车在 (0.54,-0.08)，距原点 0.55 m，地图净空 0.60 m
    sim 242  Nav2 status 6：planner 对替身目标 (0.37,0.33) 报
             "GridBased: failed to create plan, no valid path found"
    sim 242~277  9 次 forward_escaping（0.05 m/s × 1.8 s）沿车头方向
             把车推到 (1.04,-0.52)，净空 0.60 → 0.10 m，压在 ChairD 上
             （contact=604 samples），最终 return_home_failed_waiting_manual

本测试锁住三条不变量：

    I1  返航段的 forward/reverse 盲走逃逸不可达（代码结构断言）；
    I2  返航重试不再触发脱困（结构断言）；
    I3  几何到位判据在"已在容差内 + 已静止"时成立，在"仍在动"时不成立。

I1/I2/I3 任一被改坏，返航段接触就会回来。
"""

import math
import os
import re
import sys
import time

FAILS = []


def check(name, ok, detail=''):
    print(('  PASS  ' if ok else '  FAIL  ') + name + (('  ' + detail) if detail else ''))
    if not ok:
        FAILS.append(name)


# --------------------------------------------------------------------------
# I1 / I2：源码结构断言
# --------------------------------------------------------------------------
def structural_checks():
    src_path = os.environ.get(
        'STEMM_MANAGER_PY',
        '/home/kbf/wheeltec_ros2/src/stemm_nav2_manager/'
        'stemm_nav2_manager/manager.py')
    if not os.path.isfile(src_path):
        check('manager.py 可读', False, src_path)
        return
    text = open(src_path, encoding='utf-8').read()

    # --- I1a：forward 盲走逃逸必须被 not return_home_active 门控 ---
    # 抓 `if front_open ...:` 到其后的 navigation_state 赋值
    m = re.search(
        r'if front_open([^\n]*):\s*\n\s*self\.recovery_twist = Twist\(\)'
        r'\s*\n\s*self\.recovery_twist\.linear\.x = 0\.05'
        r'\s*\n\s*self\.recovery_until = now \+ self\.forward_escape_duration'
        r'\s*\n\s*self\.navigation_state = \'forward_escaping_from_stuck_place\'',
        text)
    check('I1a 找到 forward escape 分支', m is not None)
    if m:
        check('I1a forward escape 受 not return_home_active 门控',
              'not return_home_active' in m.group(1), repr(m.group(1))[:70])

    # --- I1b：reverse 盲走逃逸必须同样门控 ---
    m = re.search(
        r'if rear_open and self\.stuck_recovery_cycles >= 2([^\n]*):\s*\n'
        r'\s*self\.recovery_twist = Twist\(\)'
        r'\s*\n\s*self\.recovery_twist\.linear\.x = self\.reverse_escape_speed',
        text)
    check('I1b 找到 reverse escape 分支', m is not None)
    if m:
        check('I1b reverse escape 受 not return_home_active 门控',
              'not return_home_active' in m.group(1), repr(m.group(1))[:70])

    # --- I1c：返航段放弃分支必须早于 "Level 3+ aggressive turn" ---
    i_giveup = text.find('if return_home_active and self.stuck_recovery_cycles >=')
    i_aggr = text.find('Level 3+: keep alternating aggressive turns')
    check('I1c 返航放弃分支早于激进转圈分支',
          i_giveup > 0 and i_aggr > 0 and i_giveup < i_aggr,
          'giveup@%d aggr@%d' % (i_giveup, i_aggr))

    # --- I2：_schedule_return_home_retry 内不得再调 start_escape_recovery ---
    # 只检查真正的调用语句；注释里为了解释原因会提到这个函数名。
    i_fn = text.find('def _schedule_return_home_retry')
    i_next = text.find('\n    def ', i_fn + 10)
    body = text[i_fn:i_next if i_next > 0 else len(text)]
    code_lines = [l for l in body.splitlines()
                  if l.strip() and not l.strip().startswith('#')]
    calls = [l for l in code_lines if 'start_escape_recovery()' in l]
    check('I2 返航重试函数内不调用 start_escape_recovery',
          not calls, ' | '.join(c.strip() for c in calls)[:80])

    # --- 净空环半径必须向上取整（0.40/0.05 曾算成 7 格 = 0.35 m）---
    m = re.search(r'clearance_cells = max\(\s*\n?\s*1, int\(([^)]*)\)', text)
    check('D 净空环半径使用 ceil', m is not None and 'ceil' in (m.group(1) or ''),
          repr(m.group(1))[:60] if m else 'not found')


# --------------------------------------------------------------------------
# I3：几何到位判据行为
# --------------------------------------------------------------------------
class _Pos:
    def __init__(self, x, y):
        self.x, self.y = x, y


class _Pose:
    def __init__(self, x, y):
        self.pose = type('P', (), {'position': _Pos(x, y)})()


class FakeManager:
    """只装配 _return_home_geometry_satisfied 需要的字段。"""

    home_x = 0.0
    home_y = 0.0
    return_home_geometry_enabled = True
    return_home_geometry_radius = 1.5
    return_home_geometry_quiet_sec = 1.5

    def __init__(self, x, y):
        self.cur = _Pose(x, y)
        self._return_home_still_since = 0.0
        self._return_home_still_x = 0.0
        self._return_home_still_y = 0.0
        self._return_home_geometry_dist = 0.0

    def update_pose(self):
        return self.cur


def _bind_geometry_fn():
    """把真实实现绑到 FakeManager，避免重写一份逻辑。"""
    src_path = os.environ.get(
        'STEMM_MANAGER_PY',
        '/home/kbf/wheeltec_ros2/src/stemm_nav2_manager/'
        'stemm_nav2_manager/manager.py')
    text = open(src_path, encoding='utf-8').read()
    m = re.search(
        r'(    def _return_home_geometry_satisfied\(self\):.*?)\n    def ',
        text, re.S)
    if not m:
        return None
    code = m.group(1)
    code = '\n'.join(l[4:] if l.startswith('    ') else l
                     for l in code.splitlines())
    ns = {'math': math, 'time': time}
    exec(compile(code, '<geometry>', 'exec'), ns)
    return ns['_return_home_geometry_satisfied']


def behavior_checks():
    fn = _bind_geometry_fn()
    check('I3 能提取 _return_home_geometry_satisfied 实现', fn is not None)
    if fn is None:
        return

    # (1) 距原点 0.55 m（p14v6_ab 的真实终位）→ 静止够久后成立
    m = FakeManager(0.54, -0.08)
    first = fn(m)
    check('I3a 首次调用不成立（开始计时）', first is False)
    time.sleep(0.05)
    m._return_home_still_since -= 2.0          # 模拟已静止 2 s
    second = fn(m)
    check('I3b 已静止 2 s 于 0.55 m 处 → 成立', second is True)
    check('I3c 记录的实际距离正确',
          abs(m._return_home_geometry_dist - math.hypot(0.54, -0.08)) < 1e-6,
          '%.3f' % m._return_home_geometry_dist)

    # (2) 仍在容差外（3.0 m）→ 永不成立
    m = FakeManager(2.1, -2.1)
    fn(m)
    m._return_home_still_since -= 5.0
    check('I3d 距原点 3.0 m → 不成立', fn(m) is False)

    # (3) 在容差内但一直在动 → 不成立
    m = FakeManager(0.50, -0.05)
    fn(m)
    m.cur = _Pose(0.90, -0.30)                 # 位移 0.47 m
    check('I3e 容差内但仍在移动 → 不成立', fn(m) is False)

    # (4) 关闭开关时永不成立（可回退性）
    m = FakeManager(0.10, 0.10)
    m.return_home_geometry_enabled = False
    fn(m)
    m._return_home_still_since -= 5.0
    check('I3f return_home_geometry_enabled=false 时可回退', fn(m) is False)


def main():
    print('=== 返航零接触回归测试 ===')
    print('[I1/I2/D] 源码结构断言')
    structural_checks()
    print('[I3] 几何到位判据行为')
    behavior_checks()
    print()
    if FAILS:
        print('结果: %d 项失败 -> %s' % (len(FAILS), ', '.join(FAILS)))
        return 1
    print('结果: 全部通过')
    return 0


if __name__ == '__main__':
    sys.exit(main())
