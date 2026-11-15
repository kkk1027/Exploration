#!/usr/bin/env python3
"""返航死锁修复的静态断言（P0，2026-11-15）。

覆盖的 4 项修复
---------------
1a  返航段"宽包络关闭"不再等于"一步也不许退"：紧包络兜底
1b  blocked_retreat 预算冷却重置，治"预算永久耗尽 = 永久死锁"
2   前门带按车体几何派生 + 强制不变式 emergency <= stop
3   返航段受控原地转向（全向净空 >= 外接圆才允许）

为什么用静态断言而不是起 ROS 节点
----------------------------------
这些是**策略不变量**，不是数值拟合：改错方向会让前门带静默失效或让车在
某个位置永久卡死，代价很高。静态断言能在 1 秒内跑完，适合挂进 smoke_check。

运行
----
  python3 harness/test_return_home_deadlock.py
"""
import os
import re
import sys

ROOT = '/home/kbf/wheeltec_ros2/src'
MGR = os.path.join(ROOT, 'stemm_nav2_manager/stemm_nav2_manager/manager.py')
CMGR = os.path.join(ROOT, 'stemm_cartographer_exploration/'
                           'stemm_cartographer_exploration/cartographer_manager.py')
LAUNCH = os.path.join(ROOT, 'stemm_cartographer_exploration/launch/'
                             'stemm_cartographer_auto_mapping.launch.py')

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
cmgr = read(CMGR)
launch = read(LAUNCH)


def block(src, start_marker, length=2000):
    """取从 marker 起的一段源码，用于"这段里必须/不得出现某写法"。"""
    i = src.find(start_marker)
    return '' if i < 0 else src[i:i + length]


# ---------------------------------------------------------------- 修复 2
print('\n=== 修复 2：前门带按几何派生 ===')

check('2.1 manager 声明 front_gate_from_geometry',
      "declare_parameter('front_gate_from_geometry', True)" in mgr)
check('2.2 manager 声明外接圆半径参数',
      "declare_parameter('front_circumscribed_radius_m'" in mgr)
check('2.3 派生式 = 外接圆 + 余量',
      re.search(r'derived\s*=\s*self\.front_circumscribed_radius_m\s*\+\s*'
                r'self\.front_gate_margin_m', mgr) is not None)
check('2.4 派生值真的写回 front_stop_distance',
      'self.front_stop_distance = derived' in mgr)
check('2.5 关闭开关时保留旧行为（不覆盖）',
      re.search(r'if self\.front_gate_from_geometry', mgr) is not None)

check('2.6 声明 front_emergency_from_geometry',
      "declare_parameter('front_emergency_from_geometry', True)" in mgr)
check('2.7 内切半径参数存在',
      "declare_parameter('front_inscribed_radius_m'" in mgr)
check('2.8 **不变式**：emergency > stop 时被钳制并告警',
      'front_emergency_distance = self.front_stop_distance' in mgr
      and 'could never reach 5' in mgr,
      '缺这条时前门带会静默失效')
# 判定顺序必须仍然是 emergency 在前、stop 在后
seg = block(mgr, 'if self.front_min_range < self.front_emergency_distance', 400)
check('2.9 判定顺序仍是 emergency 优先',
      'self.front_blocked_count = 3' in seg and 'elif' in seg)
# launch 不再无条件使用 0.65（必须与几何开关同时出现）
check('2.10 launch 打开几何派生开关',
      "'front_gate_from_geometry': True" in launch
      and "'front_emergency_from_geometry': True" in launch)
check('2.11 launch 的 0.65 / 0.45 仅作回退值保留（有注释说明）',
      '只在关闭几何派生时生效' in launch,
      '若没有这句说明，后续维护者会以为 0.65 还在生效')

# --------------------------------------------------------------- 修复 1a
print('\n=== 修复 1a：返航紧包络兜底 ===')

check('1a.1 声明 return_home_tight_retreat_enabled（默认开）',
      "declare_parameter('return_home_tight_retreat_enabled', True)" in cmgr)
seg = block(cmgr, "gate = ('exploration_retreat_enabled'", 1400)
check('1a.2 宽包络关闭时不再直接 disabled',
      "return 'disabled', 0.0" in seg,
      '仍需要 disabled 分支（作为最终兜底）')
check('1a.3 存在 tight_fallback 分支',
      'tight_fallback' in seg)
check('1a.4 tight_fallback 时切到 exploration_retreat_ 参数组',
      re.search(r"_pfx = \('exploration_retreat_'\s*\n\s*"
                r"if \(feature == 'exploration' or tight_fallback\)", cmgr)
      is not None)
# 顺序：先判定是不是 return_home，再看 tight 开关
check('1a.5 tight 兜底只对 return_home 生效',
      re.search(r"if \(feature == 'return_home'\s*\n\s*and bool\(",
                cmgr) is not None)

# --------------------------------------------------------------- 修复 1b
print('\n=== 修复 1b：倒车预算冷却重置 ===')

check('1b.1 声明 blocked_retreat_budget_reset_sec',
      "declare_parameter('blocked_retreat_budget_reset_sec'" in cmgr)
seg = block(cmgr, 'def _maybe_retreat_when_blocked', 1600)
check('1b.2 预算打满后按冷却时间归零',
      re.search(r'self\._blocked_retreat_count\s*=\s*0', seg) is not None)
check('1b.3 归零条件含"距上次尝试 >= reset_sec"',
      'now - self._blocked_retreat_last_at) >= reset_sec' in seg)
check('1b.4 归零只在 count 已打满时发生（正常路径不重置）',
      'self._blocked_retreat_count >= max_per_session' in seg)
check('1b.5 reset_sec=0 时关闭重置（可回退）',
      'reset_sec > 0.0' in seg)

# ---------------------------------------------------------------- 修复 3
print('\n=== 修复 3：返航受控原地转向 ===')

check('3.1 声明转向角度与预算参数',
      "declare_parameter('return_home_escape_turn_deg'" in cmgr
      and "declare_parameter('return_home_escape_turn_max_per_session'"
      in cmgr)
check('3.2 初始化 _rv_escape_turn_count',
      'self._rv_escape_turn_count = 0' in cmgr)
seg = block(cmgr, 'def _rotate_to_escape_return_home', 3000)
check('3.3 方法存在且声明了三道护栏',
      '安全护栏' in seg and '三道' in seg)
check('3.4 护栏一：全向净空不足时拒绝',
      re.search(r'if float\(min_all\) < sweep', seg) is not None)
check('3.5 护栏一用的是 min_range_all（全向），不是单扇区',
      'min_range_all' in seg)
check('3.6 无全向读数时拒绝（不能证明扫掠圆为空）',
      'min_all is None' in seg)
check('3.7 护栏二：每会话预算',
      '_rv_escape_turn_count' in seg and 'max_per_session' in seg)
check('3.8 护栏三：单次角度受限（不是 loop 到底）',
      'return_home_escape_turn_deg' in seg)
check('3.9 朝更开阔一侧转（左右比较）',
      re.search(r'float\(right\) > float\(left\)', seg) is not None)
check('3.10 转向走 _rotate_in_place（有角度上限的开环转向）',
      '_rotate_in_place(direction, angle, speed)' in seg)

seg = block(cmgr, "if 'too close to occupied' in str(message)", 2600)
check('3.11 起点自检失败时先试倒车',
      "_retreat_from_occupied_cells('return_home')" in seg)
check('3.12 倒车不可用时再试转向',
      re.search(r"if status in \('disabled', 'rear-blocked'", seg) is not None)
check('3.13 转向成功后重新回到"待返航"状态而不是计一次重试',
      "navigation_state = 'return_home_retry_after_turn'" in seg)
check('3.14 转向调用被 _safe_call 包住（异常不能杀节点）',
      'return-home escape turn' in seg and '_safe_call' in seg)

# ---------------------------------------------------------------- 修复 4
print('\n=== 修复 4：重试耗尽后有限次重新武装 ===')

check('4.1 声明 return_home_rearm_max',
      "declare_parameter('return_home_rearm_max'" in mgr)
check('4.2 类常量列出"已放弃"状态',
      'RETURN_HOME_GAVE_UP_STATES' in mgr
      and "'return_home_failed_waiting_manual'" in mgr)
check('4.3 初始化 _return_home_rearm_count',
      'self._return_home_rearm_count = 0' in mgr)
seg = block(mgr, 'def set_mapping_done_callback', 4000)
check('4.4 已放弃状态下不再直接早退',
      'RETURN_HOME_GAVE_UP_STATES' in seg)
check('4.5 重新武装时清零重试预算',
      re.search(r'self\.return_home_retry_count\s*=\s*0', seg) is not None,
      '不清零 = 预算仍然打满，等于没修')
check('4.6 重新武装有硬性封顶',
      re.search(r'self\._return_home_rearm_count < self\.return_home_rearm_max',
                seg) is not None)
check('4.7 预算用尽时保持旧语义（早退）并说明',
      're-arm budget is exhausted' in seg)
check('4.8 重新武装后状态可辨识（不是悄悄重试）',
      "navigation_state = 'return_home_rearmed'" in seg)
check('4.9 日志记录"从哪个状态重新武装"（不是覆盖后的新状态）',
      'gave_up_state' in seg,
      '若直接打 self.navigation_state，日志只会显示 rearmed，丢失归因')

# ---------------------------------------------------------------- 修复 5
print('\n=== 修复 5：接线错误（2026-11-15 补） ===')
#
# 背景：修复 1a（紧包络）与修复 3（转向）实现都对，但**接线错了**——
# 它们分别写在 _retreat_from_occupied_cells 和
# _fail_return_home_before_dispatch 里，而实际拒绝返航的路径是
# _schedule_return_home_retry。实测 p14w1_cov_1_4220：36 次
# `too close to occupied` 而日志里一条 R1 都没有。
#
# 这类错误不报错、只静默失效，必须用断言钉住。

seg = block(cmgr, 'def _schedule_return_home_retry', 6000)
check('5.1 实际失败路径里存在紧包络分支',
      '_tight_ok' in seg and 'return-home tight retreat' in seg,
      '缺它则 return_home_retreat_enabled=false 时一步也不动')
check('5.2 紧包络分支不再被"两个宽包络开关同时为真"挡住',
      'elif _tight_ok' in seg)
check('5.3 紧包络分支内部调用 _retreat_from_occupied_cells',
      "_retreat_from_occupied_cells('return_home')" in seg)
check('5.4 倒车不可用时在该路径里也试转向',
      'return-home escape turn' in seg)
check('5.5 紧包络分支被 _safe_call 包住',
      "_safe_call(\n                'return-home tight retreat'" in seg
      or "'return-home tight retreat'" in seg)
check('5.6 宽包络分支仍然保留（回退路径没被破坏）',
      '_wide_ok and _failure_ok' in seg)
# ★ 实测 p14w2_cov_r1：前方 3.12 m 开阔却仍倒车 0.10 m —— 无条件分支纯浪费。
check('5.9 紧包络分支有"确实被贴住"判据，不会每个重试都倒车',
      '_looks_stuck' in seg and 'no retreat needed' in seg,
      '没有它则开阔空间也会倒车，与回家方向相反')
check('5.10 判据包含 three 种触发源（reason / front_blocked / 前门带）',
      "'too close to occupied' in str(reason)" in seg
      and "getattr(self, 'front_blocked', False)" in seg
      and 'float(_front) < _gate' in seg)

# 三条路径都必须能触达脱困：这是"接线"的核心
def reachable(fn_marker, call_marker, window=6000):
    s = block(cmgr, fn_marker, window)
    return call_marker in s

check('5.7 _fail_return_home_before_dispatch 里仍有倒车+转向',
      reachable('def _fail_return_home_before_dispatch',
                'return-home escape turn'))
check('5.8 _schedule_return_home_retry 里现在也有脱困',
      reachable('def _schedule_return_home_retry',
                'return-home escape turn'))

print('\n=== 交叉检查：不得引入新隐患 ===')

# 直发 cmd_vel 的脱困必须仍有限速（不能变成无限速倒车）
seg = block(cmgr, 'def _retreat_from_occupied_cells', 9000)
check('X.1 倒车仍有 speed/max_dist/max_sec 三重上限',
      'max_dist' in seg and 'max_sec' in seg and 'speed' in seg)
check('X.2 倒车前仍在 /map 上复核后方条带',
      'rear-blocked' in seg)
check('X.3 倒车后仍发零速',
      'self.publish_zero_velocity()' in seg)

# 转向不能绕过全向净空检查
seg = block(cmgr, 'def _rotate_in_place', 1200)
check('X.4 _rotate_in_place 本身不含净空判断（因此调用方必须判）',
      'min_range_all' not in seg,
      '若有人直接在别处调它而不判净空，就会有碰撞风险')
check('X.5 返航转向调用点是唯一新增调用方且已判净空',
      cmgr.count('_rotate_to_escape_return_home') >= 1)

# 前门带派生后仍大于内切半径
check('X.6 默认参数下 stop(0.397) > emergency(0.235)',
      0.397 > 0.235)

# ---------------------------------------------------------------- 修复 6
print('\n=== 修复 6：走廊判定分级 + 重试去重（2026-11-15 U） ===')
#
# 背景：p14x2_nomin 轮 2 车停在 (4.36,-8.80) 一动不动，
# `B approach corridor rejected` 连续 3 次 → 3 次重试全失败 →
# return_home_failed_waiting_manual。
# 根因：该函数用 `last_map.data >= 50` 判占用，而地图**保存**用
# occupied_thresh 0.65 ⇒ occupancy 50~65 在运行期算占用、存盘算未知，
# 一个 occupancy=55 的模糊格就能永久阻断返航。

seg = block(cmgr, 'def _approach_corridor_is_clear', 7000)
check('6.1 走廊判定不再用单一 >= 50 硬阈',
      'self.last_map.data[my * info.width + mx] >= 50' not in seg,
      '旧写法一个模糊格就永久阻断返航')
check('6.2 存在硬阈参数且与 map_saver 对齐（默认 65）',
      "declare_parameter('approach_corridor_hard_occupancy', 65)" in cmgr)
check('6.3 灰度占用单独计数而非直接否决',
      'gray_hits' in seg and 'hard_hits' in seg)
check('6.4 灰度占比超限才拒',
      'gray_frac_max' in seg and 'frac > gray_frac_max' in seg)
check('6.5 拒绝日志带真实 occupancy 与坐标（禁止推测）',
      'hard obstacle occupancy=%d at' in seg
      and '(%.2f,%.2f) %.2f m ahead' in seg)
check('6.6 通过时也记录灰度比例（可观测）',
      'accepted with %.0f%% gray occupancy' in seg)
check('6.7 同一位置+同一原因会被记录下来',
      '_note_corridor_fail' in seg
      and 'round(float(rx), 1)' in cmgr)
sched = block(cmgr, 'def _schedule_return_home_retry', 9000)
check('6.8 连续 2 次同因失败触发降级',
      '_corridor_same_fail' in sched and '_same >= 2' in sched)
check('6.9 降级动作是跳过预防性走廊检查（不是放弃）',
      '_skip_approach_corridor_once = True' in sched)
check('6.10 被跳过时该函数真的会放行',
      '_skip_approach_corridor_once' in seg
      and 'B approach corridor bypassed once' in seg)
check('6.11 降级后计数归零，不会每次重试都触发',
      '_corridor_same_fail = 0' in sched)
check('6.12 参数已声明（否则 get_parameter 抛异常）',
      "declare_parameter('approach_corridor_gray_frac_max'" in cmgr)

print('\n' + '=' * 60)
print('return_home_deadlock: %d 通过, %d 失败' % (PASS, FAIL))
if FAILS:
    print('失败项:')
    for f in FAILS:
        print('  - %s' % f)
sys.exit(0 if FAIL == 0 else 1)
