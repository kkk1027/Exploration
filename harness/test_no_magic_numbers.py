#!/usr/bin/env python3
"""test_no_magic_numbers.py —— 判据位置不许再长出新魔法数字（S5 §5-E）。

为什么不能用"正则扫数字"
-----------------------
S5 §1.1 实测：`frontier_arbiter` 里带米/秒的数值字面量 **77 处**（含超时、权重、
半径、采样数）。用正则"发现数字就报错"要么大量误报（所有超时都会被抓），
要么加满白名单后形同虚设（白名单比代码还长）。

所以本测试改成**三条结构性断言**，全部围绕"阈值从哪来"而不是"有没有数字"：

1. **哨兵断言**：`pass_width_m` / `narrow_floor_m` 的代码默认值与 launch 默认值
   必须是 `0.0`。0.0 的语义是"未指定，用 profile 派生值"，不是真的 0 米。
   只要这两个参数里重新出现 0.90 / 0.58，这里立刻红。

2. **单入口断言**：`self.pass_width` 与 `self.narrow_floor` 在
   `frontier_arbiter` 里各自**只允许被赋值一次**，且赋值必须走
   `_metric_threshold()`。绕开这个入口直接写 `self.pass_width = <数字>`
   会把"车体几何唯一事实源"破掉。

3. **棘轮断言**：把当前所有"米/秒制参数默认值"记成一份**冻结清单**。
   新增任何未登记的 `p('<name>_m', <数字>)` / `p('<name>_sec', <数字>)`
   直接失败。这不妨碍存量 77 处，但保证总数只减不增——每清掉一个，
   就从清单里删掉一个，不再回头。

配套：`test_robot_profile.py` 保证 profile 派生量与改造前逐位相等；
本测试保证"数字不会再从代码里偷偷长回来"。两者合起来才是 S5 §5-A 的
"判据位置 0 处魔法数字"。

用法
----
    python3 harness/test_no_magic_numbers.py            # 断言
    python3 harness/test_no_magic_numbers.py --list     # 打印清单便于清理
"""
import argparse
import os
import re
import sys

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIM_PKG = os.path.join(BASE, 'src', 'stemm_cartographer_exploration_sim')
ARB = os.path.join(SIM_PKG, 'scripts', 'frontier_arbiter')
LAUNCH = os.path.join(SIM_PKG, 'launch', 'sim_explore.launch.py')

#: 已登记的"米/秒制参数默认值"（2026-11-14 冻结，共 77 个）。
#: 新增条目必须在 PR 说明里写清"为什么这个值不能由 profile 派生"。
#:
#: 为什么这些可以留：
#:   * `*_sec`：时间预算/超时，与车体几何无关（秒数不是几何量）。
#:   * `*_radius_m` / `*_dist_m` / `*_clear*_m`：**场景/搜索尺度**（探针采样半径、
#:     失败黑名单半径、候选池搜索距离），不是"车能不能过"的判据。
#:   * 已由 S5 后续步骤接管的（pass_width_m / narrow_floor_m）**保留名字但值是
#:     0.0 哨兵**，见断言 1。
KNOWN_METRIC_DEFAULTS = frozenset((
    'blacklist_sec',
    'candidate_max_age_sec',
    'cone_far_m',
    'cone_near_m',
    'contact_fail_cooldown_sec',
    'contact_hard_radius_m',
    'contact_zone_radius_m',
    'contact_zone_sec',
    'corridor_hard_m',
    'corridor_radius_m',
    'corridor_skip_far_m',
    'corridor_tail_dense_m',
    'corridor_ttl_max_sec',
    'corridor_ttl_sec',
    'cost_gate_disc_m',
    'cost_repair_radius_m',
    'escape_cooldown_sec',
    'escape_max_dist_m',
    'escape_min_clearance_m',
    'escape_turn_relax_m',
    'explore_sec',
    'failfast_blocked_sec',
    'failfast_comp_radius_m',
    'failfast_drift_m',
    'failfast_escalate_sec',
    'failfast_front_m',
    'failfast_hard_radius_m',
    'failfast_ttl_sec',
    'failfast_window_sec',
    'failfast_zone_radius_m',
    'fallback_cooldown_sec',
    'fallback_max_dist_m',
    'inplace_cooldown_sec',
    'inplace_min_disp_m',
    'inplace_min_goal_m',
    'inplace_min_robot_m',
    'inplace_min_width_m',
    'inplace_only_beyond_m',
    'inplace_radius_m',
    'narrow_blacklist_sec',
    'narrow_floor_m',
    'narrow_len_max_m',
    'near_first_radius_m',
    'noexit_pinch_m',
    'noexit_retreat_m',
    'noexit_retreat_need_m',
    'noexit_short_m',
    'noexit_turn_hold_m',
    'noexit_turn_m',
    'pass_width_m',
    'path_timeout_sec',
    'probe_escape_clear_m',
    'probe_narrow_clear_m',
    'probe_radius_m',
    'pullback_m',
    'raw_max_age_sec',
    'raw_max_dist_m',
    'raw_starve_sec',
    'region_penalty_sec',
    'republish_sec',
    'retry_radius_m',
    'retry_window_sec',
    'return_scan_radius_m',
    'return_turn_width_m',
    'risk_dist_half_m',
    'risk_route_target_m',
    'tail_park_sec',
    'unknown_span_max_m',
    'verdict_period_sec',
    'verdict_ttl_sec',
    'veto_cooldown_sec',
    'veto_home_guard_m',
    'veto_migrate_min_move_m',
    'veto_migrate_radius_m',
    'veto_self_radius_m',
    'veto_self_window_sec',
    'vision_ready_timeout_sec',
))

#: 必须走 profile 派生（代码默认值必须是 0.0 哨兵）
PROFILE_DRIVEN = ('pass_width_m', 'narrow_floor_m')

#: 已知**未派生**的几何阈值（存量，Step 1 不改值，只登记）。
#: 这三项都是"车体几何 + 余量"的判据，理应由 profile 派生；
#: 但改值会引入行为差异，超出 Step 1 的范围。清理时清一项删一项。
KNOWN_UNDERIVED_GEOMETRIC = frozenset((
    'inplace_min_width',     # 原地转所需最小净宽 0.70
    'return_turn_width',     # 返航转向所需净宽 0.85
    'escape_min_clearance',  # 脱困最小净空 0.55
))

#: 默认值取自 `self.pass_width`（即已经是派生值）的属性。
#: 这些**不是**字面量，但写成 `float(p('… ', self.pass_width))` 属于
#: "隐含派生"，必须在这里登记并校验默认确实引用 pass_width。
DERIVED_FROM_PASS_WIDTH = ('corridor_width',)

_ok = []
_bad = []


def chk(cond, label, detail=''):
    (_ok if cond else _bad).append((label, detail))
    return cond


def read(p):
    return open(p, encoding='utf-8').read() if os.path.exists(p) else ''


PARAM_RE = re.compile(
    r"p\(\s*'([a-z0-9_]+)'\s*,\s*(-?[0-9]+\.?[0-9]*(?:[eE][-+]?[0-9]+)?)\s*\)")


def scan(text):
    """返回 [(name, value)] —— 只看 `p('name', <数值字面量>)` 形式。"""
    out = []
    for name, val in PARAM_RE.findall(text):
        if name.endswith('_m') or name.endswith('_sec'):
            out.append((name, float(val)))
    return out


def launch_default(text, name):
    m = re.search(r"DeclareLaunchArgument\(\s*['\"]%s['\"]\s*,"
                  r"\s*default_value\s*=\s*['\"]([^'\"]+)['\"]" % re.escape(name),
                  text)
    return float(m.group(1)) if m else None


def test_1_sentinel():
    """1 哨兵：profile 驱动的参数默认值必须是 0.0。"""
    arb = read(ARB)
    seen = dict(scan(arb))
    for name in PROFILE_DRIVEN:
        chk(name in seen and seen[name] == 0.0,
            '1 %s 的代码默认值是 0.0 哨兵' % name,
            'got=%r' % (seen.get(name),))
    lt = read(LAUNCH)
    for arg, pname in (('pass_width', 'pass_width_m'),
                       ('narrow_floor', 'narrow_floor_m')):
        v = launch_default(lt, arg)
        chk(v == 0.0,
            '1 launch %s 的默认值是 0.0 哨兵（不再写死阈值）' % arg,
            'got=%r' % (v,))


def test_2_single_entry():
    """2 单入口：self.pass_width / self.narrow_floor 只能赋值一次且走派生入口。"""
    lines = read(ARB).splitlines()
    for attr, name in (('pass_width', 'pass_width'),
                       ('narrow_floor', 'narrow_floor')):
        assign = [(i + 1, ln.strip()) for i, ln in enumerate(lines)
                  if re.match(r'\s*self\.%s\s*=' % attr, ln)]
        chk(len(assign) == 1,
            '2 self.%s 只被赋值一次' % attr,
            '实际 %d 处：%s' % (len(assign), [n for n, _ in assign]))
        if assign:
            ln_no, body = assign[0]
            # 该处或其续行必须出现 _metric_threshold
            nxt = lines[ln_no - 1:ln_no + 2]
            chk(any('_metric_threshold' in x for x in nxt),
                '2 self.%s 的赋值走 _metric_threshold()' % attr,
                '第 %d 行：%s' % (ln_no, body))


def test_3_ratchet():
    """3 棘轮：米/秒制参数默认值只减不增。"""
    now = {n for n, _ in scan(read(ARB))}
    new = sorted(now - KNOWN_METRIC_DEFAULTS)
    gone = sorted(KNOWN_METRIC_DEFAULTS - now)
    chk(not new,
        '3 没有新增未登记的米/秒制参数默认值',
        ('新增：%s —— 请说明为什么不能由 profile 派生；' % new) if new else '')
    if gone:
        # 清掉是好事，但必须同步删清单，否则棘轮会失效
        chk(False,
            '3 清单里已清理的条目必须同步删除（否则棘轮失效）',
            '已不存在于代码中：%s' % gone)
    else:
        chk(True, '3 清单与代码一致（%d 项）' % len(KNOWN_METRIC_DEFAULTS))


def test_4_no_metric_threshold_outside_entry():
    """4 语义（同样棘轮）：宽度/净空类属性不得新增未派生的字面量。

    存量 3 处已登记：它们是**几何阈值**，理应由 profile 派生，但 S5 Step 1 的
    范围只有 pass_width / narrow_floor，改这三项会引入行为差异。所以
    先登记、不改值，后续步骤逐项清理（清一删一）。

    `corridor_width` 不在名单内：它的默认值直接取 `self.pass_width`，
    已经是派生值，不是字面量。
    """
    lines = read(ARB).splitlines()
    offenders = []
    for i, ln in enumerate(lines):
        s = ln.strip()
        m = re.match(r'self\.([a-z0-9_]*(?:width|floor|clearance|corridor))'
                     r'\s*=\s*float\(p\(', s)
        if not m:
            continue
        name = m.group(1)
        if name in PROFILE_DRIVEN:
            continue
        if name in DERIVED_FROM_PASS_WIDTH:
            chk('self.pass_width' in s,
                '4 self.%s 的默认值确实取自 pass_width' % name, s)
            continue
        if name in KNOWN_UNDERIVED_GEOMETRIC:
            continue
        offenders.append('%d: self.%s' % (i + 1, name))
    chk(not offenders,
        '4 没有新增未派生的宽度/净空类字面量',
        ('新增：%s —— 请改用 _metric_threshold 或从 profile 派生；%s'
         % (offenders, '')) if offenders else '')
    # 存量项必须真的还在（清掉后要同步删名单，否则棘轮失效）
    present = set()
    for ln in lines:
        m = re.match(r'self\.([a-z0-9_]+)\s*=\s*float\(p\(', ln.strip())
        if m:
            present.add(m.group(1))
    stale = sorted(KNOWN_UNDERIVED_GEOMETRIC - present)
    chk(not stale,
        '4 登记的未派生项与代码一致（%d 项）'
        % len(KNOWN_UNDERIVED_GEOMETRIC),
        ('已不存在，请从名单删除：%s' % stale) if stale else '')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--list', action='store_true', help='打印当前清单')
    a = ap.parse_args()

    if a.list:
        for n, v in sorted(scan(read(ARB))):
            flag = '（已登记）' if n in KNOWN_METRIC_DEFAULTS else '（未登记！）'
            print('  %-28s %-8s %s' % (n, v, flag))
        return 0

    for fn in (test_1_sentinel, test_2_single_entry, test_3_ratchet,
               test_4_no_metric_threshold_outside_entry):
        try:
            fn()
        except Exception as exc:                       # noqa: BLE001
            chk(False, '%s 抛异常' % fn.__name__, repr(exc))

    for label, detail in _ok:
        print('  ✓ %s  %s' % (label, detail))
    for label, detail in _bad:
        print('  ✗ %s  %s' % (label, detail))
    print('test_no_magic_numbers: %d 通过, %d 失败' % (len(_ok), len(_bad)))
    return 0 if not _bad else 1


if __name__ == '__main__':
    sys.exit(main())
