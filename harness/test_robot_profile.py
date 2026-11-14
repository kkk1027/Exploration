#!/usr/bin/env python3
"""test_robot_profile.py —— S5 Step 1 等价性闸门 + 结构断言（< 1 s，可挂 smoke_check）。

为什么这个测试是"地基"
----------------------
S5 §6 R1：派生量一旦不等于现值，后面所有 A/B 都不可解释。所以这里的断言
**容差 0**（实际取 1e-9，只为吸收十进制小数转二进制的末位误差）。

五组断言
--------
A 闸门      `pass_width_m` / `narrow_floor_m` / `inscribed_radius_m` 与改造前逐位相等
B 派生自洽  `pass_width = body_width + 2×side`；三个 profile 全部成立
C 单源       profile 的几何与 collision_probe / escape_supervisor / patch_sim_time
             **三处独立定义**逐一相等 —— 这是"4 处几何"收敛成 1 处的机械证明
D 口径       `collision_radius_m`(0.370) ≠ `circumscribed_radius_m`(0.397) 是故意的；
             `inscribed_radius_m`(0.235) ≠ `min_origin_edge_m`(0.18) 也是故意的
E 能力       阿克曼 `turn_outer_radius_m` 不被低估；不可原地转时 turn_corridor > 0

用法
----
    python3 harness/test_robot_profile.py            # 全跑
    python3 harness/test_robot_profile.py -v         # 逐项打印
"""
import argparse
import math
import os
import re
import sys

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIM_PKG = os.path.join(BASE, 'src', 'stemm_cartographer_exploration_sim')
SCRIPTS = os.path.join(SIM_PKG, 'scripts')
sys.path.insert(0, SCRIPTS)

from robot_model import ProfileError, available_profiles, load_profile  # noqa: E402

ARB = os.path.join(SCRIPTS, 'frontier_arbiter')
COLL = os.path.join(SCRIPTS, 'collision_probe')
SUP = os.path.join(SCRIPTS, 'escape_supervisor')
PATCH = os.path.join(SCRIPTS, 'patch_sim_time')
LAUNCH = os.path.join(SIM_PKG, 'launch', 'sim_explore.launch.py')
PROFILES = ('wheeltec_diff', 'omni_mecanum', 'ackermann_small')

TOL = 1e-9

#: 改造前的真实值（S5 §3 M1 表）。硬编码是**故意的**：闸门必须拿一个
#: 独立于 profile 的来源对照，否则"用 profile 验 profile"等于没验。
GATE = (
    ('pass_width_m', 0.90),
    ('narrow_floor_m', 0.58),
    ('inscribed_radius_m', 0.235),
)

_ok = []
_bad = []


def chk(cond, label, detail=''):
    (_ok if cond else _bad).append((label, detail))
    return cond


def near(a, b, tol=TOL):
    return abs(float(a) - float(b)) <= tol


def read(p):
    return open(p, encoding='utf-8').read() if os.path.exists(p) else ''


def param_default(text, name, cast=float):
    m = re.search(r"p\(\s*['\"]%s['\"]\s*,\s*([-0-9.eE]+)\s*\)" % re.escape(name),
                  text)
    return cast(m.group(1)) if m else None


# --------------------------------------------------------------------------
def test_A_gate():
    """A 闸门：wheeltec_diff 派生量 == 改造前现值（**逐位相等**）。

    S5 §5-A 写的是"0 容差相等"。`parse`/`_metric_threshold` 都按浮点比较，
    所以这里的断言用 `==` 而不是 `abs(diff) < 1e-9`：只要有一个 ulp 的偏差
    就会红。`profile.py` 里对两个阈值做了 1 µm 量化来保证这一点（见那里的注释）。
    """
    prof = load_profile('wheeltec_diff')
    for attr, want in GATE:
        got = getattr(prof, attr)
        chk(got == want,
            'A 闸门 %s 与 %.3f 逐位相等' % (attr, want),
            'got=%.17g diff=%.3e' % (got, abs(got - want)))
    # 冻结的 Step 0 快照里还有两个半径，也一并锁住（这两个按 1e-6 快照给定）
    chk(abs(prof.circumscribed_radius_m - 0.39702) <= 5e-6,
        'A 闸门 circumscribed_radius_m ≈ 0.39702（Nav2 外接）',
        '%.9f' % prof.circumscribed_radius_m)
    chk(abs(prof.collision_radius_m - 0.370135) <= 5e-6,
        'A 闸门 collision_radius_m ≈ 0.370135（取证口径）',
        '%.9f' % prof.collision_radius_m)
    chk(prof.body_width_m == 0.470,
        'A 闸门 body_width_m 逐位等于 0.470（不是 0.400）',
        '%.17g' % prof.body_width_m)


def test_B_derivation():
    """B 派生自洽：三个 profile 的阈值都能由几何复算出来。"""
    for name in PROFILES:
        p = load_profile(name)
        chk(near(p.pass_width_m, p.body_width_m + 2 * p.side_open_m),
            'B %s pass_width = 宽 + 2×side_open' % name,
            '%.6f vs %.6f' % (p.pass_width_m,
                              p.body_width_m + 2 * p.side_open_m))
        chk(near(p.narrow_floor_m, p.body_width_m + 2 * p.side_tight_m),
            'B %s narrow_floor = 宽 + 2×side_tight' % name,
            '%.6f vs %.6f' % (p.narrow_floor_m,
                              p.body_width_m + 2 * p.side_tight_m))
        chk(p.narrow_floor_m < p.pass_width_m,
            'B %s narrow_floor < pass_width' % name,
            '%.3f < %.3f' % (p.narrow_floor_m, p.pass_width_m))
        chk(0 < p.inscribed_radius_m <= p.circumscribed_radius_m,
            'B %s 0 < 内切 <= 外接' % name,
            '%.3f / %.3f' % (p.inscribed_radius_m,
                             p.circumscribed_radius_m))
        chk(p.inscribed_radius_exact,
            'B %s 内切半径是精确值（矩形快路径）' % name)
        chk(p.collision_radius_m <= p.circumscribed_radius_m,
            'B %s 碰撞半径 <= 外接半径' % name,
            '%.3f <= %.3f' % (p.collision_radius_m,
                              p.circumscribed_radius_m))


def test_C_single_source():
    """C 单源：profile 与另外三处独立几何定义逐一相等。"""
    p = load_profile('wheeltec_diff')
    coll = read(COLL)
    sup = read(SUP)

    for tag, text in (('collision_probe', coll), ('escape_supervisor', sup)):
        for key, attr in (('body_x_min', 'collision_x_min'),
                          ('body_x_max', 'collision_x_max'),
                          ('body_half_width', 'collision_y_half')):
            v = param_default(text, key)
            chk(v is not None and near(v, getattr(p, attr)),
                'C %s.%s == profile.%s' % (tag, key, attr),
                '%s vs %s' % (v, getattr(p, attr)))

    # patch_sim_time 是 bash 里的字符串常量，必须逐点相等（含顶点顺序）
    ptxt = read(PATCH)
    m = re.search(r"FOOTPRINT='(\[\[.*?\]\])'", ptxt)
    if not chk(m is not None, 'C patch_sim_time 里能找到 FOOTPRINT 常量'):
        return
    txt = m.group(1)
    pair = re.compile(r'\[\s*(-?[0-9.]+)\s*,\s*(-?[0-9.]+)\s*\]')
    fp_patch = [(float(a), float(b)) for a, b in pair.findall(txt)]
    chk(len(fp_patch) == len(p.footprint),
        'C patch_sim_time FOOTPRINT 顶点数一致',
        '%d vs %d' % (len(fp_patch), len(p.footprint)))
    for i, (a, b) in enumerate(zip(fp_patch, p.footprint)):
        chk(near(a[0], b[0]) and near(a[1], b[1]),
            'C patch_sim_time FOOTPRINT 顶点 %d 一致' % i,
            '%s vs %s' % (a, b))

    # launch 里的显式默认必须是 0.0 哨兵（不准再写 0.90 / 0.58）
    lt = read(LAUNCH)
    for name in ('pass_width', 'narrow_floor'):
        v = None
        m = re.search(r"DeclareLaunchArgument\(\s*['\"]%s['\"]\s*,"
                      r"\s*default_value\s*=\s*['\"]([^'\"]+)['\"]"
                      % re.escape(name), lt)
        if m:
            v = float(m.group(1))
        chk(v == 0.0,
            'C launch %s 默认是 0.0 哨兵（阈值不在此处硬编码）' % name,
            'got=%r' % (v,))


def test_D_two_radii():
    """D 口径：两个半径故意不同，不许被"统一"。"""
    p = load_profile('wheeltec_diff')
    chk(not near(p.collision_radius_m, p.circumscribed_radius_m, 1e-6),
        'D collision_radius(0.370) 与 circumscribed(0.397) 故意不同',
        '%.6f vs %.6f' % (p.collision_radius_m, p.circumscribed_radius_m))
    gap = p.circumscribed_radius_m - p.collision_radius_m
    chk(0.02 < gap < 0.04,
        'D 两者相差约 2.7 cm（planning footprint 的前后余量）',
        'gap=%.4f' % gap)
    chk(not near(p.inscribed_radius_m, p.min_origin_edge_m, 1e-6),
        'D inscribed(0.235) 与 min_origin_edge(0.18) 故意不同',
        '%.3f vs %.3f' % (p.inscribed_radius_m, p.min_origin_edge_m))
    chk(near(p.min_origin_edge_m, 0.18) and near(p.inscribed_radius_m, 0.235),
        'D min_origin_edge==0.18 / inscribed==0.235（Nav2 语义 vs 几何内切）')


def test_E_capabilities():
    """E 能力：不能用机型名分支，且阿克曼的扫掠半径不被低估。"""
    diff = load_profile('wheeltec_diff')
    omni = load_profile('omni_mecanum')
    akm = load_profile('ackermann_small')

    chk(diff.can_rotate_in_place and near(diff.turn_corridor_m, 0.0),
        'E 差速可原地转 ⇒ turn_corridor == 0')
    chk(near(omni.turn_corridor_m, 0.0) and omni.can_move_lateral,
        'E 全向可原地转 + 可横移')
    chk(not akm.can_rotate_in_place and akm.turn_corridor_m > 0.0,
        'E 阿克曼不可原地转 ⇒ turn_corridor > 0',
        '%.4f' % akm.turn_corridor_m)
    chk(akm.reverse_allowed,
        'E 阿克曼必须允许倒车（否则等于禁掉头，S5 §6 R3）')

    # R_outer 不许被低估：外廓多边形包住碰撞盒 ⇒ 绕转弯圆心扫过的半径
    # 必须 >= 碰撞盒扫过的半径。两个来源独立，构成真实交叉验证。
    r = akm.min_turning_radius_m
    box = [(akm.collision_x_min, -akm.collision_y_half),
           (akm.collision_x_min, akm.collision_y_half),
           (akm.collision_x_max, akm.collision_y_half),
           (akm.collision_x_max, -akm.collision_y_half)]
    r_box = max(max(math.hypot(x, y - r) for x, y in box),
                max(math.hypot(x, y + r) for x, y in box))
    chk(akm.turn_outer_radius_m >= r_box - TOL,
        'E 阿克曼 turn_outer_radius 不低估碰撞盒扫掠半径',
        '%.6f >= %.6f' % (akm.turn_outer_radius_m, r_box))
    # 与 S5 §3 M1 的闭式对拍。注意方向：闭式用的是 collision_geometry 的 x 范围，
    # 而实现扫的是 footprint（规划口径，比碰撞盒前后各留约 2 cm 余量）。
    # 所以 `实现 >= 闭式` 才是"不被低估"——闭式在 footprint 外扩后会略微偏小。
    closed = math.hypot(r + akm.body_width_m / 2,
                        max(abs(akm.collision_x_min),
                            abs(akm.collision_x_max)))
    chk(akm.turn_outer_radius_m >= closed - TOL,
        'E 阿克曼 R_outer 不低估 S5 闭式（方向：宁大不小）',
        '%.6f >= %.6f' % (akm.turn_outer_radius_m, closed))
    chk(near(akm.turn_corridor_m, 2 * akm.turn_outer_radius_m,
             1e-9),
        'E turn_corridor == 2 × R_outer（保守上界，S5 §3 M1）')


def test_F_schema_guard():
    """F 结构：派生量不许写进 YAML；坏配置必须报错。"""
    for name in PROFILES:
        p = load_profile(name)
        for k in ('pass_width_m', 'narrow_floor_m', 'turn_corridor_m',
                  'body_width_m', 'circumscribed_radius_m'):
            chk(k not in p.raw,
                'F %s 的 YAML 里没有派生量 %s' % (name, k))

    bad = (
        ({'profile_name': 'x', 'kinematics': 'differential',
          'footprint': [[0, 0], [0, 1], [1, 1], [1, 0]],
          'collision_geometry': {'x_min': -0.1, 'x_max': 0.1, 'y_half': 0.1},
          'capabilities': {'in_place_rotation': True},
          'clearance_budget': {'side_open_m': 0.2, 'side_tight_m': 0.1,
                               'goal_disc_m': 0.3},
          'pass_width_m': 0.9}, '派生量出现在配置里'),
        ({'profile_name': 'x', 'kinematics': 'ackermann',
          'footprint': [[0, 0], [0, 1], [1, 1], [1, 0]],
          'collision_geometry': {'x_min': -0.1, 'x_max': 0.1, 'y_half': 0.1},
          'capabilities': {'in_place_rotation': False},
          'clearance_budget': {'side_open_m': 0.2, 'side_tight_m': 0.1,
                               'goal_disc_m': 0.3}}, '不可原地转却没给转弯半径'),
        ({'profile_name': 'x', 'kinematics': 'differential',
          'footprint': [[0, 0], [0, 1], [1, 1], [1, 0]],
          'collision_geometry': {'x_min': -0.1, 'x_max': 0.1, 'y_half': 0.1},
          'capabilities': {'in_place_rotation': True},
          'clearance_budget': {'side_open_m': 0.1, 'side_tight_m': 0.2,
                               'goal_disc_m': 0.3}}, 'side_tight > side_open'),
    )
    from robot_model import RobotProfile
    for data, why in bad:
        try:
            RobotProfile(data, source='<合成>')
        except ProfileError:
            chk(True, 'F 拒绝坏配置：%s' % why)
        else:
            chk(False, 'F 拒绝坏配置：%s' % why, '没有报错！')

    chk(len(available_profiles()) >= len(PROFILES),
        'F 三个 profile 都能被搜到', str(available_profiles()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-v', '--verbose', action='store_true')
    a = ap.parse_args()

    for fn in (test_A_gate, test_B_derivation, test_C_single_source,
               test_D_two_radii, test_E_capabilities, test_F_schema_guard):
        try:
            fn()
        except Exception as exc:                       # noqa: BLE001
            chk(False, '%s 抛异常' % fn.__name__, repr(exc))

    if a.verbose:
        for label, detail in _ok:
            print('  ✓ %s  %s' % (label, detail))
    for label, detail in _bad:
        print('  ✗ %s  %s' % (label, detail))
    print('test_robot_profile: %d 通过, %d 失败' % (len(_ok), len(_bad)))
    return 0 if not _bad else 1


if __name__ == '__main__':
    sys.exit(main())
