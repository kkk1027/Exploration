#!/usr/bin/env python3
"""probe_profile.py —— 打印"车体几何/通行阈值"的现状快照（S5 Step 0 交付物）。

用途（两件事，顺序不能反）
--------------------------
1. **Step 0 基线口径冻结**：把改造前的实际值原样打出来，存进 runs/。
   之后任何 A/B 都要以"这份快照"为对照，不能凭记忆。
2. **Step 1 等价性闸门**：M1 引入 `RobotProfile` 后，派生量必须与本快照
   **0 容差**相等。不相等就不许进 Step 2 —— 这是整个 S5 的地基（S5 §6 R1）。

设计原则：**不重复定义数字**
--------------------------
本脚本不写任何常量，全部从真实源码/配置里**读出来**再算。理由：
如果这里也硬编码一份，就把"车体几何有 4 处独立定义"变成 5 处，
恰好违反 S5 §2 原则 6（单一事实源）。

读到的东西
----------
    sim_explore.launch.py              pass_width / narrow_floor / corridor_width
    frontier_arbiter                   代码默认值与使用点计数
    stemm_safe_nav2_params.yaml        Nav2 footprint（yaml 原始值）
    patch_sim_time.py                  Nav2 footprint（运行期注入值）
    collision_probe                    body_x_min/x_max/body_half_width（取证口径）
    escape_supervisor                  body_x_min/x_max/body_half_width/laser_x
    cartographer_manager               goal_clearance / return_home_goal_clearance

用法
----
    python3 harness/probe_profile.py            # 打印
    python3 harness/probe_profile.py --freeze   # 写入 runs/profile_baseline.json
    python3 harness/probe_profile.py --check    # Step 1 之后：与快照 0 容差比对
"""
import argparse
import ast
import json
import math
import os
import re
import sys

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WS = '/home/kbf/wheeltec_ros2'
SIM_PKG = os.path.join(BASE, 'src', 'stemm_cartographer_exploration_sim')
ARB = os.path.join(SIM_PKG, 'scripts', 'frontier_arbiter')
LAUNCH = os.path.join(SIM_PKG, 'launch', 'sim_explore.launch.py')
COLL = os.path.join(SIM_PKG, 'scripts', 'collision_probe')
SUP = os.path.join(SIM_PKG, 'scripts', 'escape_supervisor')
NAVYAML = os.path.join(WS, 'src', 'stemm_cartographer_exploration',
                       'config', 'stemm_safe_nav2_params.yaml')
CARTMGR = os.path.join(WS, 'src', 'stemm_cartographer_exploration',
                       'stemm_cartographer_exploration',
                       'cartographer_manager.py')
PATCH = os.path.join(SIM_PKG, 'scripts', 'patch_sim_time')
if not os.path.exists(PATCH):
    # 兼容带 .py 后缀的变体（实测本仓库是无后缀可执行脚本）
    PATCH = PATCH + '.py'

SNAP = os.path.join(BASE, 'runs', 'profile_baseline.json')
MGR = os.path.join(WS, 'src', 'stemm_nav2_manager', 'stemm_nav2_manager',
                   'manager.py')
SCRIPTS = os.path.join(SIM_PKG, 'scripts')


def load_robot_profile(name='wheeltec_diff'):
    """加载 RobotProfile（S5 M1）。失败返回 None，不报错。

    本脚本要能区分"改造前（无 profile）"与"改造后（profile 驱动）"两种情况，
    所以这里只尝试、不抛异常。
    """
    try:
        if SCRIPTS not in sys.path:
            sys.path.insert(0, SCRIPTS)
        from robot_model import load_profile
        return load_profile(name)
    except Exception:                                  # noqa: BLE001
        return None


def derived_or(explicit, prof, attr):
    """S5 M1 的取值口径：显式值 > 0 时用它，否则用 profile 派生值。

    与 `frontier_arbiter._metric_threshold` 必须一致（同一套语义）：
    launch/节点参数里的 0.0 是"未指定，用 profile"的哨兵值，不是真的 0 米。
    """
    if explicit is not None and explicit > 0.0:
        return explicit
    if prof is not None:
        return round(float(getattr(prof, attr)), 9)
    return explicit


def read(p):
    if not os.path.exists(p):
        return ''
    return open(p, encoding='utf-8', errors='replace').read()


def launch_default(text, name):
    """从 DeclareLaunchArgument('name', default_value='0.90' 里取值。"""
    m = re.search(r"DeclareLaunchArgument\(\s*['\"]%s['\"]\s*,"
                  r"\s*default_value\s*=\s*['\"]([^'\"]+)['\"]" % re.escape(name),
                  text)
    return float(m.group(1)) if m else None


def param_default(text, name, cast=float):
    """从 p('name', 0.34) 里取第二个实参。"""
    m = re.search(r"p\(\s*['\"]%s['\"]\s*,\s*([-0-9.eE]+)\s*\)" % re.escape(name),
                  text)
    if not m:
        return None
    try:
        return cast(m.group(1))
    except Exception:                                  # noqa: BLE001
        return None


def declare_default(text, name, cast=float):
    """从 declare_parameter('name', 0.35) 里取值。"""
    m = re.search(r"declare_parameter\(\s*['\"]%s['\"]\s*,\s*([-0-9.eE]+)\s*\)"
                  % re.escape(name), text)
    if not m:
        return None
    try:
        return cast(m.group(1))
    except Exception:                                  # noqa: BLE001
        return None


def parse_footprint(text, key):
    """从 yaml 里找 `<key>: "[[..],[..]]"`，返回 [[x,y],...]。"""
    m = re.search(r'^\s*%s:\s*"(\[\[.*?\]\])"' % re.escape(key), text, re.M)
    if not m:
        return None
    try:
        return ast.literal_eval(m.group(1))
    except Exception:                                  # noqa: BLE001
        return None


def poly_derived(fp):
    """由多边形求 body_width / body_length / 内切 / 外接半径。"""
    if not fp:
        return {}
    xs = [p[0] for p in fp]
    ys = [p[1] for p in fp]
    w = max(ys) - min(ys)
    ln = max(xs) - min(xs)
    insc = min(abs(min(xs)), abs(max(xs)), abs(min(ys)), abs(max(ys)))
    circ = max(math.hypot(x, y) for x, y in fp)
    return {'body_width_m': round(w, 6), 'body_length_m': round(ln, 6),
            'inscribed_radius_m': round(insc, 6),
            'circumscribed_radius_m': round(circ, 6)}


def count_uses(text, name):
    """两个口径都给：属性引用数（≈判据位置）与行数（≈grep -c）。

    S5 §1.1 表里的 "pass_width 19 处、narrow_floor 11 处" 是 `grep -c` 的**行数**；
    而真正要清的"判据位置"是属性解引用次数。两者不同，不能混用。
    """
    if not text:
        return None, None
    refs = len(re.findall(r'\bself\.%s\b' % re.escape(name), text))
    lines = sum(1 for ln in text.splitlines() if name in ln)
    return refs, lines


def collect():
    launch = read(LAUNCH)
    arb = read(ARB)
    coll = read(COLL)
    sup = read(SUP)
    yaml = read(NAVYAML)
    cart = read(CARTMGR)
    patch = read(PATCH)

    out = {}

    # ---- 0) S5 M1：先加载 profile，后面所有米制阈值都按"显式 > profile"解析 ----
    prof = load_robot_profile()
    out['robot_profile__loaded'] = bool(prof is not None)
    if prof is not None:
        out['robot_profile__name'] = prof.profile_name
        d = prof.as_dict()
        for k in ('body_width_m', 'inscribed_radius_m', 'circumscribed_radius_m',
                  'collision_radius_m', 'min_origin_edge_m', 'turn_corridor_m'):
            out['profile__' + k] = d[k]

    # ---- 1) 通行阈值（launch 声明值 + 节点代码默认值） ----
    # 0.0 是"未指定，用 profile 派生值"的哨兵（S5 M1），不是真的 0 米。
    # 这里同时保留原始值与生效值，避免"看起来只有 0"这种误读。
    out['pass_width_m__raw'] = launch_default(launch, 'pass_width')
    out['narrow_floor_m__raw'] = launch_default(launch, 'narrow_floor')
    out['pass_width_m'] = derived_or(out['pass_width_m__raw'], prof,
                                     'pass_width_m')
    out['narrow_floor_m'] = derived_or(out['narrow_floor_m__raw'], prof,
                                       'narrow_floor_m')
    out['corridor_width_m'] = launch_default(launch, 'corridor_width')
    out['arb_pass_width_m'] = derived_or(
        param_default(arb, 'pass_width_m'), prof, 'pass_width_m')
    out['arb_narrow_floor_m'] = derived_or(
        param_default(arb, 'narrow_floor_m'), prof, 'narrow_floor_m')
    out['arb_corridor_hard_m'] = param_default(arb, 'corridor_hard_m')
    out['arb_max_candidates'] = param_default(arb, 'max_candidates', int)
    out['arb_max_candidates_doc'] = None      # 文件头注释里写的默认值，见下
    m = re.search(r'max_candidates\s+每轮校验的候选数（默认\s*(\d+)）', arb)
    if m:
        out['arb_max_candidates_doc'] = int(m.group(1))
    out['arb_pass_width_uses'] = count_uses(arb, 'pass_width')[0]
    out['arb_pass_width_lines'] = count_uses(arb, 'pass_width')[1]
    out['arb_narrow_floor_uses'] = count_uses(arb, 'narrow_floor')[0]
    out['arb_narrow_floor_lines'] = count_uses(arb, 'narrow_floor')[1]

    # ---- 2) Nav2 footprint（两个来源，口径不同是故意的） ----
    fp_yaml = parse_footprint(yaml, 'footprint')
    out['nav2_footprint_yaml'] = fp_yaml
    out['nav2_footprint_yaml_derived'] = poly_derived(fp_yaml)
    m = re.search(r"FOOTPRINT\s*=\s*'?(\[\[.*?\]\])'?", patch, re.S)
    fp_patch = None
    if m:
        try:
            fp_patch = ast.literal_eval(m.group(1))
        except Exception:                              # noqa: BLE001
            fp_patch = None
    out['nav2_footprint_runtime'] = fp_patch
    out['nav2_footprint_runtime_derived'] = poly_derived(fp_patch)

    # ---- 3) 车体几何（取证口径 / 脱困口径） ----
    geo = {}
    for src, text in (('collision_probe', coll), ('escape_supervisor', sup)):
        geo[src] = {
            'body_x_min': param_default(text, 'body_x_min'),
            'body_x_max': param_default(text, 'body_x_max'),
            'body_half_width': param_default(text, 'body_half_width'),
            'laser_x': param_default(text, 'laser_x'),
        }
        g = geo[src]
        if g['body_x_min'] is not None and g['body_x_max'] is not None \
                and g['body_half_width'] is not None:
            g['circumscribed_radius_m'] = round(
                math.hypot(max(abs(g['body_x_min']), abs(g['body_x_max'])),
                           g['body_half_width']), 6)
        if g['laser_x'] is not None and g['body_x_max'] is not None:
            g['front_offset_m'] = round(g['body_x_max'] - g['laser_x'], 6)
    out['body_geometry'] = geo

    # ---- 4) 车端净空口径 ----
    # 注意：探索目标用的是 manager 里的 goal_clearance（0.38），
    # 返航目标另有一份 return_home_goal_clearance（0.35）——两个数不同是故意的。
    out['goal_clearance_m'] = declare_default(read(MGR), 'goal_clearance')
    out['return_home_goal_clearance_m'] = declare_default(
        cart, 'return_home_goal_clearance')

    # ---- 5) 派生量：用"车宽 + 两侧余量"反推，验证等价性 ----
    # ★ 2026-11-14 实测发现：body_width 有两个候选值，两者都能反推出同样的阈值
    #   pass_width 0.90 = 0.40 + 2×0.25 = 0.47 + 2×0.215
    #   narrow_floor 0.58 = 0.40 + 2×0.09 = 0.47 + 2×0.055
    # ⇒ 单看阈值无法判定哪一个才是"真车宽"，正是 S5 §1.1 说的口径矛盾。
    # 真值来源只能是 URDF：x∈[-0.16,0.29]、|y|≤0.23 ⇒ 宽 0.46（含轮 0.47）。
    # 所以 **yaml footprint 的 0.40 是偏小的**，launch 注释里的"车宽 0.40"也是错的。
    bw_yaml = out['nav2_footprint_yaml_derived'].get('body_width_m')
    bw_rt = (out['nav2_footprint_runtime_derived'] or {}).get('body_width_m')
    for tag, bw in (('yaml', bw_yaml), ('runtime', bw_rt)):
        if bw and out['pass_width_m']:
            out['side_open_m__' + tag] = round(
                (out['pass_width_m'] - bw) / 2, 6)
        if bw and out['narrow_floor_m']:
            out['side_tight_m__' + tag] = round(
                (out['narrow_floor_m'] - bw) / 2, 6)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--freeze', action='store_true', help='写入 runs/profile_baseline.json')
    ap.add_argument('--check', action='store_true', help='与快照 0 容差比对')
    ap.add_argument('--json', action='store_true', help='只输出 JSON')
    a = ap.parse_args()

    cur = collect()

    if a.check:
        if not os.path.exists(SNAP):
            print('✗ 找不到快照 %s，先跑 --freeze' % SNAP, file=sys.stderr)
            return 3
        base = json.load(open(SNAP, encoding='utf-8'))
        fails = []
        for k, v in base.items():
            if isinstance(v, float):
                c = cur.get(k)
                if not isinstance(c, (int, float)) or abs(c - v) > 1e-9:
                    fails.append('%s: 快照=%s 现在=%s' % (k, v, c))
        if fails:
            print('✗ 等价性破坏（%d 项）：' % len(fails))
            for f in fails:
                print('   ', f)
            return 1
        print('✓ 等价性通过：%d 个标量派生量与快照 0 容差相等'
              % sum(1 for v in base.values() if isinstance(v, float)))
        return 0

    if a.json:
        print(json.dumps(cur, ensure_ascii=False, indent=1))
        return 0

    def show(label, key, unit=''):
        v = cur.get(key)
        print('  %-38s %s%s' % (label, '—' if v is None else v, unit))

    print('[车体几何 / 通行阈值 现状快照]  (S5 Step 0)')
    print('  来源均为源码/配置实读，本脚本不硬编码任何常量')
    print()
    print('-- 通行阈值（launch 声明） --')
    print('  %-38s raw=%s → 生效=%s'
          % ('pass_width_m', cur.get('pass_width_m__raw'), cur.get('pass_width_m')))
    print('  %-38s raw=%s → 生效=%s'
          % ('narrow_floor_m', cur.get('narrow_floor_m__raw'),
             cur.get('narrow_floor_m')))
    print('  0.0 是"未指定→用 profile 派生"的哨兵值（S5 M1），不是真的 0 米')
    show('corridor_width_m', 'corridor_width_m')
    print('-- 通行阈值（frontier_arbiter 代码默认） --')
    show('pass_width_m（节点参数，生效值）', 'arb_pass_width_m')
    show('narrow_floor_m（节点参数，生效值）', 'arb_narrow_floor_m')
    show('pass_width_m 参数原始默认', 'pass_width_m__raw')
    show('narrow_floor_m 参数原始默认', 'narrow_floor_m__raw')
    show('corridor_hard_m', 'arb_corridor_hard_m')
    show('max_candidates（代码）', 'arb_max_candidates')
    show('max_candidates（文件头注释）', 'arb_max_candidates_doc')
    show('pass_width 属性引用数', 'arb_pass_width_uses')
    show('pass_width 行数(grep -c)', 'arb_pass_width_lines')
    show('narrow_floor 属性引用数', 'arb_narrow_floor_uses')
    show('narrow_floor 行数(grep -c)', 'arb_narrow_floor_lines')
    print('-- Nav2 footprint --')
    print('  yaml 原始值        %s' % (cur.get('nav2_footprint_yaml'),))
    d = cur.get('nav2_footprint_yaml_derived') or {}
    print('     → 宽 %.3f  长 %.3f  内切 %.3f  外接 %.3f'
          % (d.get('body_width_m', 0), d.get('body_length_m', 0),
             d.get('inscribed_radius_m', 0), d.get('circumscribed_radius_m', 0)))
    print('  运行期注入值       %s' % (cur.get('nav2_footprint_runtime'),))
    d2 = cur.get('nav2_footprint_runtime_derived') or {}
    if d2:
        print('     → 宽 %.3f  长 %.3f  内切 %.3f  外接 %.3f'
              % (d2.get('body_width_m', 0), d2.get('body_length_m', 0),
                 d2.get('inscribed_radius_m', 0),
                 d2.get('circumscribed_radius_m', 0)))
    print('-- 车体几何（各口径，口径不同是故意的） --')
    for src, g in (cur.get('body_geometry') or {}).items():
        print('  %-20s x_min=%-7s x_max=%-7s y_half=%-6s laser_x=%-6s '
              '外接=%-7s 前沿偏移=%s'
              % (src, g.get('body_x_min'), g.get('body_x_max'),
                 g.get('body_half_width'), g.get('laser_x'),
                 g.get('circumscribed_radius_m'), g.get('front_offset_m')))
    print('-- 车端净空口径 --')
    show('goal_clearance_m', 'goal_clearance_m')
    show('return_home_goal_clearance_m', 'return_home_goal_clearance_m')
    if cur.get('robot_profile__loaded'):
        print('-- S5 M1 RobotProfile 派生量（阈值唯一来源） --')
        print('  profile = %s' % cur.get('robot_profile__name'))
        for k in ('body_width_m', 'inscribed_radius_m', 'circumscribed_radius_m',
                  'collision_radius_m', 'min_origin_edge_m', 'turn_corridor_m'):
            show(k, 'profile__' + k)
    else:
        print('-- S5 M1 RobotProfile --')
        print('  未加载（改造前状态，或 profile 目录缺失）')
    print('-- 反推的两侧余量（两种 body_width 都算一遍，逐一看等价性） --')
    show('side_open_m  (pass_width - 0.40)/2', 'side_open_m__yaml')
    show('side_tight_m (narrow_floor - 0.40)/2', 'side_tight_m__yaml')
    show('side_open_m  (pass_width - 0.47)/2', 'side_open_m__runtime')
    show('side_tight_m (narrow_floor - 0.47)/2', 'side_tight_m__runtime')

    if a.freeze:
        os.makedirs(os.path.dirname(SNAP), exist_ok=True)
        json.dump(cur, open(SNAP, 'w', encoding='utf-8'),
                  ensure_ascii=False, indent=1)
        print()
        print('  ✓ 已冻结到 %s' % SNAP)
    return 0


if __name__ == '__main__':
    sys.exit(main())
