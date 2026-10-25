#!/usr/bin/env python3
"""test_param_types.py —— launch 传参类型 vs 节点 declare 类型 的一致性检查。

来历（2026-10-24，V2-③ 第一次跑就崩）：
  launch 里 `veto:=true` 会被 launch_ros 归一化成 **bool**，
  而仲裁节点用字符串默认值 `p('veto_enabled', 'false')` 声明，
  rclpy 直接抛 InvalidParameterTypeException —— 仲裁节点当场死掉，整轮作废，
  但 `python3 -m py_compile` 和"干跑 launch"都查不出来（它们不构造节点）。

本检查把这条不变量变成静态断言：
  1. 从 launch 里找出传给各节点的 `LaunchConfiguration('x')` 参数；
  2. 用 DeclareLaunchArgument 的默认值按 launch_ros 的规则推断类型
     （'true'/'false'→bool，整数串→int，小数串→float，其它→str）；
  3. 到对应脚本里找 `declare_parameter` 的默认值字面量，比对类型。
类型不一致就 FAIL，并指名道姓说是哪个节点哪个参数。
"""
import ast
import os
import re
import sys

WS = '/home/kbf/stemm_exploration_sim_ws'
LAUNCH = os.path.join(WS, 'src/stemm_cartographer_exploration_sim/launch/sim_explore.launch.py')
SCRIPTS = os.path.join(WS, 'src/stemm_cartographer_exploration_sim/scripts')


def infer_type(text):
    """按 launch_ros.utilities.normalize_parameter_value 的规则推断字符串的类型。"""
    if text is None:
        return None
    t = str(text).strip()
    low = t.lower()
    if low in ('true', 'false'):
        return bool
    try:
        int(t, 10)
        return int
    except ValueError:
        pass
    try:
        float(t)
        return float
    except ValueError:
        pass
    return str


def literal_type(node):
    if isinstance(node, ast.Constant):
        return type(node.value)
    return None


def launch_args(tree):
    """DeclareLaunchArgument('x', default_value='...') → {pkg参数名: 类型}"""
    out = {}
    for n in ast.walk(tree):
        if isinstance(n, ast.Call) and getattr(n.func, 'id', None) == 'DeclareLaunchArgument':
            if not n.args or not isinstance(n.args[0], ast.Constant):
                continue
            name = n.args[0].value
            dv = None
            for kw in n.keywords:
                if kw.arg == 'default_value' and isinstance(kw.value, ast.Constant):
                    dv = kw.value.value
            out[name] = infer_type(dv)
    return out


def node_params(tree, argtypes):
    """收集 (节点名, 参数名, 期望类型)：只看 value 是 LaunchConfiguration 的条目。"""
    res = []
    for n in ast.walk(tree):
        if not (isinstance(n, ast.Call) and getattr(n.func, 'id', None) == 'Node'):
            continue
        name = '?'
        for kw in n.keywords:
            if kw.arg == 'name' and isinstance(kw.value, ast.Constant):
                name = kw.value.value
        for kw in n.keywords:
            if kw.arg != 'parameters' or not isinstance(kw.value, ast.List):
                continue
            for el in kw.value.elts:
                if not isinstance(el, ast.Dict):
                    continue
                for k, v in zip(el.keys, el.values):
                    if not isinstance(k, ast.Constant):
                        continue
                    if (isinstance(v, ast.Call) and getattr(v.func, 'id', None) == 'LaunchConfiguration'
                            and v.args and isinstance(v.args[0], ast.Constant)):
                        lname = v.args[0].value
                        res.append((name, k.value, argtypes.get(lname)))
    return res


def declared(script):
    """脚本里 declare_parameter 的默认值类型：{参数名: 类型}

    注意：仲裁节点用的是别名 `p = self.declare_parameter`，所以除了直接的
    `self.declare_parameter(...)`，还要认所有"被赋值为 declare_parameter 的局部名"。
    """
    path = os.path.join(SCRIPTS, script)
    if not os.path.exists(path):
        return None
    tree = ast.parse(open(path, encoding='utf-8').read())
    aliases = {'declare_parameter'}
    for n in ast.walk(tree):
        if isinstance(n, ast.Assign) and isinstance(n.value, ast.Attribute) \
                and n.value.attr == 'declare_parameter':
            for t in n.targets:
                if isinstance(t, ast.Name):
                    aliases.add(t.id)
    out = {}
    for n in ast.walk(tree):
        if not (isinstance(n, ast.Call) and len(n.args) >= 2
                and isinstance(n.args[0], ast.Constant)
                and isinstance(n.args[1], ast.Constant)):
            continue
        fn = n.func
        hit = ((isinstance(fn, ast.Attribute) and fn.attr in aliases)
               or (isinstance(fn, ast.Name) and fn.id in aliases))
        if hit:
            out[n.args[0].value] = type(n.args[1].value)
    return out


# 节点名 → 脚本（只列会用 launch 传参的）
NODE_SCRIPT = {
    'frontier_arbiter': 'frontier_arbiter',
    'explore_watchdog': 'explore_watchdog',
    'collision_probe': 'collision_probe',
    'escape_supervisor': 'escape_supervisor',
    'vision_obstacle_grid': 'vision_obstacle_grid',
    'sim_session_driver': 'session_driver',
}


def main():
    tree = ast.parse(open(LAUNCH, encoding='utf-8').read())
    argtypes = launch_args(tree)
    params = node_params(tree, argtypes)
    bad = []
    checked = 0
    for node, pname, ptype in params:
        script = NODE_SCRIPT.get(node)
        if script is None:
            continue
        dec = declared(script)
        if dec is None or pname not in dec:
            continue                      # 节点没声明（ros2 会在运行时忽略）→ 不判
        d = dec[pname]
        if ptype is None:
            bad.append((node, pname, 'launch 默认值缺失', str(d)))
            continue
        checked += 1
        if ptype is not d:
            bad.append((node, pname, ptype.__name__, d.__name__))
    # 反向检查：声明了却**从未被 LaunchConfiguration 引用**的 launch 参数 = 空旋钮。
    # 今天抓到的第三个静默失效：corridor_width 声明了、也一直从 round.sh 传进来，
    # 但 launch 里没有任何地方引用它 → CORRIDOR_W=0.85 实测日志仍打 1.05。
    # 注意只查"引用"，不查引用到了哪个位置（condition/remappings/arguments 都算引用）。
    text = open(LAUNCH, encoding='utf-8').read()
    dead = []
    for name in sorted(argtypes):
        if "LaunchConfiguration('%s')" % name not in text:
            dead.append(name)
    # 第三类陷阱：浮点参数。launch_ros 把 "10" 解析成 **int**，而节点 declare 的
    # 默认值是 30.0(float) → rclpy 抛 InvalidParameterTypeException 打死节点。
    # 实测：VETO_CD=10 直接把仲裁节点打死，前 7 轮全部作废。
    # 规则：凡是 round.sh 传给 launch 的"浮点型 launch 参数"，实参必须是
    # 经 as_float 格式化后的 * _F 变量。
    round_sh = os.path.join(WS, 'harness', 'round.sh')
    rtxt = open(round_sh, encoding='utf-8').read()
    float_args = []
    for n in ast.walk(tree):
        if isinstance(n, ast.Call) and getattr(n.func, 'id', None) == 'DeclareLaunchArgument' \
                and n.args and isinstance(n.args[0], ast.Constant):
            dv = None
            for kw in n.keywords:
                if kw.arg == 'default_value' and isinstance(kw.value, ast.Constant):
                    dv = kw.value.value
            if dv is not None and infer_type(dv) is float:   # infer_type 返回的是类型对象
                float_args.append(n.args[0].value)
    bad_fmt = []
    for name in float_args:
        m = re.search(r'\b%s:=\$(\{[A-Z_]+:-[0-9.]+\}|[A-Z_]+)' % re.escape(name), rtxt)
        if m:
            arg = m.group(1)
            var = re.sub(r'^\$\{|}$', '', arg).split(':-')[0]
            # 已格式化 = 传的是 *_F 变量，或该变量在 round.sh 里用 as_float 赋过值
            ok = arg.endswith('_F}') or arg.endswith('_F') \
                or re.search(r'\b%s=\$\(as_float' % re.escape(var), rtxt) is not None
            if not ok:
                bad_fmt.append((name, arg))
    print(f'  （浮点型 launch 参数 {len(float_args)} 个，其中 round.sh 直接传的 {len(bad_fmt)} 个未格式化）')
    for name, arg in bad_fmt:
        bad.append(('round.sh', name, f'传了未格式化的 {arg}（可能被解析成 int，节点会崩）', '—'))
    print(f'  （另有 {len(dead)} 个 launch 参数声明了但从未被引用）')
    for name in dead:
        bad.append(('launch', name, '已声明但无人引用（空旋钮）', '—'))
    # 第四类：导航参数**不允许按场景/世界切换**（2026-10-25 用户要求）
    rtxt2 = open(os.path.join(WS, 'harness', 'round.sh'), encoding='utf-8').read()
    scoped = []
    for name in ('CORRIDOR_W', 'pass_width', 'MAX_VEL', 'SPEED'):
        if re.search(r'case\s+.*(WORLD|world).*\n(?:.*\n){0,12}.*%s=' % name, rtxt2):
            scoped.append(name)
    if re.search(r'\)\s*CORRIDOR_W=0\.85', rtxt2) or re.search(r'\*house\*\)\s*CORRIDOR_W', rtxt2):
        scoped.append('CORRIDOR_W(世界分支)')
    for name in scoped:
        bad.append(('round.sh', name, '导航参数按世界分支取值（应当与场景无关）', '—'))
    print(f'  （导航参数按世界切换的项：{len(scoped)}）')
    print(f'launch 传参 -> 节点 declare 类型一致性：检查 {checked} 处')
    for node, pname, want, got in bad:
        print(f'  ✗ {node}.{pname}: launch 会传 {want}，节点 declare 的是 {got}')
    if bad:
        print('结果：FAIL（要么运行期抛 InvalidParameterTypeException 打死节点，要么是个空旋钮）')
        return 1
    print('结果：PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
