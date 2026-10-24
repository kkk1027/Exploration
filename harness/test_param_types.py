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
    print(f'launch 传参 -> 节点 declare 类型一致性：检查 {checked} 处')
    for node, pname, want, got in bad:
        print(f'  ✗ {node}.{pname}: launch 会传 {want}，节点 declare 的是 {got}')
    if bad:
        print('结果：FAIL（运行期会抛 InvalidParameterTypeException 打死节点）')
        return 1
    print('结果：PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
