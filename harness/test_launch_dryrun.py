#!/usr/bin/env python3
"""test_launch_dryrun.py —— 干跑构造 sim_explore.launch.py（不启动任何进程）。

来历：2026-10-23 给 launch 加延迟参数时写错 import（PythonExpression 未导入），
`python3 -m py_compile` 只查语法、查不出 NameError，结果两轮起栈直接失败。
这里真正执行 generate_launch_description()，把这类"运行时 NameError/类型错"堵在 smoke 阶段。
"""
import importlib.util
import os
import subprocess
import sys

WS = '/home/kbf/stemm_exploration_sim_ws'
LAUNCHES = [
    (WS + '/src/stemm_cartographer_exploration_sim/launch/sim_explore.launch.py', 10),
    (WS + '/src/stemm_cartographer_exploration_sim/launch/nav_verify.launch.py', 5),
]
P = LAUNCHES[0][0]

# launch 里要用 ament_index_python 查 wheeltec/stemm 的 share 目录，缺任何一个工作区都会
# PackageNotFoundError。这里在环境不全时**带着四个工作区重跑自己**（而不是假 PASS）。
_NEED = ('wheeltec_two_wheel_model', 'stemm_cartographer_exploration')
if os.environ.get('_DRYRUN_REEXEC') != '1':
    prefix = os.environ.get('AMENT_PREFIX_PATH', '')
    if not all(n.replace('_', '_') and n.split('_')[0] in prefix.lower() for n in _NEED):
        setups = ['/opt/ros/humble/setup.bash',
                  '/home/kbf/stemm_sim_ws/install/setup.bash',
                  '/home/kbf/stemm_explore_ws/install/setup.bash',
                  WS + '/install/setup.bash']
        cmd = 'set +u; ' + '; '.join('source %s >/dev/null 2>&1' % f for f in setups) + \
              '; _DRYRUN_REEXEC=1 python3 ' + __file__
        raise SystemExit(subprocess.call(['bash', '-c', cmd]))


def main():
    rc = 0
    for path, min_actions in LAUNCHES:
        if not os.path.exists(path):
            print(f'✗ 缺少 launch：{path}')
            rc = 1
            continue
        spec = importlib.util.spec_from_file_location(
            os.path.basename(path).replace('.', '_'), path)
        mod = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(mod)
            ld = mod.generate_launch_description()
        except Exception as e:                  # noqa: BLE001
            print(f'✗ {os.path.basename(path)} 构造失败：{type(e).__name__}: {e}')
            rc = 1
            continue
        n = len(getattr(ld, 'entities', []))
        ok = n > min_actions
        print(f'{"✓" if ok else "✗"} {os.path.basename(path)} 构造成功，动作数 = {n}')
        if not ok:
            rc = 1
    return rc


if __name__ == '__main__':
    sys.exit(main())
