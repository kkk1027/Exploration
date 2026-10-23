#!/usr/bin/env python3
"""test_launch_dryrun.py —— 干跑构造 sim_explore.launch.py（不启动任何进程）。

来历：2026-10-23 给 launch 加延迟参数时写错 import（PythonExpression 未导入），
`python3 -m py_compile` 只查语法、查不出 NameError，结果两轮起栈直接失败。
这里真正执行 generate_launch_description()，把这类"运行时 NameError/类型错"堵在 smoke 阶段。
"""
import importlib.util
import sys

P = ('/home/kbf/stemm_exploration_sim_ws/src/stemm_cartographer_exploration_sim/'
     'launch/sim_explore.launch.py')


def main():
    spec = importlib.util.spec_from_file_location('sim_explore_launch', P)
    mod = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(mod)
        ld = mod.generate_launch_description()
    except Exception as e:                      # noqa: BLE001
        print(f'✗ launch 构造失败：{type(e).__name__}: {e}')
        return 1
    n = len(getattr(ld, 'entities', []))
    print(f'✓ launch 构造成功，动作数 = {n}')
    return 0 if n > 10 else 1


if __name__ == '__main__':
    sys.exit(main())
