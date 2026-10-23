#!/usr/bin/env python3
"""test_arbiter_static.py —— 仲裁器的静态不变量检查（<1 秒，不依赖 ROS 运行）。

来历：2026-10-23 修走廊判据时把 path_and_width 的返回值从 2 元扩到 4 元，
漏改了三处 `return False, None`，结果真机仿真里 frontier_arbiter 直接崩、
整轮退化成"车端 manager 自己派目标"。这类"签名变了、分支漏改"的 bug 用静态断言兜住。
"""
import ast
import sys

SRC = ('/home/kbf/stemm_exploration_sim_ws/src/stemm_cartographer_exploration_sim/'
       'scripts/frontier_arbiter')
N_RET = 4          # path_and_width 的返回值个数


def main():
    tree = ast.parse(open(SRC, encoding='utf-8').read())
    funcs = {n.name: n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)}
    fn = funcs.get('path_and_width')
    assert fn is not None, '找不到 path_and_width'
    # 只看"属于 path_and_width 本身"的 return（跳过内部嵌套函数 map_val/scan 的返回）
    own = []

    def collect(node):
        for child in ast.iter_child_nodes(node):
            if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)):
                continue                      # 嵌套函数不属于本函数签名
            if isinstance(child, ast.Return):
                own.append(child)
            collect(child)

    collect(fn)
    bad = []
    for node in own:
        if node.value is not None:
            v = node.value
            if isinstance(v, ast.Tuple):
                if len(v.elts) != N_RET:
                    bad.append((node.lineno, f'{len(v.elts)} 元'))
            else:
                bad.append((node.lineno, '非元组'))
    print(f'path_and_width: 检查 {sum(1 for _ in ast.walk(fn))} 个节点，'
          f'返回值应统一为 {N_RET} 元')
    if bad:
        for ln, why in bad:
            print(f'  ✗ 第 {ln} 行 {why}')
        print('结果：FAIL')
        return 1
    # 另一条不变量：调用点必须 4 元解包
    calls = 0
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and isinstance(node.value, ast.Call):
            f = node.value.func
            if isinstance(f, ast.Attribute) and f.attr == 'path_and_width':
                calls += 1
                tgt = node.targets[0]
                assert isinstance(tgt, ast.Tuple) and len(tgt.elts) == N_RET, \
                    f'第 {node.lineno} 行调用点没有 {N_RET} 元解包'
    print(f'调用点: {calls} 处，全部 {N_RET} 元解包')
    print('结果：PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
