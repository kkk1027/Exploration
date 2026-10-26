#!/usr/bin/env python3
"""test_arbiter_attrs.py —— 静态检查："读 self.X 但从未声明/赋值" 的幽灵属性。

来历（2026-10-26，真机复现）：`frontier_arbiter` 里 `self.corridor_skip_far` 只有**使用**、
从来没有赋值。只要 `path_and_width` 走到"没有净空场"的回退分支就
`AttributeError: 'Arbiter' object has no attribute 'corridor_skip_far'`，
仲裁节点当场死掉 —— 车端再也拿不到新目标，用户看到的现象是"规划路径不刷新、
车长时间卡在窄处"。这类 bug 语法检查、launch 干跑、单元测试全都发现不了。

做法：收集模块里所有 `self.<name>` 的**读取**，减去所有出现过赋值目标的
`self.<name> = ...`（含 `__init__` 里 `self.x: T = ...`、`augmented assign`、
for/with 的目标），差集即"幽灵属性"，直接 FAIL 并列出。
"""
import ast
import sys

SRC = ('/home/kbf/stemm_exploration_sim_ws/src/stemm_cartographer_exploration_sim/'
       'scripts/frontier_arbiter')


def main():
    tree = ast.parse(open(SRC, encoding='utf-8').read())
    assigned, read = set(), set()

    def add_target(t):
        if isinstance(t, ast.Attribute) and isinstance(t.value, ast.Name) \
                and t.value.id == 'self':
            assigned.add(t.attr)

    # 方法名也是属性（def f(self) → self.f 可读）
    methods = {n.name for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)}
    assigned |= methods
    for n in ast.walk(tree):
        if isinstance(n, ast.Attribute) and isinstance(n.value, ast.Name) \
                and n.value.id == 'self':
            if isinstance(n.ctx, ast.Load):
                read.add(n.attr)
        if isinstance(n, ast.Assign):
            for t in n.targets:
                add_target(t)
                if isinstance(t, (ast.Tuple, ast.List)):
                    for e in t.elts:
                        add_target(e)
        if isinstance(n, (ast.AnnAssign, ast.AugAssign)):
            add_target(n.target)
        if isinstance(n, (ast.For, ast.AsyncFor)):
            add_target(n.target)
        if isinstance(n, ast.withitem):
            if n.optional_vars is not None:
                add_target(n.optional_vars)

    # 这些是"允许只读"的：rclpy 基类/RosNode 属性、以及我们明确从外部注入的
    ALLOW = {
        'get_logger', 'create_subscription', 'create_publisher', 'create_client',
        'create_timer', 'declare_parameter', 'get_parameter', 'get_clock', 'destroy_node',
        'get_name', 'get_namespace', 'count_publishers', 'count_subscribers',
        'get_topic_names_and_types', 'get_service_names_and_types', 'deleted', 'handle',
        'context', 'executor', 'set_parameters', 'set_parameters_atomically',
        'cli',      # 历史遗留：get_path 服务客户端从未创建；已加 getattr 空值保护
    }
    ghost = sorted(read - assigned - ALLOW)
    print(f'self.<attr> 读取 {len(read)} 个 / 赋值 {len(assigned)} 个')
    if ghost:
        for g in ghost:
            print(f'  ✗ self.{g} 只读未赋值（运行到该分支就 AttributeError）')
        print('结果：FAIL')
        return 1
    print('结果：PASS（没有幽灵属性）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
