#!/usr/bin/env python3
"""rclpy logger 调用规范的静态检查。

为什么需要它
------------
rclpy 的 `RcutilsLogger.info/warn/error` **只接受一个位置参数**（消息字符串），
不支持 C 风格的位置参数展开：

    # 会抛 TypeError，节点直接 exit 1
    self.get_logger().info('x=%.2f y=%.2f', x, y)

    # 正确
    self.get_logger().info('x=%.2f y=%.2f' % (x, y))

2026-11-15 实测踩到：在 `front_stop_distance` 派生处写了一版 C 风格调用，
`stemm_cartographer_nav2_manager` 在 `__init__` 里抛
`TypeError: RcutilsLogger.info() takes 2 positional arguments but 6 were given`，
进程 exit 1，整轮建图没有 manager —— 但 launch 其余节点照常启动，
日志里只有一行 TypeError，很容易漏看。

注意：`throttle_duration_sec` / `skip_first` 是 rclpy 支持的**关键字**参数，
不算违规（本仓库既有代码在用）。

覆盖文件
--------
  stemm_nav2_manager/stemm_nav2_manager/manager.py
  stemm_cartographer_exploration/stemm_cartographer_exploration/cartographer_manager.py
"""
import ast
import sys

ROOT = '/home/kbf/wheeltec_ros2/src'
FILES = (
    ROOT + '/stemm_nav2_manager/stemm_nav2_manager/manager.py',
    ROOT + '/stemm_cartographer_exploration/stemm_cartographer_exploration/'
           'cartographer_manager.py',
)
LOGGER_METHODS = ('info', 'warn', 'warning', 'error', 'debug', 'fatal')
ALLOWED_KW = {'throttle_duration_sec', 'skip_first'}


def is_logger_call(node):
    """判断 Call 节点是否形如 *.get_logger().info(...)。"""
    if not isinstance(node, ast.Call):
        return None
    fn = node.func
    if not isinstance(fn, ast.Attribute):
        return None
    if fn.attr not in LOGGER_METHODS:
        return None
    inner = fn.value
    if not (isinstance(inner, ast.Call)
            and isinstance(inner.func, ast.Attribute)
            and inner.func.attr == 'get_logger'):
        return None
    return fn.attr


def main():
    bad = 0
    for path in FILES:
        with open(path, encoding='utf-8') as f:
            tree = ast.parse(f.read())
        for node in ast.walk(tree):
            method = is_logger_call(node)
            if method is None:
                continue
            if len(node.args) > 1:
                bad += 1
                print('✗ %s:%d  get_logger().%s() 传了 %d 个位置参数'
                      % (path, node.lineno, method, len(node.args)))
            bad_kw = [k.arg for k in node.keywords
                      if k.arg not in ALLOWED_KW]
            if bad_kw:
                bad += 1
                print('✗ %s:%d  get_logger().%s() 非法关键字参数 %s'
                      % (path, node.lineno, method, bad_kw))

    print()
    if bad:
        print('logger_call_style: %d 处违规 —— 节点会在 __init__ 里 exit 1' % bad)
        print('修法：改成 \'...%.2f...\' % (x,) 的形式')
        return 1
    print('logger_call_style: 0 处违规（%d 个文件已检查）' % len(FILES))
    return 0


if __name__ == '__main__':
    sys.exit(main())
