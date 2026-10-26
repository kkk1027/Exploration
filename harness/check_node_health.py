#!/usr/bin/env python3
"""check_node_health.py —— 从 launch.log 里区分"节点真崩"与"收关噪声"。

背景（2026-10-26 两次踩坑）：
  1. 提交 aeef06c 让 escape_supervisor 在探索期第一次 tick 就 Traceback 退出，
     launch 不为这类节点设硬失败 → 整轮"安静地少一个节点"，digest 照样 PASS；
  2. 第一版体检只看 `Traceback` 字样 → 收关时每个 rclpy 节点都会打
     `rclpy.executors.ExternalShutdownException` + `exit code 1`（正常现象），
     于是每一轮都被误判成"节点猝死"，批次白重跑一轮。

正确判据：Traceback 块的最后一行（异常类型）落在**关停白名单**里 → 忽略；
否则算真崩，并记录节点名 + 异常行。

用法：
  python3 harness/check_node_health.py <launch.log>          # 人读输出，真崩时 exit 1
  python3 harness/check_node_health.py <launch.log> --quiet   # 只 exit code
"""
import re
import sys

# 收关时 rclpy/launch 的正常退出噪声（不是崩溃）
IGNORE = ('ExternalShutdownException', 'KeyboardInterrupt', 'SystemExit',
          'SignalException', 'RCLError', 'rclpy.shutdown')

HEAD = re.compile(r'^\[([^\]]+)\]\s+Traceback \(most recent call last\):')
EXC = re.compile(r'^(?:\[[^\]]+\]\s*)?([A-Za-z_][\w.]*(?:Error|Exception|Interrupt))\b(.*)')


def scan(path):
    """返回 (traceback 块数, [ (node, exc_line), ...真崩 ])"""
    blocks = []
    cur = None
    try:
        fh = open(path, encoding='utf-8', errors='replace')
    except FileNotFoundError:
        return 0, [('(launch.log 不存在)', '')]
    with fh:
        for line in fh:
            line = line.rstrip('\n')
            m = HEAD.match(line)
            if m:
                if cur:
                    blocks.append(cur)
                cur = {'node': m.group(1), 'lines': []}
                continue
            if cur is not None:
                if line.startswith('['):
                    cur['lines'].append(line)
                else:                      # Traceback 结束
                    blocks.append(cur)
                    cur = None
        if cur:
            blocks.append(cur)
    crashes = []
    for b in blocks:
        exc = None
        for line in reversed(b['lines']):
            m = EXC.match(line)
            if m:
                exc = m.group(1)
                break
        if exc is None:                    # 没抓到异常行：保守当作不确定，不报
            continue
        if any(tok in exc for tok in IGNORE):
            continue
        crashes.append((b['node'], exc))
    return len(blocks), crashes


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    path = sys.argv[1]
    quiet = '--quiet' in sys.argv[2:]
    n, crashes = scan(path)
    if not quiet:
        print('traceback_blocks=%d real_crash=%d' % (n, len(crashes)))
        for node, exc in crashes:
            print('  ✗ %s: %s' % (node, exc))
    return 1 if crashes else 0


if __name__ == '__main__':
    sys.exit(main())
