#!/usr/bin/env python3
"""probe_return.py —— 返航窗口 TF 取证（不依赖时钟）：按帧计数增量与 stamp 增量判断是否停滞。"""
import sys, time
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

PAIRS = {('map', 'odom_combined'): 0, ('odom_combined', 'base_footprint'): 0}

class P(Node):
    def __init__(self):
        super().__init__('probe_return')
        self.cnt = {k: 0 for k in PAIRS}
        self.stamp = {k: None for k in PAIRS}
        self.create_subscription(TFMessage, '/tf', self.cb, 200)
    def cb(self, m):
        for tr in m.transforms:
            k = (tr.header.frame_id, tr.child_frame_id)
            if k in self.cnt:
                self.cnt[k] += 1
                self.stamp[k] = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9

rclpy.init(); p = P()
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
t0 = time.monotonic(); prev = dict(p.cnt); prevst = dict(p.stamp)
while time.monotonic() - t0 < dur:
    for _ in range(50):
        rclpy.spin_once(p, timeout_sec=0.1)
    now = time.monotonic() - t0
    parts = []
    for k in p.cnt:
        d = p.cnt[k] - prev[k]
        st = p.stamp[k]
        dst = (st - prevst[k]) if (st is not None and prevst[k] is not None) else float('nan')
        flag = '停滞' if d == 0 else ''
        parts.append(f'{k[0]}->{k[1][:4]}: +{d}帧 Δstamp={dst:+.2f}s {flag}')
        prev[k], prevst[k] = p.cnt[k], st
    print(f'  [t+{now:5.1f}s] ' + ' | '.join(parts), flush=True)
p.destroy_node(); rclpy.shutdown()
