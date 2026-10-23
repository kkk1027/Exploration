#!/usr/bin/env python3
"""costmap_dump.py —— 打印视觉栅格 vs 全局代价地图的对照切片（调试用）。

用法: costmap_dump.py [--x 3.54] [--y 1.37] [--half 2.0] [--wait 30]
输出: 全图值分布 + 以 (x,y) 为中心的一片 ASCII（C=致命 i=内切 . =空闲 ?=未知 o=膨胀）
"""
import argparse
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def decode(msg):
    g = np.array(msg.data, dtype=np.int8).astype(np.int16)
    m = g < 0
    if m.any():
        g[m] += 256
    return g.reshape(msg.info.height, msg.info.width)


def scale_of(g):
    """100=致命/99=内切（占用语义，/costmap 默认）；否则 254/253（原始代价）。"""
    if g.size == 0:
        return 254, 253
    return (100, 99) if (not (g == 254).any() and (g == 100).any()) else (254, 253)


class Dump(Node):
    def __init__(self, topic='/global_costmap/costmap'):
        super().__init__('costmap_dump')
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE)
        self.cm = None
        self.vg = None
        self.create_subscription(OccupancyGrid, topic, self.on_cm, q)
        self.create_subscription(OccupancyGrid, '/vision/obstacle_grid', self.on_vg, q)

    def on_cm(self, m):
        self.cm = m

    def on_vg(self, m):
        self.vg = m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, default=3.54)
    ap.add_argument('--y', type=float, default=1.37)
    ap.add_argument('--half', type=float, default=2.0)
    ap.add_argument('--wait', type=float, default=30.0)
    ap.add_argument('--topic', default='/global_costmap/costmap')
    a = ap.parse_args()
    rclpy.init()
    n = Dump(a.topic)
    t0 = time.time()
    while time.time() - t0 < a.wait and (n.cm is None or n.vg is None):
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.cm is None:
        print('没有代价地图'); return 1
    if n.vg is None:
        print('没有视觉栅格'); return 1
    cg, vg = decode(n.cm), decode(n.vg)
    L, I = scale_of(cg)
    print('代价地图 scale: lethal=%d inscribed=%d  max=%d  size=%dx%d res=%.3f origin=(%.2f,%.2f)'
          % (L, I, cg.max(), n.cm.info.width, n.cm.info.height, n.cm.info.resolution,
             n.cm.info.origin.position.x, n.cm.info.origin.position.y))
    u, c = np.unique(cg, return_counts=True)
    print('全图值分布:', {int(k): int(v) for k, v in zip(u, c)})
    print('视觉栅格: cells=%d(res=%.3f) origin=(%.2f,%.2f)'
          % (int((vg == 100).sum()), n.vg.info.resolution,
             n.vg.info.origin.position.x, n.vg.info.origin.position.y))
    # ASCII 切片
    cr, cox, coy = n.cm.info.resolution, n.cm.info.origin.position.x, n.cm.info.origin.position.y
    vr, vox, voy = n.vg.info.resolution, n.vg.info.origin.position.x, n.vg.info.origin.position.y
    print('切片 (%.2f,%.2f) ±%.1fm   C=致命 i=内切 o=膨胀 .=空 ?=未知  *=(视觉有障碍)'
          % (a.x, a.y, a.half))
    step = 2
    hdr = '      '
    for dx in range(-int(a.half / (cr * step)), int(a.half / (cr * step)) + 1):
        hdr += '+' if dx % 5 == 0 else ' '
    print(hdr)
    for dy in range(int(a.half / (cr * step)), -int(a.half / (cr * step)) - 1, -1):
        line = '%+5.1f ' % (a.y + dy * cr * step)
        for dx in range(-int(a.half / (cr * step)), int(a.half / (cr * step)) + 1):
            wx, wy = a.x + dx * cr * step, a.y + dy * cr * step
            cx, cy = int((wx - cox) / cr), int((wy - coy) / cr)
            ch = ' '
            if 0 <= cx < n.cm.info.width and 0 <= cy < n.cm.info.height:
                v = int(cg[cy, cx])
                ch = 'C' if v == L else 'i' if v == I else '.' if v == 0 else '?' if v == 255 else 'o'
            vx, vy = int((wx - vox) / vr), int((wy - voy) / vr)
            if 0 <= vx < n.vg.info.width and 0 <= vy < n.vg.info.height and int(vg[vy, vx]) == 100:
                ch = {'.': '*', '?': '%', 'o': 'O', 'i': 'I', 'C': 'C', ' ': '*'}.get(ch, ch)
            line += ch
        print(line)
    n.destroy_node(); rclpy.shutdown(); return 0


if __name__ == '__main__':
    sys.exit(main())
