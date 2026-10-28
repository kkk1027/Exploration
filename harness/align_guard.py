#!/usr/bin/env python3
"""align_guard.py —— 持续监视"雷达点云 vs 静态地图"是否贴合（本轮事故的哨兵）。

2026-10-28 事故：把 AMCL 的 update_min_d/a 压到 0 后，车**静止**时滤波器仍每帧
重采样，在没有运动信息的情况下粒子云随机游走，最终锁到错误位姿
（map->base=(0.52,-0.73,yaw 3.4°)，里程计只有 (0.004,0,0.36°)）。
雷达点云被 map->odom 一起挪走 ~0.9 m，RViz 上看起来就是"点云和地图镜像/不贴合"。
当时是靠人眼看出来的，因此这里做一个每 --period 秒自检的哨兵：

判据（三条一起看，避免误报）：
  hit_odom  扫描投影到 odom_combined 后与地图贴合率  ← 地图/雷达本身是否正常
  hit_map   扫描投影到 map           后与地图贴合率  ← 含 AMCL 修正
  * hit_odom 高而 hit_map 低 ⇒ **AMCL 位姿错了**（本事故形态），打印 WARN 并按 --out 记盘
  * 两者都低 ⇒ 地图或雷达链路有问题

用法（栈在跑时，独立进程）：
  python3 harness/align_guard.py [--period 10] [--tol 0.10] [--out logs/align_guard.jsonl]
"""
import argparse
import json
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import tf2_ros


class Guard(Node):
    def __init__(self, a):
        super().__init__('align_guard')
        self.a = a
        self.map = None
        self.scans = []
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, q)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.occ = None
        self.f = open(a.out, 'w', encoding='utf-8') if a.out else None

    def on_map(self, m):
        self.map = m
        self.occ = None

    def on_scan(self, m):
        self.scans.append(m)
        if len(self.scans) > 12:
            self.scans.pop(0)

    def occ_cells(self):
        if self.occ is not None:
            return self.occ
        g = self.map
        arr = np.asarray(g.data, dtype=np.int16).reshape(g.info.height, g.info.width)
        r, c = np.nonzero(arr >= 50)
        res = g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        # ★ OccupancyGrid 行 0 = 最小 y（不是 PGM 的"行 0 在上"）
        self.occ = np.stack([ox + (c + 0.5) * res, oy + (r + 0.5) * res], 1)
        return self.occ

    def hit(self, P, occ, tol):
        if len(P) == 0:
            return 0.0
        d = np.full(len(P), 1e9)
        for i in range(0, len(occ), 4096):
            s = occ[i:i + 4096]
            d = np.minimum(d, ((P[:, None, :] - s[None, :, :]) ** 2).sum(-1).min(1))
        return float((np.sqrt(d) <= tol).mean())

    def project(self, frame):
        out = []
        for s in self.scans:
            for t in (Time.from_msg(s.header.stamp), Time()):
                try:
                    tr = self.buf.lookup_transform(frame, s.header.frame_id, t,
                                                   timeout=rclpy.duration.Duration(seconds=0.15))
                    break
                except Exception:                              # noqa: BLE001
                    tr = None
            if tr is None:
                continue
            tt = tr.transform.translation
            q = tr.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            ang = s.angle_min + np.arange(len(s.ranges)) * s.angle_increment
            r = np.asarray(s.ranges, dtype=np.float64)
            ok = np.isfinite(r) & (r >= s.range_min) & (r <= min(s.range_max, 12.0))
            ang, r = ang[ok], r[ok]
            lx = r * np.cos(ang)
            ly = r * np.sin(ang)
            out.append(np.stack([tt.x + lx * math.cos(yaw) - ly * math.sin(yaw),
                                 tt.y + lx * math.sin(yaw) + ly * math.cos(yaw)], 1))
        return np.vstack(out) if out else np.zeros((0, 2))

    def check(self):
        if self.map is None or len(self.scans) < 3:
            return
        occ = self.occ_cells()
        ko = self.hit(self.project('odom_combined'), occ, self.a.tol)
        km = self.hit(self.project('map'), occ, self.a.tol)
        row = dict(t=round(time.time(), 1), hit_odom=round(ko, 4), hit_map=round(km, 4),
                   tol=self.a.tol)
        if ko >= 0.9 and km < 0.8:
            row['verdict'] = 'AMCL_POSE_WRONG'
            print(f'[guard] ⚠ AMCL 位姿错误：点云在 odom 帧贴合 {ko * 100:.1f}%，'
                  f'但经 AMCL 后只有 {km * 100:.1f}% ⇒ 检查 /amcl 与 map->odom', flush=True)
        elif ko < 0.8:
            row['verdict'] = 'MAP_OR_SCAN_BAD'
            print(f'[guard] ⚠ 地图/雷达本身不贴合：odom 帧 {ko * 100:.1f}%', flush=True)
        else:
            row['verdict'] = 'OK'
            print(f'[guard] ✓ 贴合正常 odom={ko * 100:.1f}% map={km * 100:.1f}%', flush=True)
        if self.f:
            self.f.write(json.dumps(row, ensure_ascii=False) + '\n')
            self.f.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--period', type=float, default=10.0)
    ap.add_argument('--tol', type=float, default=0.10)
    ap.add_argument('--out', default='logs/align_guard.jsonl')
    a = ap.parse_args()
    rclpy.init()
    g = Guard(a)
    t0 = time.time()
    while time.time() - t0 < 60 and g.map is None:
        rclpy.spin_once(g, timeout_sec=0.2)
    while rclpy.ok():
        end = time.time() + a.period
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(g, timeout_sec=0.1)
        try:
            g.check()
        except Exception as e:                                 # noqa: BLE001
            print(f'[guard] 检查失败：{e}', flush=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
