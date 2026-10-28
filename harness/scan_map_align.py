#!/usr/bin/env python3
"""scan_map_align.py —— 用【激光扫描 vs 实际加载进 nav2 的静态地图】自证地图朝向。

问题（2026-10-28 真值工具报出）：`maps/explore_latest.pgm` 的行序疑似是 bottom-up，
而 nav2/map_server 按 top-down 读 —— 那样加载出来的静态地图相对世界就是 y 镜像的，
用户之前反复看到的"点云和地图对不上/沿中轴镜像"就可能是这个。

本探针**不依赖任何外部真值**，只比较两件都在跑的东西：
  * /scan（雷达点，世界真实几何）
  * /map（nav2 实际加载的静态地图）
做法：把每条扫描线在 range 内的末端点用 TF 投到 map 帧，然后统计这些末端点
落在"地图占据格"上的比例（容差 tol）。比例高 = 地图与雷达一致；低 = 不一致。
再分别用四种变换重算，看哪一种把比例拉满：
  as-is          地图按 nav2 规范读（行 0 在上）
  y-flip         地图沿自身 y 轴上下翻转（等价于 bottom-up 文件被 top-down 读）
  ymirror@y0     地图在世界 y=0 处镜像
  ymirror@best   在 ±1.5 m 内搜一个最佳镜像轴（用于确认"镜像轴不在 0"的怀疑）
比例最高者就是现实，据此可以判断 nav2 眼里那张图是不是镜像的。

用法（栈在跑时）：
  python3 harness/scan_map_align.py [--tol 0.10] [--frames 30] [--out logs/align.json]
退出码 0；2 拿不到数据。
"""
import argparse
import json
import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import tf2_ros


class Align(Node):
    def __init__(self, a):
        super().__init__('scan_map_align')
        self.a = a
        self.map = None
        self.scans = []
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, q)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

    def on_map(self, m):
        self.map = m

    def on_scan(self, m):
        if len(self.scans) < self.a.frames:
            self.scans.append(m)

    # ---------- 地图工具 ----------
    def occ_cells(self):
        g = self.map
        arr = np.asarray(g.data, dtype=np.int16).reshape(g.info.height, g.info.width)
        rows, cols = np.nonzero(arr >= 50)
        res = g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        x = ox + (cols + 0.5) * res
        # ★ nav_msgs/OccupancyGrid 规范：data 行 0 = **最小 y**（底边），
        #   nav2 map_server / RViz 都按这个约定显示。这里曾误用 PGM 的"行 0 在上"，
        #   导致把镜像地图当成正常地图（2026-10-28 因此误判过一次）。
        y = oy + (rows + 0.5) * res
        return np.stack([x, y], axis=1)

    def hit_rate(self, pts_xy, occ, tol):
        if len(pts_xy) == 0 or len(occ) == 0:
            return 0.0
        d = np.full(len(pts_xy), 1e9)
        for i in range(0, len(occ), 4096):
            s = occ[i:i + 4096]
            dd = ((pts_xy[:, None, :] - s[None, :, :]) ** 2).sum(-1)
            d = np.minimum(d, dd.min(1))
        return float((np.sqrt(d) <= tol).mean())

    def transform_map(self, occ, mode, axis=None):
        """对占据格做变换；返回新点集（用来模拟"另一种读法/镜像"）。"""
        if mode == 'as-is':
            return occ
        if mode == 'y-flip':
            g = self.map
            res = g.info.resolution
            oy = g.info.origin.position.y
            h = g.info.height
            y_new = 2 * (oy + h * res / 2.0) - occ[:, 1]
            return np.stack([occ[:, 0], y_new], axis=1)
        if mode == 'ymirror':
            return np.stack([occ[:, 0], 2 * axis - occ[:, 1]], axis=1)
        if mode == 'xmirror':
            return np.stack([2 * axis - occ[:, 0], occ[:, 1]], axis=1)
        return occ

    def run(self):
        import time
        t0 = time.time()
        while time.time() - t0 < 60 and (self.map is None or len(self.scans) < min(5, self.a.frames)):
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.map is None:
            print('[align] ✗ 没有 /map')
            return 2
        if not self.scans:
            print('[align] ✗ 没有 /scan')
            return 2
        # ★ 必须再空转几秒把 /tf 灌进 buffer，否则第一次投影会全部 LookupException
        #   （实测：一拿到 5 帧就结束等待 ⇒ TF buffer 还是空的）。
        import time as _t
        t1 = _t.time()
        while _t.time() - t1 < self.a.tf_warm:
            rclpy.spin_once(self, timeout_sec=0.1)
        occ = self.occ_cells()
        print(f'[align] 地图 {self.map.info.width}x{self.map.info.height} '
              f'res={self.map.info.resolution:.3f} 占据格={len(occ)}；扫描帧={len(self.scans)}')

        # 累积扫描末端点（map 帧）
        pts = []
        for s in self.scans:
            f = s.header.frame_id
            try:
                tr = self.buf.lookup_transform('map', f, Time.from_msg(s.header.stamp),
                                               timeout=rclpy.duration.Duration(seconds=0.2))
            except Exception:
                try:
                    tr = self.buf.lookup_transform('map', f, Time())
                except Exception:
                    continue
            t = tr.transform.translation
            q = tr.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            ang = s.angle_min + np.arange(len(s.ranges)) * s.angle_increment
            r = np.asarray(s.ranges, dtype=np.float64)
            ok = np.isfinite(r) & (r >= s.range_min) & (r <= min(s.range_max, 12.0))
            ang, r = ang[ok], r[ok]
            x = t.x + r * np.cos(ang + yaw)
            y = t.y + r * np.sin(ang + yaw)
            pts.append(np.stack([x, y], axis=1))
        if not pts:
            print('[align] ✗ 投影失败（TF 不可用）')
            return 2
        P = np.vstack(pts)
        print(f'[align] 激光末端点={len(P)}  tol={self.a.tol} m')

        results = {}
        for mode in ('as-is', 'y-flip'):
            occ2 = self.transform_map(occ, mode)
            results[mode] = round(self.hit_rate(P, occ2, self.a.tol), 4)
            print(f'  {mode:12s} 命中率={results[mode] * 100:5.1f}%')
        # 扫镜像轴：y 轴镜像，轴心 y0 从 -1.5..1.5
        best = (None, -1.0)
        for y0 in np.arange(-1.5, 1.51, 0.1):
            r = self.hit_rate(P, self.transform_map(occ, 'ymirror', axis=float(y0)), self.a.tol)
            results[f'ymirror@{y0:+.1f}'] = round(r, 4)
            if r > best[1]:
                best = (float(y0), r)
        print(f'  ymirror 扫描：最佳轴心 y0={best[0]:+.1f}  命中率={best[1] * 100:5.1f}%')
        xb = (None, -1.0)
        for x0 in np.arange(-1.5, 1.51, 0.5):
            r = self.hit_rate(P, self.transform_map(occ, 'xmirror', axis=float(x0)), self.a.tol)
            if r > xb[1]:
                xb = (float(x0), r)
        print(f'  xmirror 扫描：最佳轴心 x0={xb[0]:+.1f}  命中率={xb[1] * 100:5.1f}%')

        verdict = 'as-is 正常'
        if results['y-flip'] > results['as-is'] + 0.15:
            verdict = (f'⚠ 地图疑似 y 镜像：y-flip 命中 {results["y-flip"] * 100:.1f}% '
                       f'>> as-is {results["as-is"] * 100:.1f}%')
        elif best[1] > results['as-is'] + 0.15:
            verdict = (f'⚠ 地图疑似在 y={best[0]:+.1f} 处镜像：命中 {best[1] * 100:.1f}% '
                       f'>> as-is {results["as-is"] * 100:.1f}%')
        print(f'[align] 结论：{verdict}')
        out = dict(as_is=results['as-is'], y_flip=results['y-flip'],
                   best_ymirror_axis=best[0], best_ymirror_rate=round(best[1], 4),
                   best_xmirror_axis=xb[0], best_xmirror_rate=round(xb[1], 4),
                   tol=self.a.tol, scan_points=int(len(P)), verdict=verdict)
        if self.a.out:
            json.dump(out, open(self.a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
            print(f'[align] 输出 {self.a.out}')
        return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tol', type=float, default=0.10)
    ap.add_argument('--frames', type=int, default=30)
    ap.add_argument('--tf-warm', type=float, default=4.0,
                    help='等待把 /tf 灌进 buffer 的秒数（否则投影全失败）')
    ap.add_argument('--out', default='logs/scan_map_align.json')
    a = ap.parse_args()
    rclpy.init()
    n = Align(a)
    try:
        rc = n.run()
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == '__main__':
    main()
