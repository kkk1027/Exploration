#!/usr/bin/env python3
"""vision_src_probe.py —— 视觉层"这些格到底是什么东西标出来的"溯源探针。

问题（2026-10-28）：nav_only 在地图原点原地转约 1 rad 后，视觉累积格从 12 涨到 ~1000，
其中 ~87% 离静态地图障碍 >0.3 m。这些到底是
  (a) 地面掠射/深度噪点造成的**鬼影**（低高度带 z≈0.05~0.15），还是
  (b) 雷达扫不到的真家具（z≈0.3~0.8）？
判据：把 `/vision/live_cloud`（车体帧、带真实 z）按当前 yaw 旋到地图帧，逐点分类：
  * near   ：落点格 0.30 m 内有静态地图占据格（与雷达一致）
  * iso    ：0.30 m 内没有（"看起来是新的东西"）
再按 z 带 / 径向距离统计，直接看 iso 点的 z 分布落在哪个高度带。

用法（栈起来后）：
  python3 harness/vision_src_probe.py [--epochs 3] [--sec 10]
"""
import argparse
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class SrcProbe(Node):
    def __init__(self, args):
        super().__init__('vision_src_probe')
        self.a = args
        self.cloud = None
        self.map = None
        self.yaw = 0.0
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qv = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(PointCloud2, '/vision/live_cloud', self.on_cloud, qv)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, q)
        self.create_subscription(Odometry, '/odom_combined', self.on_odom, 20)

    def on_cloud(self, m):
        self.cloud = m

    def on_map(self, m):
        self.map = m

    def on_odom(self, m):
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def cloud_np(msg):
        """按行切片的稳妥解析：每行 point_step 字节，再按字段 offset 取 4 字节 float32。"""
        n = int(msg.width) * int(msg.height)
        step = int(msg.point_step)
        if n <= 0 or step <= 0 or len(msg.data) < n * step:
            return None
        buf = np.frombuffer(msg.data, dtype=np.uint8, count=n * step).reshape(n, step)
        names = {f.name: f.offset for f in msg.fields}
        if not all(k in names for k in ('x', 'y', 'z')):
            return None
        cols = []
        for k in ('x', 'y', 'z'):
            o = int(names[k])
            if o + 4 > step:
                return None
            cols.append(buf[:, o:o + 4].copy().view(np.float32).reshape(-1))
        return cols[0], cols[1], cols[2]

    def prep_map(self):
        g = self.map
        arr = np.asarray(g.data, dtype=np.int8).reshape(g.info.height, g.info.width)
        occ = arr >= 50
        # 0.30 m 邻域膨胀（res=0.05 ⇒ 6 格），用布尔 OR 平移实现，避免依赖 scipy
        pad = 6
        big = np.zeros((occ.shape[0] + 2 * pad, occ.shape[1] + 2 * pad), dtype=bool)
        big[pad:-pad, pad:-pad] = occ
        dil = np.zeros_like(occ)
        for dy in range(-pad, pad + 1):
            for dx in range(-pad, pad + 1):
                dil |= big[pad + dy:pad + dy + occ.shape[0],
                           pad + dx:pad + dx + occ.shape[1]]
        return dict(arr=arr, occ=occ, near=dil,
                    res=g.info.resolution, ox=g.info.origin.position.x,
                    oy=g.info.origin.position.y,
                    h=g.info.height, w=g.info.width)

    def epoch(self, tag):
        if self.cloud is not None and not getattr(self, '_dbg', False):
            self._dbg = True
            c = self.cloud
            print(f"[src] 云结构: w={c.width} h={c.height} step={c.point_step} "
                  f"len(data)={len(c.data)} fields={[(f.name, f.offset) for f in c.fields]}",
                  flush=True)
        if self.cloud is None or self.map is None:
            print(f'[src] {tag}: 数据未到齐（cloud={self.cloud is not None} '
                  f'map={self.map is not None}）', flush=True)
            return
        m = self.meta
        pts = self.cloud_np(self.cloud)
        if pts is None:
            print(f'[src] {tag}: 点云缺字段', flush=True)
            return
        x, y, z = pts
        ok = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & (z > -5) & (z < 5)
        x, y, z = x[ok], y[ok], z[ok]
        # live_cloud 在 base_footprint 帧；机器人停在地图原点，只差一个 yaw
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        mx = c * x - s * y
        my = s * x + c * y
        r = np.hypot(mx, my)
        col = ((mx - m['ox']) / m['res']).astype(int)
        row = ((m['h'] - 1) - ((my - m['oy']) / m['res'])).astype(int)  # PGM 行翻转
        inside = (col >= 0) & (col < m['w']) & (row >= 0) & (row < m['h'])
        near = np.zeros(len(mx), dtype=bool)
        near[inside] = m['near'][row[inside], col[inside]]
        zbin = np.digitize(z, [0.05, 0.15, 0.30, 0.60, 0.90, 1.20])
        rbin = np.digitize(r, [1.0, 2.0, 3.0, 5.0])
        print(f'\n[src] {tag}  yaw={self.yaw:+.2f}  点={len(z)}  '
              f'z: p10={np.percentile(z, 10):.2f} p50={np.percentile(z, 50):.2f} '
              f'p90={np.percentile(z, 90):.2f}', flush=True)
        print('       z带(m)        全部    与雷达一致  孤立(>0.3m)', flush=True)
        zedges = ['<0.05', '0.05-0.15', '0.15-0.30', '0.30-0.60',
                  '0.60-0.90', '0.90-1.20', '>=1.20']
        for b in range(len(zedges)):
            sel = zbin == b
            if sel.sum() == 0:
                continue
            print(f'       {zedges[b]:<12s} {sel.sum():7d} {int((sel & near).sum()):11d} '
                  f'{int((sel & ~near).sum()):12d}', flush=True)
        print('       径向(m)       全部    与雷达一致  孤立', flush=True)
        redges = ['0-1', '1-2', '2-3', '3-5', '>=5']
        for b in range(len(redges)):
            sel = rbin == b
            if sel.sum() == 0:
                continue
            print(f'       {redges[b]:<12s} {sel.sum():7d} {int((sel & near).sum()):11d} '
                  f'{int((sel & ~near).sum()):12d}', flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--epochs', type=int, default=3)
    ap.add_argument('--sec', type=float, default=8.0)
    a = ap.parse_args()
    rclpy.init()
    n = SrcProbe(a)
    t0 = time.time()
    while time.time() - t0 < 90 and (n.cloud is None or n.map is None):
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.map is None:
        print('[src] ✗ 没有 /map')
        return 2
    n.meta = n.prep_map()
    print(f"[src] 静态占据格={int(n.meta['occ'].sum())} "
          f"（0.30 m 膨胀后 {int(n.meta['near'].sum())} 格）", flush=True)
    for i in range(a.epochs):
        end = time.time() + a.sec
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.1)
        n.epoch(f'epoch{i}')
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
