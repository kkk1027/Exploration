#!/usr/bin/env python3
"""vision_vs_truth_probe.py —— 量"视觉云/代价地图里的色块"到底贴不贴真实障碍。

用户 2026-10-28 现象：局部代价地图里真实障碍是一条笔直细线，而外面包着一圈
不规则、不贴合的青色/品红色块（膨胀/视觉标记），把宽处压成窄道、窄道堵成墙。

本探针同时抓三样东西，只读：
  ① TF 位姿滞后：用 Time()（nav2/视觉节点在 TF 冻结时也走这条）取到的
     map<-camera 变换，它的时间戳相对**点云自己的 stamp** 差多少
     ⇒ Δt 越大、车越快，视觉点被贴错的距离 ≈ 距离 × ω·Δt
  ② 视觉点 vs 雷达回波：每个视觉点(x,y) 到最近 **激光点** 的距离分布
     ⇒ 真障碍应当 <0.15 m；大片"不贴合色块"会显著偏大
  ③ 视觉点 vs 静态地图：到最近静态占据格的距离（OccupancyGrid 行 0 = 最小 y）
另外统计雷达点 vs 视觉点各自覆盖的"孤立"比例，便于对照。

用法（栈在跑时）：
  python3 harness/vision_vs_truth_probe.py [--samples 5] [--tol 0.15] [--out logs/vvt.json]
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
from sensor_msgs.msg import LaserScan, PointCloud2
import tf2_ros


def cloud_np(msg):
    n = int(msg.width) * int(msg.height)
    step = int(msg.point_step)
    if n <= 0 or step <= 0 or len(msg.data) < n * step:
        return None
    buf = np.frombuffer(msg.data, dtype=np.uint8, count=n * step).reshape(n, step)
    names = {f.name: f.offset for f in msg.fields}
    if not all(k in names for k in 'xyz'):
        return None
    out = []
    for k in 'xyz':
        o = int(names[k])
        out.append(buf[:, o:o + 4].copy().view(np.float32).reshape(-1))
    return out[0], out[1], out[2]


class VVT(Node):
    def __init__(self, a):
        super().__init__('vision_vs_truth')
        self.a = a
        self.map = None
        self.cloud = []
        self.scans = []
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        qm = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qv = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'map', m), qm)
        self.create_subscription(PointCloud2, self.a.cloud_topic,
                                 lambda m: self.cloud.append(m), qv)
        self.create_subscription(LaserScan, '/scan',
                                 lambda m: self.scans.append(m), 10)

    # ---------- 工具 ----------
    def occ_xy(self):
        g = self.map
        arr = np.asarray(g.data, dtype=np.int16).reshape(g.info.height, g.info.width)
        r, c = np.nonzero(arr >= 50)
        res = g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        # OccupancyGrid: 行 0 = 最小 y
        return np.stack([ox + (c + 0.5) * res, oy + (r + 0.5) * res], 1)

    def scan_xy(self, s):
        for t in (Time.from_msg(s.header.stamp), Time()):
            try:
                tr = self.buf.lookup_transform('map', s.header.frame_id, t,
                                               timeout=rclpy.duration.Duration(seconds=0.15))
                break
            except Exception:                                  # noqa: BLE001
                tr = None
        if tr is None:
            return np.zeros((0, 2))
        tt, q = tr.transform.translation, tr.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        ang = s.angle_min + np.arange(len(s.ranges)) * s.angle_increment
        r = np.asarray(s.ranges, dtype=np.float64)
        ok = np.isfinite(r) & (r >= s.range_min) & (r <= min(s.range_max, 12.0))
        ang, r = ang[ok], r[ok]
        lx, ly = r * np.cos(ang), r * np.sin(ang)
        return np.stack([tt.x + lx * math.cos(yaw) - ly * math.sin(yaw),
                         tt.y + lx * math.sin(yaw) + ly * math.cos(yaw)], 1)

    @staticmethod
    def nn_dist(P, Q, chunk=2048):
        """P 中每点到 Q 的最近距离。"""
        if len(P) == 0 or len(Q) == 0:
            return np.full(len(P), np.nan)
        d = np.full(len(P), 1e9)
        for i in range(0, len(Q), chunk):
            q = Q[i:i + chunk]
            d = np.minimum(d, ((P[:, None, :] - q[None, :, :]) ** 2).sum(-1).min(1))
        return np.sqrt(d)

    # ---------- 主流程 ----------
    def run(self):
        t0 = time.time()
        while time.time() - t0 < 60 and (self.map is None or not self.cloud):
            rclpy.spin_once(self, timeout_sec=0.2)
        t1 = time.time()
        while time.time() - t1 < 4:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.map is None:
            print('[vvt] ✗ 没有 /map'); return 2
        if not self.cloud:
            print(f'[vvt] ✗ 没有 {self.a.cloud_topic}；用 --cloud-topic 指定'); return 2

        occ = self.occ_xy()
        lidar = np.vstack([self.scan_xy(s) for s in self.scans]) if self.scans else np.zeros((0, 2))
        print(f'[vvt] 静态占据格={len(occ)}  雷达点={len(lidar)}  视觉帧={len(self.cloud)}')

        rows = []
        for k, m in enumerate(self.cloud[-self.a.samples:]):
            pts = cloud_np(m)
            if pts is None:
                continue
            x, y, z = pts
            ok = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
            x, y, z = x[ok], y[ok], z[ok]
            frame = m.header.frame_id
            stamp = Time.from_msg(m.header.stamp)
            # ① 位姿滞后：Time() 取到的变换时间戳 vs 点云 stamp
            lag = None
            try:
                tr = self.buf.lookup_transform('map', frame, Time())
                tst = tr.header.stamp
                lag = (stamp.nanoseconds - (tst.sec * 10**9 + tst.nanosec)) * 1e-9
                q = tr.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                tt = tr.transform.translation
                Pm = np.stack([tt.x + x * math.cos(yaw) - y * math.sin(yaw),
                               tt.y + x * math.sin(yaw) + y * math.cos(yaw)], 1)
            except Exception as e:                             # noqa: BLE001
                print(f'  [帧{k}] 变换失败 {e}')
                continue
            d_lidar = self.nn_dist(Pm, lidar) if len(lidar) else np.full(len(Pm), np.nan)
            d_map = self.nn_dist(Pm, occ)
            frac_lidar = float(np.nanmean(d_lidar <= self.a.tol)) if len(d_lidar) else float('nan')
            frac_map = float(np.nanmean(d_map <= self.a.tol))
            row = dict(frame=k, n=len(Pm), lag_s=(round(lag, 3) if lag is not None else None),
                       med_to_lidar=(round(float(np.nanmedian(d_lidar)), 3) if len(d_lidar) else None),
                       frac_near_lidar=round(frac_lidar, 4),
                       med_to_map=round(float(np.nanmedian(d_map)), 3),
                       frac_near_map=round(frac_map, 4),
                       z_p10=round(float(np.percentile(z, 10)), 3),
                       z_p50=round(float(np.percentile(z, 50)), 3),
                       z_p90=round(float(np.percentile(z, 90)), 3))
            rows.append(row)
            print(f"  [帧{k}] 点={row['n']:5d} 位姿滞后={row['lag_s']}s  "
                  f"到雷达: 中位={row['med_to_lidar']}m ≤{self.a.tol}m 占 {frac_lidar * 100:5.1f}%  "
                  f"到静态图: 中位={row['med_to_map']}m 占 {frac_map * 100:5.1f}%  "
                  f"z p50={row['z_p50']}m")
        if rows:
            lags = [r['lag_s'] for r in rows if r['lag_s'] is not None]
            fl = [r['frac_near_lidar'] for r in rows if r['frac_near_lidar'] == r['frac_near_lidar']]
            if lags:
                print(f'[vvt] 位姿滞后：中位 {np.median(lags):+.3f}s  最大 {max(lags):+.3f}s '
                      f'（>0 表示 TF 比点云旧）')
            if fl:
                print(f'[vvt] 视觉点贴合雷达（≤{self.a.tol}m）：平均 {np.mean(fl) * 100:.1f}% '
                      f'⇒ 越低说明越多"不贴合的色块"')
        if self.a.out:
            json.dump(rows, open(self.a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
            print(f'[vvt] 输出 {self.a.out}')
        return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cloud-topic', default='/vision/live_ttl_cloud')
    ap.add_argument('--samples', type=int, default=5)
    ap.add_argument('--tol', type=float, default=0.15)
    ap.add_argument('--out', default='logs/vision_vs_truth.json')
    a = ap.parse_args()
    rclpy.init()
    n = VVT(a)
    try:
        rc = n.run()
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == '__main__':
    main()
