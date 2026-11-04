#!/usr/bin/env python3
"""costmap_lethal_audit.py —— 局部代价地图 lethal 格的"归因审计"。

为什么需要：上一轮证明视觉层给局部代价地图带来 +2663 个 lethal 格、且只在
"当前没在看的方位"残留。但**残留不等于虚假** —— 如果这些格对应真实家具
（雷达 2D 扫不到的低矮/高处几何），那"回落到基线"这个判据本身就是错的，
保留它们才是正确行为。所以要把 lethal 格按来源归因，而不是只看总数。

归因（逐格，全部用 map 帧坐标）：
  lidar     距任一激光末端点 ≤ tol
  map       距静态地图占据格 ≤ tol（OccupancyGrid 行 0 = 最小 y）
  vgrid     距 /vision/obstacle_grid 标记格 ≤ tol（视觉累积记忆）
  unsupported 三者都不沾 ⇒ 既没有雷达支撑、也不在静态图、也不是视觉记忆 → 最可疑

同时给出：锥内/锥外（相对机器人朝向 ±cone_deg、cone_m）、以及"仅视觉支撑"的子集统计。

用法（栈在跑、且已在想要的位姿时）：
  python3 harness/costmap_lethal_audit.py --tol 0.15 --out logs/lethal_audit.jsonl
输出：每个采样一行 JSON + 终端汇总表。
"""
import argparse
import json
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import tf2_ros

try:
    from nav2_msgs.msg import Costmap as Nav2Costmap
except Exception:                                            # noqa: BLE001
    Nav2Costmap = None


class Audit(Node):
    def __init__(self, a):
        super().__init__('lethal_audit')
        self.a = a
        self.raw = None
        self.map = None
        self.vgrid = None
        self.scans = []
        self.pos = None
        self.yaw = None
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        qm = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qv = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE)
        if Nav2Costmap is not None:
            self.create_subscription(Nav2Costmap, '/local_costmap/costmap_raw',
                                     lambda m: setattr(self, 'raw', m), qm)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 lambda m: setattr(self, 'cm', m), qm)
        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'map', m), qm)
        self.create_subscription(OccupancyGrid, '/vision/obstacle_grid',
                                 lambda m: setattr(self, 'vgrid', m), qv)
        self.create_subscription(LaserScan, '/scan',
                                 lambda m: self.scans.append(m) if len(self.scans) < 20 else None, 10)
        self.create_subscription(Odometry, '/odom_combined', self.on_odom, 20)

    def on_odom(self, m):
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pos = (m.pose.pose.position.x, m.pose.pose.position.y)

    # ---------- 点集 ----------
    def occ_xy(self):
        g = self.map
        arr = np.asarray(g.data, dtype=np.int16).reshape(g.info.height, g.info.width)
        r, c = np.nonzero(arr >= 50)
        res = g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        return np.stack([ox + (c + 0.5) * res, oy + (r + 0.5) * res], 1)   # 行0=最小y

    def vgrid_xy(self):
        if self.vgrid is None:
            return np.zeros((0, 2))
        g = self.vgrid
        arr = np.asarray(g.data, dtype=np.int8).reshape(g.info.height, g.info.width)
        r, c = np.nonzero(arr > 0)
        res = g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        return np.stack([ox + (c + 0.5) * res, oy + (r + 0.5) * res], 1)

    def lidar_xy(self):
        out = []
        for s in self.scans:
            tr = None
            for t in (Time.from_msg(s.header.stamp), Time()):
                try:
                    tr = self.buf.lookup_transform('map', s.header.frame_id, t,
                                                   timeout=rclpy.duration.Duration(seconds=0.15))
                    break
                except Exception:                            # noqa: BLE001
                    tr = None
            if tr is None:
                continue
            tt, q = tr.transform.translation, tr.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            ang = s.angle_min + np.arange(len(s.ranges)) * s.angle_increment
            r = np.asarray(s.ranges, dtype=np.float64)
            ok = np.isfinite(r) & (r >= s.range_min) & (r <= min(s.range_max, 12.0))
            ang, r = ang[ok], r[ok]
            lx, ly = r * np.cos(ang), r * np.sin(ang)
            out.append(np.stack([tt.x + lx * math.cos(yaw) - ly * math.sin(yaw),
                                 tt.y + lx * math.sin(yaw) + ly * math.cos(yaw)], 1))
        return np.vstack(out) if out else np.zeros((0, 2))

    def lethal_xy(self, want=254):
        """want=254 → 障碍本体(lethal)；want=253 → 内切/膨胀环(inscribed)。
        两者必须分开：内切环天生离障碍 ~0.335 m，混在一起会把"真实膨胀"误判成
        "无支撑鬼影"（本轮就踩了：一次性把 1634 格误算成 unsupported）。"""
        if self.raw is None:
            return np.zeros((0, 2))
        d = np.frombuffer(bytes(self.raw.data), dtype=np.uint8)
        w = int(self.raw.metadata.size_x)
        res = float(self.raw.metadata.resolution)
        ox = float(self.raw.metadata.origin.position.x)
        oy = float(self.raw.metadata.origin.position.y)
        idx = np.flatnonzero(d == want)
        row, col = idx // w, idx % w
        return np.stack([ox + (col + 0.5) * res, oy + (row + 0.5) * res], 1)

    @staticmethod
    def near_mask(P, Q, tol, chunk=4096):
        if len(P) == 0:
            return np.zeros(0, dtype=bool)
        if len(Q) == 0:
            return np.zeros(len(P), dtype=bool)
        best = np.full(len(P), 1e9)
        for i in range(0, len(Q), chunk):
            q = Q[i:i + chunk]
            best = np.minimum(best, ((P[:, None, :] - q[None, :, :]) ** 2).sum(-1).min(1))
        return np.sqrt(best) <= tol

    def audit(self, tag):
        L = self.lethal_xy(254)
        if len(L) == 0 or self.pos is None:
            print(f'[audit] {tag}: 无数据（raw={self.raw is not None} pos={self.pos is not None}）')
            return None
        lid = self.lidar_xy()
        occ = self.occ_xy()
        vg = self.vgrid_xy()
        m_lid = self.near_mask(L, lid, self.a.tol)
        m_map = self.near_mask(L, occ, self.a.tol)
        m_vg = self.near_mask(L, vg, self.a.tol)
        unsup = ~(m_lid | m_map | m_vg)
        vis_only = m_vg & ~m_lid
        # 锥内/锥外
        dx, dy = L[:, 0] - self.pos[0], L[:, 1] - self.pos[1]
        r = np.hypot(dx, dy)
        br = (np.arctan2(dy, dx) - self.yaw + np.pi) % (2 * np.pi) - np.pi
        inc = (np.abs(br) <= math.radians(self.a.cone_deg)) & (r <= self.a.cone_m) & (r >= 0.35)
        n_insc = int(len(self.lethal_xy(253)))
        if self.a.dump_cells:
            pts = {}
            for name, mask in (('vision_only', vis_only), ('unsupported', unsup),
                               ('all_lethal', np.ones(len(L), dtype=bool))):
                sel = L[mask]
                pts[name] = [[round(float(x), 3), round(float(y), 3)] for x, y in sel]
            with open(self.a.dump_cells, 'w', encoding='utf-8') as f:
                json.dump(dict(pos=self.pos, yaw=self.yaw, cells=pts), f, ensure_ascii=False)
        row = dict(tag=tag, t=round(time.time(), 1), n=int(len(L)), inscribed=n_insc,
                   lidar=int(m_lid.sum()), stat=int(m_map.sum()), vgrid=int(m_vg.sum()),
                   vis_only=int(vis_only.sum()), unsupported=int(unsup.sum()),
                   cone_in=int(inc.sum()),
                   vis_only_cone_in=int((vis_only & inc).sum()),
                   unsupported_cone_in=int((unsup & inc).sum()))
        print(f"[audit] {tag}: lethal(254)={row['n']:5d} inscribed(253)={n_insc:5d} "
              f"雷达支撑={row['lidar']:5d} "
              f"静态图={row['stat']:5d} 视觉记忆={row['vgrid']:5d} "
              f"**仅视觉**={row['vis_only']:5d} **无任何支撑**={row['unsupported']:5d} "
              f"| 锥内={row['cone_in']:4d}(仅视觉 {row['vis_only_cone_in']:3d}, "
              f"无支撑 {row['unsupported_cone_in']:3d})", flush=True)
        return row

    def run(self):
        t0 = time.time()
        while time.time() - t0 < self.a.wait:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.raw is not None and self.map is not None and self.pos is not None:
                break
        t1 = time.time()
        while time.time() - t1 < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        rows = []
        for i in range(self.a.samples):
            row = self.audit(f's{i}')
            if row:
                rows.append(row)
            end = time.time() + self.a.period
            while time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.1)
        if rows and self.a.out:
            with open(self.a.out, 'w', encoding='utf-8') as f:
                for r in rows:
                    f.write(json.dumps(r, ensure_ascii=False) + '\n')
            print(f'[audit] 输出 {self.a.out}')
        return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tol', type=float, default=0.15)
    ap.add_argument('--samples', type=int, default=3)
    ap.add_argument('--period', type=float, default=5.0)
    ap.add_argument('--wait', type=float, default=60.0)
    ap.add_argument('--cone-deg', type=float, default=35.0)
    ap.add_argument('--cone-m', type=float, default=4.5)
    ap.add_argument('--out', default='logs/lethal_audit.jsonl')
    ap.add_argument('--dump-cells', default='',
                    help='把 vision_only / unsupported / all_lethal 的格子坐标落盘（JSON），'
                         '供 harness/classify_costmap.py 用世界真值判定真假')
    a = ap.parse_args()
    rclpy.init()
    n = Audit(a)
    try:
        rc = n.run()
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == '__main__':
    main()
