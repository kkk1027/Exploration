#!/usr/bin/env python3
"""ghost_probe.py —— 量化"视觉层鬼影暂留"：原地转一圈，看视觉栅格里的孤立残留怎么长、怎么不消失。

问题（用户 2026-10-28 观察）：小车旋转过程中，视觉障碍/膨胀在小车周围留下一圈"多重漂移鬼影"，
转过去以后这些鬼影停在栅格图上不消失（不是贴合地图的真实障碍）。

判据设计（只用可测的量）：
  * marks         ：/vision/obstacle_grid 里 >0 的格数（视觉累积层当前标记总数）
  * iso_frac      ：其中"与静态 /map 占据格距离 > --iso-m"的比例 = 鬼影候选占比
  * new_since_base：相对基线（旋转前，车静止 10 s）新增的标记格数
  * near_frac     ：距静态障碍 <= --iso-m 的比例（应≈真实障碍）
  鬼影暂留的证据链：旋转后 iso_frac 与 new_since_base 显著上升，且 --hold-sec 内不回落。

流程：
  等待 /vision/obstacle_grid 与 /map → 基线采样 → 发布 /cmd_vel 原地转 --rotate-deg →
  停车后继续采样 --hold-sec，每 --period 秒打一行 JSONL。

用法：
  python3 harness/ghost_probe.py --rotate-deg 360 --hold-sec 90 --out logs/ghost_x.jsonl
退出码 0 正常；2 拿不到数据。
"""
import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class GhostProbe(Node):
    def __init__(self, args):
        super().__init__('ghost_probe')
        self.a = args
        self.vision = None
        self.smap = None
        self.costmap = None
        self.yaw = None
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qv = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(OccupancyGrid, '/vision/obstacle_grid', self.on_vision, qv)
        self.create_subscription(OccupancyGrid, '/map', self.on_smap, q)
        # ★ 2026-10-28：全局代价地图的视觉层是 clearing=False + raytrace 0（只增不减），
        # 必须单独量它是否"视觉格衰减了但代价地图还留着"。
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_costmap, qv)
        self.create_subscription(Odometry, '/odom_combined', self.on_odom, 20)
        self.pub = self.create_publisher(Twist, args.cmd_topic, 10)
        self.live_n = None
        try:
            from sensor_msgs.msg import PointCloud2
            self.create_subscription(PointCloud2, '/vision/live_cloud',
                                     self.on_live, qv)
        except Exception:
            pass
        self.base_marks = None
        self.baseline_cells = None

    def on_vision(self, m):
        self.vision = m

    def on_smap(self, m):
        self.smap = m

    def on_costmap(self, m):
        self.costmap = m

    def on_live(self, m):
        self.live_n = int(m.width) * int(m.height)

    def on_odom(self, m):
        q = m.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    # ---------- 栅格工具 ----------
    @staticmethod
    def cells_xy(grid, vals):
        """把 OccupancyGrid 中满足条件的格转成 map 帧 (x,y) 数组。"""
        idx = np.flatnonzero(vals)
        h, w = grid.info.height, grid.info.width
        row = idx // w
        col = idx % w
        res = grid.info.resolution
        ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
        x = ox + (col + 0.5) * res
        y = oy + (row + 0.5) * res
        # PGM 行 0 在图上、origin 在左下 ⇒ y 需要翻转（本项目已踩过的镜像坑）
        return np.stack([x, oy + (h - row - 0.5) * res], axis=1)

    def static_points(self):
        g = self.smap
        arr = np.asarray(g.data, dtype=np.int8).reshape(g.info.height, g.info.width)
        return self.cells_xy(g, arr >= 50)

    def vision_points(self):
        g = self.vision
        arr = np.asarray(g.data, dtype=np.int8).reshape(g.info.height, g.info.width)
        return arr, self.cells_xy(g, arr > 0)

    def iso_stats(self, vpts, spts, thr):
        """vpts 中距离最近静态障碍 > thr 的比例。分块算，避免内存爆。"""
        if len(vpts) == 0:
            return 0.0, 0.0
        if len(spts) == 0:
            return 1.0, 0.0
        dmin = np.full(len(vpts), 1e9)
        chunk = 2048
        for i in range(0, len(spts), chunk):
            s = spts[i:i + chunk]
            d = np.sqrt(((vpts[:, None, :] - s[None, :, :]) ** 2).sum(-1))
            dmin = np.minimum(dmin, d.min(1))
        iso = float((dmin > thr).mean())
        return iso, float(np.median(dmin))

    # ---------- 主流程 ----------
    def run(self):
        t0 = time.time()
        while time.time() - t0 < self.a.wait:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.vision is not None and self.smap is not None:
                break
        if self.vision is None or self.smap is None:
            print('[ghost] ✗ 拿不到 /vision/obstacle_grid 或 /map', flush=True)
            return 2
        spts = self.static_points()
        print(f'[ghost] 静态图占据格={len(spts)}  阈值 iso_m={self.a.iso_m} m', flush=True)

        f = open(self.a.out, 'w', encoding='utf-8')

        def sample(tag):
            arr, vpts = self.vision_points()
            marks = int((arr > 0).sum())
            iso, med = self.iso_stats(vpts, spts, self.a.iso_m)
            cm_marks, cm_iso = 0, 0.0
            if self.costmap is not None:
                carr = np.asarray(self.costmap.data, dtype=np.int8).reshape(
                    self.costmap.info.height, self.costmap.info.width)
                cpts = self.cells_xy(self.costmap, carr >= 50)
                cm_marks = int(len(cpts))
                cm_iso = self.iso_stats(cpts, spts, self.a.iso_m)[0] if len(cpts) else 0.0
            row = dict(tag=tag, t=round(time.time() - t0, 2),
                       yaw=(round(self.yaw, 3) if self.yaw is not None else None),
                       marks=marks, iso_frac=round(iso, 4),
                       med_dist=round(med, 3),
                       cm_marks=cm_marks, cm_iso=round(cm_iso, 4),
                       live_pts=self.live_n,
                       new=0 if self.baseline_cells is None else int(
                           len(self._diff(vpts))))
            f.write(json.dumps(row) + '\n')
            f.flush()
            print(f"[ghost] {tag:>8} t={row['t']:6.1f}s yaw={row['yaw']} "
                  f"视觉格={marks:5d}(iso {iso * 100:4.1f}%) "
                  f"代价图格={cm_marks:5d}(iso {cm_iso * 100:4.1f}%) "
                  f"med={med:.2f}m new={row['new']} live_pts={row['live_pts']}", flush=True)
            return row

        # 1) 基线（车静止）
        for i in range(self.a.base_n):
            sample(f'base{i}')
            end = time.time() + self.a.period
            while time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.1)
        _, self.baseline_cells = self.vision_points()
        self.base_marks = len(self.baseline_cells)

        # 2) 原地转
        self.pub.publish(Twist())
        sample('pre-rot')
        # 驱动必须持续发（cmd_vel_timeout=1.5 s）
        self.drive(self.a.rate, self.a.rotate_sec)
        self.pub.publish(Twist())
        sample('post-rot')

        # 3) 停车后观察暂留
        n = max(1, int(self.a.hold_sec / self.a.period))
        for i in range(n):
            end = time.time() + self.a.period
            while time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.1)
            sample(f'hold{i:02d}')
        f.close()
        print(f'[ghost] 输出 {self.a.out}', flush=True)
        return 0

    def _diff(self, vpts):
        """基线没覆盖、现在有的格（按 5 cm 网格取整后做集合差）。"""
        if self.baseline_cells is None:
            return vpts
        key = lambda p: set(map(tuple, np.round(p / 0.05).astype(np.int64).tolist()))
        return key(vpts) - key(self.baseline_cells)

    def drive(self, wz, sec):
        end = time.time() + sec
        tw = Twist()
        tw.angular.z = wz
        while time.time() < end:
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rotate-deg', type=float, default=360.0)
    ap.add_argument('--rate', type=float, default=1.2, help='rad/s')
    ap.add_argument('--hold-sec', type=float, default=90.0)
    ap.add_argument('--period', type=float, default=10.0)
    ap.add_argument('--base-n', type=int, default=3)
    ap.add_argument('--wait', type=float, default=120.0)
    ap.add_argument('--iso-m', type=float, default=0.30)
    ap.add_argument('--cmd-topic', default='/cmd_vel',
                    help='/cmd_vel 会被 velocity_smoother 的零速抢，建议 /cmd_vel_nav')
    ap.add_argument('--out', default='logs/ghost_probe.jsonl')
    a = ap.parse_args()
    a.rotate_sec = math.radians(abs(a.rotate_deg)) / max(0.1, abs(a.rate))
    rclpy.init()
    node = GhostProbe(a)
    try:
        rc = node.run()
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
