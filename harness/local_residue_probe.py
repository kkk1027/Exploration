#!/usr/bin/env python3
"""local_residue_probe.py —— 局部代价地图（含体素视觉层）的"鬼影/膨胀无故扩张"量化探针。

用户 2026-10-28 观察：局部代价地图里鬼影只要出现过一次就消不掉，
并且障碍的膨胀范围看起来会无缘无故地扩大。

本探针只读 + 只发 /cmd_vel（不发目标），逐秒记录：
  yaw        当前偏航（/odom_combined）
  raw_lethal 局部代价地图 raw（master，未膨胀）里 >=99 的格数   ← 障碍本体
  inf_area   /local_costmap/costmap 里 cost∈(0,99) 的格数        ← 膨胀出来的面积
  all_occ    /local_costmap/costmap 里 cost>0 的格数
  ratio      inf_area / max(1, raw_lethal)   ← "每个障碍格平均带出的膨胀面积"，
             该值无故上升 = 膨胀范围在扩（几何/参数不变时应基本恒定）
  vgrid      /vision/obstacle_grid 的标记格数（交叉参考）

流程：基线静止 → 转 N 整圈（偏航反馈，每圈后停 settle 秒采样）→ 转回原朝向 → 停 hold 秒。
判据：转回原朝向并停稳后，raw_lethal / inf_area / ratio 若显著高于基线且不回落
     ⇒ 残留（鬼影）。ratio 上升而 raw_lethal 不变 ⇒ 膨胀无故扩张。

用法：
  python3 harness/local_residue_probe.py --turns 3 --settle 10 --hold 40 --out logs/x.jsonl
退出码 0；2 拿不到数据。
"""
import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

try:
    from nav2_msgs.msg import Costmap as Nav2Costmap
except Exception:                                   # noqa: BLE001
    Nav2Costmap = None


class LocalResidue(Node):
    def __init__(self, a):
        super().__init__('local_residue_probe')
        self.a = a
        self.raw = None          # /local_costmap/costmap_raw  (nav2_msgs/Costmap)
        self.occ = None          # /local_costmap/costmap      (OccupancyGrid，已膨胀)
        self.vgrid = None        # /vision/obstacle_grid       (交叉参考)
        self.yaw = None
        self.pos = None
        self.create_subscription(Odometry, '/odom_combined', self.on_odom, 20)
        self.pub = self.create_publisher(Twist, a.cmd_topic, 10)

    def on_odom(self, m):
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y * q.y + q.z * q.z))
        self.pos = (m.pose.pose.position.x, m.pose.pose.position.y)

    def setup_subs(self):
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.on_occ_grid, q)
        if Nav2Costmap is not None:
            self.create_subscription(Nav2Costmap, '/local_costmap/costmap_raw',
                                     self.on_raw, q)
        self.create_subscription(OccupancyGrid, '/vision/obstacle_grid', self.on_grid_v, q)

    def on_occ_grid(self, m):
        self.occ = m

    def on_raw(self, m):
        self.raw = m

    def on_grid_v(self, m):
        self.vgrid = m

    def cone_split(self):
        """把 raw（master）里 >=99 的格按"机器人前方视野锥内/外"分开计数。
        用户真正痛的是"车前方/周围被虚假 lethal 压窄"，而不是整窗总数。"""
        import numpy as np
        if self.raw is None or self.pos is None or self.yaw is None:
            return None, None
        d = np.frombuffer(bytes(self.raw.data), dtype=np.uint8)
        w, h = int(self.raw.metadata.size_x), int(self.raw.metadata.size_y)
        res = float(self.raw.metadata.resolution)
        ox = float(self.raw.metadata.origin.position.x)
        oy = float(self.raw.metadata.origin.position.y)
        idx = np.flatnonzero(d == 254)   # 只统计障碍本体
        if idx.size == 0:
            return 0, 0
        row = idx // w
        col = idx % w
        x = ox + (col + 0.5) * res
        y = oy + (row + 0.5) * res
        dx = x - self.pos[0]
        dy = y - self.pos[1]
        r = np.hypot(dx, dy)
        br = np.arctan2(dy, dx) - self.yaw
        br = (br + np.pi) % (2 * np.pi) - np.pi
        inc = (np.abs(br) <= math.radians(self.a.cone_deg)) & (r <= self.a.cone_m) & (r >= 0.35)
        return int(inc.sum()), int((~inc).sum())

    def counts(self):
        """★ 2026-11-04 修正口径：nav2 costmap_raw 里
             255 = NO_INFORMATION（未知）、254 = LETHAL（障碍本体）、253 = INSCRIBED（内切环）。
           原来用 `d >= 99` 统计会把 **255 未知格**也算成 lethal，属严重口径错误
           （实测同一时刻 254=449、253=1480、而 >=99 记到 3986）。
           现在分开计数：lethal(254) 才是"障碍本体"。"""
        raw_lethal = raw_inscribed = raw_unknown = None
        if self.raw is not None:
            import numpy as np
            d = np.frombuffer(bytes(self.raw.data), dtype=np.uint8)
            raw_lethal = int((d == 254).sum())
            raw_inscribed = int((d == 253).sum())
            raw_unknown = int((d == 255).sum())       # 253/254/255 都在 >=99
        inf_area = all_occ = None
        if self.occ is not None:
            import numpy as np
            d = np.asarray(self.occ.data, dtype=np.int16).reshape(-1)
            all_occ = int((d > 0).sum())
            inf_area = int(((d > 0) & (d < 99)).sum())
        vg = None
        if self.vgrid is not None:
            import numpy as np
            vg = int((np.asarray(self.vgrid.data, dtype=np.int8) > 0).sum())
        return raw_lethal, inf_area, all_occ, vg, raw_inscribed, raw_unknown

    def sample(self, tag, rows, t0):
        rl, ia, ao, vg, ri, ru = self.counts()
        cone_in, cone_out = self.cone_split()
        row = dict(tag=tag, t=round(time.time() - t0, 1), cone_in=cone_in, cone_out=cone_out,
                   inscribed=ri, unknown=ru,
                   yaw=(round(self.yaw, 3) if self.yaw is not None else None),
                   raw_lethal=rl, inf_area=ia, all_occ=ao, vgrid=vg,
                   ratio=(round(ia / rl, 2) if (ia is not None and rl) else None))
        rows.append(row)
        print(f"[local] {tag:>12} t={row['t']:6.1f}s yaw={row['yaw']} "
              f"raw_lethal={rl} inf_area={ia} all_occ={ao} "
              f"lethal254={rl} 内切253={ri} 未知255={ru} ratio={row['ratio']} vgrid={vg} "
              f"锥内={cone_in} 锥外={cone_out}", flush=True)
        return row

    def rotate_by(self, dpsi, t0, rows, tag):
        """偏航反馈旋转：直到累计转够 dpsi（rad）。"""
        yaw_start = self.yaw
        target = yaw_start + dpsi
        t_end = time.time() + self.a.rot_timeout
        tw = Twist()
        sgn = 1.0 if dpsi >= 0 else -1.0
        tw.angular.z = sgn * abs(self.a.rate)
        while time.time() < t_end:
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.yaw is None:
                continue
            delta = 0.0
            d = self.yaw - yaw_start
            # 展开角度
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            delta = d
            if sgn > 0 and (delta >= dpsi * 0.98 or delta < -math.pi + 0.2):
                break
            if sgn < 0 and (delta <= dpsi * 0.98 or delta > math.pi - 0.2):
                break
        self.pub.publish(Twist())
        for _ in range(3):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.sample(tag, rows, t0)

    def run(self):
        t0 = time.time()
        self.setup_subs()
        while time.time() - t0 < self.a.wait:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.occ is not None and self.raw is not None and self.yaw is not None:
                break
        if self.occ is None:
            print('[local] ✗ 拿不到 /local_costmap/costmap')
            return 2
        if self.raw is None:
            print('[local] ⚠ 拿不到 /local_costmap/costmap_raw（nav2_msgs/Costmap），'
                  '只看膨胀层面')
        rows = []
        # 1) 基线
        for i in range(self.a.base_n):
            for _ in range(int(self.a.period * 10)):
                rclpy.spin_once(self, timeout_sec=0.1)
            self.sample(f'base{i}', rows, t0)
        # 2) 连转 N 圈
        for k in range(self.a.turns):
            self.rotate_by(2 * math.pi, t0, rows, f'turn{k}')
            for _ in range(int(self.a.settle * 10)):
                rclpy.spin_once(self, timeout_sec=0.1)
            self.sample(f'end{k}', rows, t0)
        # 3) 转回原朝向
        self.rotate_by(-2 * math.pi * self.a.turns, t0, rows, 'back')
        n = max(1, int(self.a.hold / self.a.period))
        for i in range(n):
            for _ in range(int(self.a.period * 10)):
                rclpy.spin_once(self, timeout_sec=0.1)
            self.sample(f'hold{i:02d}', rows, t0)
        with open(self.a.out, 'w', encoding='utf-8') as f:
            for r in rows:
                f.write(json.dumps(r) + '\n')
        print(f'[local] 输出 {self.a.out}')
        return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--turns', type=int, default=3)
    ap.add_argument('--rate', type=float, default=1.2)
    ap.add_argument('--rot-timeout', type=float, default=60.0)
    ap.add_argument('--settle', type=float, default=10.0)
    ap.add_argument('--hold', type=float, default=40.0)
    ap.add_argument('--period', type=float, default=10.0)
    ap.add_argument('--base-n', type=int, default=3)
    ap.add_argument('--wait', type=float, default=120.0)
    ap.add_argument('--cmd-topic', default='/cmd_vel_nav')
    ap.add_argument('--cone-deg', type=float, default=35.0,
                    help='前方视野锥半角(度)，用于把 lethal 分解成锥内/锥外')
    ap.add_argument('--cone-m', type=float, default=4.5,
                    help='视野锥最大距离(m)')
    ap.add_argument('--out', default='logs/local_residue.jsonl')
    a = ap.parse_args()
    rclpy.init()
    n = LocalResidue(a)
    try:
        rc = n.run()
    finally:
        n.pub.publish(Twist())
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
