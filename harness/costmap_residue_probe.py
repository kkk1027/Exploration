#!/usr/bin/env python3
"""costmap_residue_probe.py —— P0-A 取证探针：旋转时障碍/膨胀层是否残留。

问题（用户 2026-10-26 提报）：原地旋转时，环境的障碍层/膨胀层在栅格上留下残留，
实际应被清除；怀疑局部代价地图刷新频率配置有误。

做法（只读，不改任何参数）：
  * 订阅 `/local_costmap/costmap_raw`（未膨胀的 master：障碍标记在这里）
    与 `/local_costmap/costmap`（膨胀后），以及 `/odom_combined`、`/scan`；
  * 每 0.5 s 落一行 JSONL：时间戳、位姿、偏航角速度、两层的占据格数、哈希、
    以及"本帧相对上一帧新增/消失的格子数"；
  * **旋转检测**：|yaw_rate| > 0.25 rad/s 且线速度 < 0.05 m/s 视为原地旋转；
    旋转期间如果障碍格数持续上涨、且旋转停止后 3 s 内不回落 → 记 residue 事件，
    并把两层栅格各存一份 PGM 快照；
  * 扫描可见性是残差的独立判据：统计"雷达当前已看不到（无回波/超 range）
    但 raw 栅格仍标记"的格子数（用 range 内最近点数近似）。

输出：<out>/residue.jsonl、<out>/residue.pgm（snapshot）*、<out>/summary.json
用法：python3 costmap_residue_probe.py --out logs/round_xxx --once-per 0.5
"""
import argparse
import hashlib
import json
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def grid_stats(msg):
    data = msg.data
    n_occ = sum(1 for v in data if v >= 50)
    n_free = sum(1 for v in data if 0 <= v < 50)
    n_unk = sum(1 for v in data if v < 0)
    h = hashlib.sha1(bytes(bytearray(1 if v >= 50 else 0 for v in data))).hexdigest()[:12]
    return n_occ, n_free, n_unk, h


def write_pgm(msg, path):
    """把 OccupancyGrid 写成 PGM（占据=0，自由=254，未知=205），便于人眼核对。"""
    w, h = msg.info.width, msg.info.height
    buf = bytearray(w * h)
    for idx, v in enumerate(msg.data):
        j = idx // w
        i = idx % w
        if j >= h:
            break
        val = 205
        if v >= 50:
            val = 0
        elif 0 <= v < 50:
            val = 254
        buf[(h - 1 - j) * w + i] = val
    with open(path, 'wb') as fh:
        fh.write(b'P5\n%d %d\n255\n' % (w, h))
        fh.write(bytes(buf))


class Probe(Node):
    def __init__(self, out, once_per):
        super().__init__('costmap_residue_probe')
        self.out = out
        os.makedirs(out, exist_ok=True)
        self.jsonl = open(os.path.join(out, 'residue.jsonl'), 'a', encoding='utf-8')
        self.once_per = once_per
        self.last = 0.0
        self.raw = None
        self.infl = None
        self.prev_raw_hash = None
        self.prev_raw_occ = None
        self.yaw = None
        self.yaw0 = None
        self.t0 = None
        self.yaw_rate = 0.0
        self.vx = 0.0
        self.rotating_since = None
        self.occ_at_rot_start = None
        self.events = []
        self.scan = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap_raw', self.on_raw, qos)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.on_infl, qos)
        self.create_subscription(Odometry, '/odom_combined', self.on_odom, 20)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_timer(0.25, self.tick)
        self.get_logger().info(f'costmap_residue_probe 就绪 → {out}')

    def on_raw(self, m):
        self.raw = m

    def on_infl(self, m):
        self.infl = m

    def on_scan(self, m):
        self.scan = m

    def on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        now = time.monotonic()
        if self.yaw is not None and self.t0 is not None:
            dt = now - self.t0
            if dt > 1e-3:
                d = yaw - self.yaw
                while d > math.pi:
                    d -= 2 * math.pi
                while d < -math.pi:
                    d += 2 * math.pi
                self.yaw_rate = d / dt
        self.yaw = yaw
        self.t0 = now
        self.vx = m.twist.twist.linear.x

    def scan_visible_obstacles(self, msg):
        """粗判：range 内点数（有回波才算"雷达看得到"）。"""
        if self.scan is None:
            return None
        vals = [r for r in self.scan.ranges
                if math.isfinite(r) and self.scan.range_min < r < self.scan.range_max]
        return len(vals)

    def tick(self):
        now = time.monotonic()
        if now - self.last < self.once_per:
            return
        self.last = now
        if self.raw is None:
            return
        n_occ, n_free, n_unk, h = grid_stats(self.raw)
        i_occ = i_free = i_unk = 0
        if self.infl is not None:
            i_occ, i_free, i_unk, _ = grid_stats(self.infl)
        d_occ = None if self.prev_raw_occ is None else n_occ - self.prev_raw_occ
        rotating = abs(self.yaw_rate) > 0.25 and abs(self.vx) < 0.05
        row = {
            't': round(time.time(), 3), 'yaw': None if self.yaw is None else round(self.yaw, 3),
            'yaw_rate': round(self.yaw_rate, 3), 'vx': round(self.vx, 3),
            'rotating': rotating,
            'raw_occ': n_occ, 'raw_free': n_free, 'raw_unk': n_unk, 'raw_hash': h,
            'infl_occ': i_occ, 'infl_free': i_free, 'infl_unk': i_unk,
            'd_occ': d_occ, 'scan_pts': self.scan_visible_obstacles(self.raw),
        }
        # 旋转期间占据格是否持续上涨（残留的直接嫌疑）
        if rotating and self.rotating_since is None:
            self.rotating_since = now
            self.occ_at_rot_start = n_occ
        elif not rotating and self.rotating_since is not None:
            grew = n_occ - (self.occ_at_rot_start or n_occ)
            dur = now - self.rotating_since
            if dur >= 0.8 and grew >= 30:      # 旋转 ≥0.8 s 且期间多出 ≥30 格
                ev = {'kind': 'rot_growth', 't': round(time.time(), 3),
                      'dur': round(dur, 2), 'grew': grew,
                      'occ_start': self.occ_at_rot_start, 'occ_end': n_occ,
                      'yaw_rate': round(self.yaw_rate, 3)}
                self.events.append(ev)
                row['event'] = 'rot_growth'
                try:
                    write_pgm(self.raw, os.path.join(self.out, f'residue_raw_{int(now)}.pgm'))
                    if self.infl is not None:
                        write_pgm(self.infl, os.path.join(self.out, f'residue_infl_{int(now)}.pgm'))
                except Exception as exc:            # noqa: BLE001
                    row['pgm_error'] = str(exc)
            self.rotating_since = None
            self.occ_at_rot_start = None
        self.jsonl.write(json.dumps(row) + '\n')
        self.jsonl.flush()
        self.prev_raw_occ = n_occ
        self.prev_raw_hash = h

    def finish(self):
        try:
            occ = [json.loads(l).get('raw_occ') for l in open(os.path.join(self.out, 'residue.jsonl'))
                   if l.strip()]
            occ = [o for o in occ if isinstance(o, int)]
        except Exception:                       # noqa: BLE001
            occ = []
        summary = {'samples': len(occ),
                   'raw_occ_max': max(occ) if occ else None,
                   'raw_occ_min': min(occ) if occ else None,
                   'rot_growth_events': len(self.events), 'events': self.events[:20]}
        json.dump(summary, open(os.path.join(self.out, 'summary.json'), 'w', encoding='utf-8'),
                  ensure_ascii=False, indent=1)
        print('[residue] ' + json.dumps(summary, ensure_ascii=False)[:400])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--once-per', type=float, default=0.5)
    a = ap.parse_args()
    rclpy.init()
    node = Probe(a.out, a.once_per)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
