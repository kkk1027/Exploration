#!/usr/bin/env python3
"""vision_check.py —— 验证"视觉障碍真的进了 nav2 全局代价地图"（方案A 的关键闭环）。

判据（缺一不可）：
  1. /vision/obstacle_grid 有数据且累积格数 > 0            （感知源活着）
  2. /global_costmap/costmap 在视觉障碍格的位置上是致命(254) （图层注入生效）
  3. 视觉障碍格在代价地图范围内的占比 >= ratio_min          （不是零星巧合）

用法: vision_check.py [--timeout 90] [--ratio-min 0.6] [--json 输出路径]
退出码 0=通过，1=不通过。
"""
import argparse
import json
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class Check(Node):
    def __init__(self):
        super().__init__('vision_check')
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE)
        self.vision = None
        self.costmap = None
        self.create_subscription(OccupancyGrid, '/vision/obstacle_grid', self.on_vision, q)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_costmap, q)

    def on_vision(self, msg):
        self.vision = msg

    def on_costmap(self, msg):
        self.costmap = msg


def grid_to_np(msg):
    g = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
    return g


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--timeout', type=float, default=90.0)
    ap.add_argument('--ratio-min', type=float, default=0.6)
    ap.add_argument('--json', default='')
    a = ap.parse_args()

    rclpy.init()
    n = Check()
    t0 = time.time()
    while time.time() - t0 < a.timeout:
        rclpy.spin_once(n, timeout_sec=0.2)
        if n.vision is not None and n.costmap is not None:
            break
    out = {'vision_seen': n.vision is not None, 'costmap_seen': n.costmap is not None}
    ok = False
    if n.vision is not None and n.costmap is not None:
        v, c = n.vision, n.costmap
        vg, cg = grid_to_np(v), grid_to_np(c)
        v_occ = np.argwhere(vg == 100)
        out['vision_cells'] = int(v_occ.shape[0])
        out['vision_frame'] = v.header.frame_id
        out['costmap_frame'] = c.header.frame_id
        out['costmap_size'] = [int(c.info.width), int(c.info.height)]
        # 视觉格坐标（世界系）→ 代价地图索引
        vr, cr = float(v.info.resolution), float(c.info.resolution)
        vox, voy = float(v.info.origin.position.x), float(v.info.origin.position.y)
        cox, coy = float(c.info.origin.position.x), float(c.info.origin.position.y)
        in_cm = 0
        lethal = 0
        infl = 0
        for (iy, ix) in v_occ:
            wx = vox + (ix + 0.5) * vr
            wy = voy + (iy + 0.5) * vr
            cx = int((wx - cox) / cr)
            cy = int((wy - coy) / cr)
            if 0 <= cx < c.info.width and 0 <= cy < c.info.height:
                in_cm += 1
                val = cg[cy, cx]
                if val == 254:
                    lethal += 1
                elif val > 0:
                    infl += 1
        out['vision_cells_in_costmap'] = in_cm
        out['lethal_at_vision'] = lethal
        out['inflated_at_vision'] = infl
        out['lethal_ratio'] = round(lethal / in_cm, 3) if in_cm else 0.0
        out['covered_ratio'] = round(lethal / v_occ.shape[0], 3) if out['vision_cells'] else 0.0
        ok = (out['vision_cells'] > 0 and in_cm > 0 and out['lethal_ratio'] >= a.ratio_min)
    out['verdict'] = 'PASS' if ok else 'FAIL'
    print(json.dumps(out, ensure_ascii=False, indent=1))
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump(out, f, ensure_ascii=False, indent=1)
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
