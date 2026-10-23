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
        self.subs = {}
        self.pubs = {}
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE)
        self.vision = None
        self.costmap = None
        self.create_subscription(OccupancyGrid, '/vision/obstacle_grid', self.on_vision, q)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_costmap, q)
        # 诊断：谁在订阅视觉障碍云（nav2 视觉图层应当出现在这里）
        self.create_timer(1.0, self.sniff)

    def on_vision(self, msg):
        self.vision = msg

    def on_costmap(self, msg):
        self.costmap = msg

    def sniff(self):
        try:
            self.subs['obstacle_cloud'] = len(self.get_subscriptions_info_by_topic('/vision/obstacle_cloud'))
            self.pubs['obstacle_cloud'] = len(self.get_publishers_info_by_topic('/vision/obstacle_cloud'))
            self.subs['live_cloud'] = len(self.get_subscriptions_info_by_topic('/vision/live_cloud'))
            self.pubs['live_cloud'] = len(self.get_publishers_info_by_topic('/vision/live_cloud'))
        except Exception as e:
            self.subs['err'] = str(e)


def grid_to_np(msg):
    """OccupancyGrid.data 是 int8[]：nav2 的 254(致命)/253(内切)/255(未知) 到 Python 侧
    分别是 -2/-3/-1。必须先按 int8 读出来再抬到 0..255，否则会漏掉所有致命格。"""
    g = np.array(msg.data, dtype=np.int8).astype(np.int16)
    mask = g < 0
    if mask.any():
        g[mask] += 256
    return g.reshape(msg.info.height, msg.info.width)


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
        # ① 值直方图（只统计落在代价地图范围内的视觉格）
        vals = []
        for (iy, ix) in v_occ:
            wx = vox + (ix + 0.5) * vr
            wy = voy + (iy + 0.5) * vr
            cx = int((wx - cox) / cr)
            cy = int((wy - coy) / cr)
            if 0 <= cx < c.info.width and 0 <= cy < c.info.height:
                vals.append(int(cg[cy, cx]))
        hist = {}
        for val in vals:
            k = '254' if val == 254 else ('253' if val == 253 else ('0' if val == 0 else '1-252'))
            hist[k] = hist.get(k, 0) + 1
        out['value_hist'] = hist

        # ② 坐标偏移搜索：若存在系统性半格错位，会在这里暴露
        best = (0.0, None)
        for ddx in range(-3, 4):
            for ddy in range(-3, 4):
                hit = 0
                for (iy, ix) in v_occ:
                    cx = int((vox + (ix + 0.5) * vr - cox) / cr) + ddx
                    cy = int((voy + (iy + 0.5) * vr - coy) / cr) + ddy
                    if 0 <= cx < c.info.width and 0 <= cy < c.info.height and cg[cy, cx] == 254:
                        hit += 1
                r = hit / max(1, v_occ.shape[0])
                if r > best[0]:
                    best = (round(r, 3), [ddx, ddy])
        out['best_offset'] = best

        # ③ 纯视觉格：视觉有、而 0.15 m 内没有任何致命格 —— 这才是"视觉层真的在起作用"
        lethal_idx = np.argwhere(cg == 254)
        if lethal_idx.size:
            pw = np.stack([cox + (lethal_idx[:, 1] + 0.5) * cr,
                           coy + (lethal_idx[:, 0] + 0.5) * cr], axis=1)
        else:
            pw = np.zeros((0, 2))
        vw = np.stack([vox + (v_occ[:, 1] + 0.5) * vr, voy + (v_occ[:, 0] + 0.5) * vr], axis=1)
        if pw.shape[0] and vw.shape[0]:
            d2 = ((vw[:, None, :] - pw[None, :, :]) ** 2).sum(axis=2)
            nearest = np.sqrt(d2.min(axis=1))
            solo = nearest > 0.15
            out['vision_only_cells'] = int(solo.sum())
            solo_lethal = 0
            for (wx, wy) in vw[solo]:
                cx = int((wx - cox) / cr); cy = int((wy - coy) / cr)
                if 0 <= cx < c.info.width and 0 <= cy < c.info.height and cg[cy, cx] == 254:
                    solo_lethal += 1
            out['vision_only_lethal'] = solo_lethal
            out['vision_only_lethal_ratio'] = round(solo_lethal / max(1, int(solo.sum())), 3)
        out['vision_cells_in_costmap'] = in_cm
        out['lethal_at_vision'] = lethal
        out['inflated_at_vision'] = infl
        out['lethal_ratio'] = round(lethal / in_cm, 3) if in_cm else 0.0
        out['covered_ratio'] = round(lethal / v_occ.shape[0], 3) if out['vision_cells'] else 0.0
        solo_ratio = out.get('vision_only_lethal_ratio', 0.0)
        ok = (out['vision_cells'] > 0 and in_cm > 0 and
              (out['lethal_ratio'] >= a.ratio_min or solo_ratio >= a.ratio_min))
    out['topic_diag'] = {'subs': n.subs, 'pubs': n.pubs}
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
