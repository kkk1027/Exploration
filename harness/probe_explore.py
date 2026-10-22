#!/usr/bin/env python3
"""probe_explore.py —— 探索效率取证：RRT 链各级候选数、被接受目标、地图已知比例、位姿。"""
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from wheeltec_rrt_msg.msg import PointArray

class P(Node):
    def __init__(self):
        super().__init__('probe_explore')
        self.det = 0; self.flt = 0; self.assigned = 0; self.goals = []
        self.map = None; self.pose = None
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, q)
        self.create_subscription(PointArray, '/detected_frontiers', self.on_det, 10)
        self.create_subscription(PointArray, '/filtered_goal_points', self.on_flt, 10)
        self.create_subscription(PoseStamped, '/assigned_frontier_goal', self.on_asg, 10)
        self.create_subscription(Odometry, '/odom_combined', self.on_odom, 20)
    def on_map(self, m): self.map = m
    def on_det(self, m):
        try: self.det += len(m.points)
        except Exception: self.det += 1
    def on_flt(self, m):
        try: self.flt += len(m.points)
        except Exception: self.flt += 1
    def on_asg(self, m): self.assigned += 1; self.goals.append((time.monotonic(), m.pose.position.x, m.pose.position.y))
    def on_odom(self, m): self.pose = (m.pose.pose.position.x, m.pose.pose.position.y)

rclpy.init(); p = P()
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
t0 = time.monotonic(); last = (0,0,0)
while time.monotonic() - t0 < dur:
    for _ in range(25):
        rclpy.spin_once(p, timeout_sec=0.1)
    known = unk = 0
    if p.map is not None:
        for v in p.map.data:
            if v >= 0: known += 1
            else: unk += 1
    tot = known + unk
    ratio = 100.0 * known / tot if tot else 0.0
    dw = p.det - last[0]; fw = p.flt - last[1]; aw = p.assigned - last[2]
    last = (p.det, p.flt, p.assigned)
    pose = f'({p.pose[0]:.1f},{p.pose[1]:.1f})' if p.pose else '?'
    print(f'  [t+{time.monotonic()-t0:5.1f}s] 已知 {ratio:5.1f}% | detected +{dw} filtered +{fw} assigned +{aw} '
          f'(累计 {p.det}/{p.flt}/{p.assigned}) | 位姿 {pose}', flush=True)
p.destroy_node(); rclpy.shutdown()
