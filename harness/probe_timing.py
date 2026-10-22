#!/usr/bin/env python3
"""probe_timing.py —— 1 Hz 采样位姿/速度/目标，供后续做"探索时间占用分布"统计。"""
import sys, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped

class P(Node):
    def __init__(self):
        super().__init__('probe_timing')
        self.pose=None; self.cmd=None; self.goal_ts=0; self.ngoal=0
        self.create_subscription(Odometry,'/odom_combined',self.o,20)
        self.create_subscription(Twist,'/cmd_vel',self.c,20)
        self.create_subscription(PoseStamped,'/validated_frontier_goal',self.g,10)
        self.create_subscription(PoseStamped,'/assigned_frontier_goal',self.g2,10)
    def o(self,m): self.pose=(m.pose.pose.position.x,m.pose.pose.position.y,
                              m.twist.twist.linear.x,m.twist.twist.angular.z)
    def c(self,m): self.cmd=(m.linear.x,m.angular.z)
    def g(self,m): self.goal_ts=time.monotonic(); self.ngoal+=1
    def g2(self,m): pass

rclpy.init(); p=P()
dur=float(sys.argv[1]) if len(sys.argv)>1 else 200.0
t0=time.monotonic(); nxt=t0
print('t,x,y,vx,wz,cmd_vx,cmd_wz,goal_age_s', flush=True)
while time.monotonic()-t0<dur:
    rclpy.spin_once(p,timeout_sec=0.1)
    if time.monotonic()>=nxt:
        nxt+=1.0
        if p.pose:
            x,y,vx,wz=p.pose
            cv,cw=(p.cmd if p.cmd else (0.0,0.0))
            age=(time.monotonic()-p.goal_ts) if p.goal_ts else -1
            print(f'{time.monotonic()-t0:.1f},{x:.3f},{y:.3f},{vx:+.3f},{wz:+.3f},{cv:+.3f},{cw:+.3f},{age:.1f}', flush=True)
p.destroy_node(); rclpy.shutdown()
