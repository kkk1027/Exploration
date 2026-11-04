#!/usr/bin/env python3
"""verify_goal_updater.py —— 证明 nav2 BT 的 GoalUpdater 真的消费了 /goal_update。

背景：`/goal_update` 有 1 pub / 1 sub 只能说明"话题对上了"，不能说明 BT 真的用它
重规划了。本脚本做注入式验证：

  1. 订阅 `/plan`（nav_msgs/Path），等一条计划；
  2. 取该计划末点 P_end，构造注入点 P_inj = P_end 朝车方向回退 1.0 m
     （保证 P_inj 在已知空闲区、离车更近，规划必可达）；
  3. 发布 geometry_msgs/PoseStamped 到 `/goal_update`（stamp=now）；
  4. 之后观察 6 s 内的新 `/plan`：
       - 末点与并 P_inj 的距离 ≤ 0.5 m ⇒ **PASS（BT 消费成功）**
       - 末点仍在 P_end 附近（>1.5 m 差）⇒ **FAIL（BT 没消费）**

用法（栈在跑、且有活跃导航目标时）：
  python3 harness/verify_goal_updater.py [--topic /goal_update] [--timeout 12]
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class Verifier(Node):
    def __init__(self, a):
        super().__init__('verify_goal_updater')
        self.a = a
        self.plan = None
        self.plans_after = []
        self.pos = None
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE)
        # /plan 是 VOLATILE 发布（RELIABLE）；订阅端要 TRANSIENT_LOCAL 会 QoS 不兼容
        qm = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Path, '/plan', self.on_plan, qm)
        self.create_subscription(Odometry, '/odom_combined', self.on_odom, 20)
        self.pub = self.create_publisher(PoseStamped, a.topic, 10)

    def on_plan(self, m):
        if self.plan is None:
            self.plan = m
        else:
            self.plans_after.append((time.time(), m))

    def on_odom(self, m):
        self.pos = (m.pose.pose.position.x, m.pose.pose.position.y)

    def run(self):
        t0 = time.time()
        while time.time() - t0 < self.a.timeout and (self.plan is None or self.pos is None):
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.plan is None or self.pos is None:
            print('[verify] ✗ 拿不到 /plan 或 /odom_combined（需要有活跃导航目标）')
            return 2
        end = self.plan.poses[-1].pose.position
        p_end = (end.x, end.y)
        # 注入点：从计划末点朝车方向回退 1.0 m，并夹在离车 ≥1.0 m
        dx, dy = self.pos[0] - p_end[0], self.pos[1] - p_end[1]
        norm = math.hypot(dx, dy) or 1.0
        # 沿"垂直于车→末点方向"偏移，避免注入点正好落在原路径上（那样看不出重规划）
        ux, uy = dx / norm, dy / norm
        p_inj = (self.pos[0] + (-uy) * self.a.offset, self.pos[1] + (ux) * self.a.offset)
        print(f'[verify] 当前计划末点={p_end[0]:.2f},{p_end[1]:.2f}  车={self.pos[0]:.2f},{self.pos[1]:.2f}')
        print(f'[verify] 注入 /goal_update → {p_inj[0]:.2f},{p_inj[1]:.2f}')
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = p_inj
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)

        # 再补发一次（有些实现会因首帧丢失/时序错过而忽略，补发可排除偶发）
        t_hold = time.time()
        while time.time() - t_hold < 2.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        print('[verify] 已补发第二次 /goal_update')
        t_inj = time.time()
        while time.time() - t_inj < self.a.observe:
            rclpy.spin_once(self, timeout_sec=0.2)
        best = None
        for _t, p in self.plans_after:
            if not p.poses:
                continue
            q = p.poses[-1].pose.position
            d = math.hypot(q.x - p_inj[0], q.y - p_inj[1])
            if best is None or d < best[0]:
                best = (d, (q.x, q.y))
        if best is None:
            print('[verify] ✗ 注入后没有新计划（可能路径被判有效、未触发重规划）')
            return 3
        ok = best[0] <= 0.50
        print(f'[verify] 注入后 {len(self.plans_after)} 条计划；'
              f'最接近注入点的末点={best[1][0]:.2f},{best[1][1]:.2f} 距离={best[0]:.2f} m')
        print('[verify] ' + ('✓ PASS：BT 已消费 /goal_update（重规划到注入点）' if ok
                            else '✗ FAIL：BT 未消费 /goal_update（末点没变）'))
        return 0 if ok else 4


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/goal_update')
    ap.add_argument('--timeout', type=float, default=20.0)
    ap.add_argument('--observe', type=float, default=6.0)
    ap.add_argument('--offset', type=float, default=2.5,
                    help='注入点相对车体的垂直偏移(m)')
    a = ap.parse_args()
    rclpy.init()
    n = Verifier(a)
    try:                                   # 与栈同用仿真时钟，排除 stamp 比较的时钟差
        from rcl_interfaces.msg import Parameter as _P
        from rclpy.parameter import Parameter as _RP
        n.set_parameters([_RP('use_sim_time', _RP.Type.BOOL, True)])
    except Exception as _e:                 # noqa: BLE001
        print(f'[verify] use_sim_time 设置失败：{_e}')
    try:
        rc = n.run()
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == '__main__':
    main()
