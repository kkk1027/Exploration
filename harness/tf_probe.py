#!/usr/bin/env python3
"""tf_probe.py —— 复刻 vision_obstacle_grid 的三种 TF 取法，逐一打印异常类型。

背景（2026-10-28 取证）：
  nav_only 里视觉节点 100% TF 失败，但它的 tf buffer 明明有 map：
     Frame odom_combined exists with parent map.
  而同一时刻 `ros2 run tf2_ros tf2_echo map odom_combined` 又能取到。
  ⇒ 失败不是"帧不存在"，是**时间轴对不上**。本探针把三种取法分开量：
     ① 按点云 stamp 取（vision 现在的首选路径）
     ② 取最新可用 Time()（tf2_echo 等价，也是 vision 原版行为）
     ③ 取时钟 now()（vision 现在的 fail-soft 回退）
  并在每帧打出"最新可用时间 vs 仿真 now"的差，直接看 AMCL 多久没发 TF。

用法（栈已经起来时）：
    source /tmp/renv.sh; python3 harness/tf_probe.py [目标帧] [源帧] [采样帧数]
默认：目标 map、源 camera_depth_optical_frame、采 3 帧点云。
"""
import sys

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener


def fmt(tr):
    t = tr.transform.translation
    return f'OK  t=({t.x:+.3f},{t.y:+.3f},{t.z:+.3f}) stamp={tr.header.stamp.sec}.{tr.header.stamp.nanosec:09d}'


class Probe(Node):
    def __init__(self, target: str, source: str, n: int):
        super().__init__('tf_probe')
        self.target, self.source, self.need = target, source, n
        self.got = 0
        # 必须和 vision_obstacle_grid 一样用仿真时钟，否则 now() 是墙钟（实测会差 1.8e9 s）
        try:
            from rclpy.parameter import Parameter
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        except Exception as e:                                   # noqa: BLE001
            print(f'[probe] 设 use_sim_time 失败: {e}', flush=True)
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.create_subscription(PointCloud2, '/camera/depth/points', self.on_cloud, 2)

    def try_one(self, tag, fn):
        try:
            print(f'  {tag}: {fmt(fn())}', flush=True)
            return True
        except Exception as e:                                   # noqa: BLE001
            print(f'  {tag}: {type(e).__name__}: {e}', flush=True)
            return False

    def on_cloud(self, msg):
        now = self.get_clock().now()
        stamp = Time.from_msg(msg.header.stamp)
        print(f'\n[帧 {self.got + 1}] 点云 stamp={stamp.nanoseconds * 1e-9:.3f}  '
              f'sim now={now.nanoseconds * 1e-9:.3f}  '
              f'年龄={(now.nanoseconds - stamp.nanoseconds) * 1e-9:+.3f}s  '
              f'frame={msg.header.frame_id}', flush=True)
        # 最新可用时间（Time() = 0 ⇒ latest common time）
        try:
            latest = self.buf.lookup_transform(self.target, self.source, Time())
            lt = latest.header.stamp
            gap = now.nanoseconds * 1e-9 - (lt.sec + lt.nanosec * 1e-9)
            print(f'  最新可用 TF 时间={lt.sec}.{lt.nanosec:09d}  '
                  f'距 sim now={gap:+.3f}s  ⇒ 若 gap>0.1 s，任何以 now 为界的取法都会抛'
                  f'ExtrapolationException(future)', flush=True)
        except Exception as e:                                   # noqa: BLE001
            print(f'  最新可用 TF: {type(e).__name__}: {e}', flush=True)

        self.try_one('①按点云 stamp (timeout=0.10)', lambda: self.buf.lookup_transform(
            self.target, self.source, stamp, timeout=Duration(seconds=0.10)))
        self.try_one('②最新可用 Time()      ', lambda: self.buf.lookup_transform(
            self.target, self.source, Time()))
        self.try_one('③时钟 now()（回退路径）', lambda: self.buf.lookup_transform(
            self.target, self.source, now))
        self.try_one('③b now()+timeout 0.10 ', lambda: self.buf.lookup_transform(
            self.target, self.source, now, timeout=Duration(seconds=0.10)))
        self.got += 1
        if self.got >= self.need:
            raise SystemExit(0)


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else 'map'
    source = sys.argv[2] if len(sys.argv) > 2 else 'camera_depth_optical_frame'
    n = int(sys.argv[3]) if len(sys.argv) > 3 else 3
    rclpy.init()
    node = Probe(target, source, n)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
