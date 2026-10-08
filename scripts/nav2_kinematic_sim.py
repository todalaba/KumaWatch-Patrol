#!/usr/bin/env python3
"""Gazebo なしで本物の Nav2 を動かすための簡易シミュレータ(開発・検証用).

world_layout.yaml(静的障害物 + 動物)を占有格子にして、差動二輪ロボットを /cmd_vel で
動かし、2D LiDAR(/scan)をレイキャストで模擬する。/clock・/odom・TF
(odom→base_footprint→base_link→base_scan)を配信するので、Nav2(AMCL・計画・DWB)と
KumaWatch Patrol のノードを use_sim_time:=true のまま動かせる。

Gazebo の代わりにはならない(物理・カメラなし)。GPU のない環境や CI で
nav2_params.yaml・地図・ミッションの流れを確認する用途。

  python3 scripts/nav2_kinematic_sim.py [--time-scale 2.0]
  ros2 launch kumawatch_navigation navigation.launch.py rviz:=false
  ros2 launch kumawatch_mission patrol_nodes.launch.py
"""

import argparse
import math
import os
import sys

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'kumawatch_gazebo'))
from kumawatch_gazebo import world_builder as wb  # noqa: E402

LAYOUT = os.path.join(ROOT, 'src', 'kumawatch_gazebo', 'config', 'world_layout.yaml')
POINTS = os.path.join(ROOT, 'src', 'kumawatch_ai', 'data', 'patrol_points.yaml')


class KinematicSim(Node):

    def __init__(self, time_scale):
        super().__init__('nav2_kinematic_sim')
        layout = wb.load_yaml(LAYOUT)
        with open(POINTS, encoding='utf-8') as f:
            base = yaml.safe_load(f)['base']
        self.grid = wb.rasterize(layout, extra_shapes=[
            wb.animal_footprint(a) for a in layout.get('animals', [])])
        self.res = layout['map_resolution']
        self.bx = layout['bounds']['x_min']
        self.by = layout['bounds']['y_min']
        self.h = len(self.grid)
        self.w = len(self.grid[0])
        self.x, self.y, self.th = float(base['x']), float(base['y']), float(base.get('yaw', 0.0))
        self.v = self.wz = 0.0
        self.last_cmd = 0.0
        self.t = 0.0
        self.dt = 0.02
        self.scale = time_scale
        self.steps = 0

        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self._publish_static()
        self.create_timer(self.dt / self.scale, self._step)
        self.get_logger().info('kinematic sim at (%.2f, %.2f), time x%.1f' % (
            self.x, self.y, self.scale))

    def _stamp(self):
        sec = int(self.t)
        return Time(sec=sec, nanosec=int((self.t - sec) * 1e9))

    def _publish_static(self):
        out = []
        for parent, child, z in (('base_footprint', 'base_link', 0.01),
                                 ('base_link', 'base_scan', 0.17)):
            t = TransformStamped()
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.z = z
            t.transform.rotation.w = 1.0
            out.append(t)
        self.static_tf.sendTransform(out)

    def _on_cmd(self, msg):
        self.v = max(-0.26, min(0.26, msg.linear.x))
        self.wz = max(-1.82, min(1.82, msg.angular.z))
        self.last_cmd = self.t

    def _occupied(self, x, y):
        c = int((x - self.bx) / self.res)
        r = int((y - self.by) / self.res)
        if not (0 <= r < self.h and 0 <= c < self.w):
            return True
        return self.grid[r][c] != wb.FREE

    def _step(self):
        self.t += self.dt
        if self.t - self.last_cmd > 0.5:
            self.v = self.wz = 0.0
        nx = self.x + self.v * math.cos(self.th) * self.dt
        ny = self.y + self.v * math.sin(self.th) * self.dt
        # 衝突したら並進を止める(ロボット半径 0.2 m 相当の簡易判定)
        if not any(self._occupied(nx + 0.2 * math.cos(a), ny + 0.2 * math.sin(a))
                   for a in (i * math.pi / 4 for i in range(8))):
            self.x, self.y = nx, ny
        self.th = math.atan2(math.sin(self.th + self.wz * self.dt),
                             math.cos(self.th + self.wz * self.dt))
        stamp = self._stamp()
        self.clock_pub.publish(Clock(clock=stamp))

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.th / 2)
        t.transform.rotation.w = math.cos(self.th / 2)
        self.tf.sendTransform(t)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)

        self.steps += 1
        if self.steps % 10 == 0:  # 5 Hz
            self._publish_scan(stamp)
        if self.steps % 250 == 0:  # 5 s ごとに真の位置を記録
            self.get_logger().info('t=%.1f true pose (%.2f, %.2f, %.2f) v=%.2f w=%.2f' % (
                self.t, self.x, self.y, self.th, self.v, self.wz))

    def _publish_scan(self, stamp):
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'base_scan'
        n = 360
        scan.angle_min = 0.0
        scan.angle_max = 2 * math.pi * (n - 1) / n
        scan.angle_increment = 2 * math.pi / n
        scan.range_min = 0.12
        scan.range_max = 3.5
        scan.scan_time = 0.2
        ranges = []
        step = self.res * 0.8
        for i in range(n):
            a = self.th + i * scan.angle_increment
            ca, sa = math.cos(a), math.sin(a)
            r = scan.range_min
            hit = float('inf')
            while r <= scan.range_max:
                if self._occupied(self.x + r * ca, self.y + r * sa):
                    hit = r
                    break
                r += step
            ranges.append(hit)
        scan.ranges = ranges
        self.scan_pub.publish(scan)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--time-scale', type=float, default=1.0)
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = KinematicSim(args.time_scale)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
