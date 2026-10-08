"""bear_proximity_monitor ノード(レベル1の熊検知).

レベル1ではカメラ画像による熊判定の代わりに、Gazebo 上の熊の既知座標
(world_layout.yaml の animals)と、TF から得たロボット位置との距離で「熊の近くに
到達した」ことを判定する。レベル2で bear_detector(カメラ + AI)に置き換える前提で、
出力は同じ kumawatch_interfaces/BearDetection とする。

出力: /kumawatch/bear_detection (熊が detection_radius 以内にいる間、rate Hz で配信)
"""

import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from kumawatch_interfaces.msg import BearDetection
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from kumawatch_mission.route_planning import nearest_within


class BearProximityMonitor(Node):

    def __init__(self):
        super().__init__('bear_proximity_monitor')
        layout = os.path.join(get_package_share_directory('kumawatch_gazebo'), 'config',
                              'world_layout.yaml')
        self.declare_parameter('layout_file', layout)
        self.declare_parameter('detection_radius', 2.5)
        self.declare_parameter('rate', 5.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')

        with open(self.get_parameter('layout_file').value, encoding='utf-8') as f:
            animals = yaml.safe_load(f).get('animals', [])
        self.bears = [a for a in animals if a.get('species') == 'bear']
        self.radius = float(self.get_parameter('detection_radius').value)
        self.get_logger().info('既知の熊 %d 頭 (検知半径 %.1f m): %s' % (
            len(self.bears), self.radius,
            ', '.join('%s(%.1f, %.1f)' % (b['id'], b['x'], b['y']) for b in self.bears)))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(BearDetection, '/kumawatch/bear_detection', 10)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._check)
        self.tf_warned = False

    def _check(self):
        map_frame = self.get_parameter('map_frame').value
        robot_frame = self.get_parameter('robot_frame').value
        try:
            tf = self.tf_buffer.lookup_transform(map_frame, robot_frame, Time())
        except TransformException as e:
            if not self.tf_warned:
                self.get_logger().info('ロボット位置(TF %s→%s)を待っています: %s' % (
                    map_frame, robot_frame, e))
                self.tf_warned = True
            return
        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        hit = nearest_within((rx, ry), self.bears, self.radius)
        if hit is None:
            return
        bear, dist = hit
        msg = BearDetection()
        msg.header.stamp = tf.header.stamp
        msg.header.frame_id = map_frame
        msg.detected = True
        msg.bear_id = bear['id']
        msg.robot_position = Point(x=rx, y=ry, z=0.0)
        msg.bear_position = Point(x=float(bear['x']), y=float(bear['y']), z=0.0)
        msg.distance = float(dist)
        msg.confidence = 1.0  # 既知座標による判定のため
        msg.source = 'ground_truth_proximity'
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BearProximityMonitor()
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
