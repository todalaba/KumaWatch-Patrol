"""bear_detector ノード: カメラ画像から YOLO で熊を検出する(AI による熊判定).

ロボットのカメラ画像を YOLOv8(Gazebo の熊で追加学習したモデル)で判定し、
熊の方位を検出枠から、距離を LiDAR から求めて、地図上の熊の位置を推定する。
数フレーム連続で検出され、かつ report_distance 以内に近づいたときに
kumawatch_interfaces/BearDetection を配信する(mission_planner が停止・記録する)。

入力:  /camera/image_raw, /camera/camera_info, /scan, TF(map → base_footprint)
出力:  /kumawatch/bear_detection(BearDetection, source="camera_yolo")
       /kumawatch/detection_image(検出枠を描いた画像。動画撮影用)
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
import cv2
from geometry_msgs.msg import Point
from kumawatch_interfaces.msg import BearDetection
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from kumawatch_ai.bear_localization import (
    bearing_from_bbox, BearTracker, estimate_range, focal_length_px, range_from_bbox_height,
    range_from_scan, to_map)
from kumawatch_ai.image_utils import bgr_to_image_msg, image_to_bgr, matched_image_qos


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class BearDetector(Node):

    def __init__(self):
        super().__init__('bear_detector')
        models = os.path.join(get_package_share_directory('kumawatch_ai'), 'models')
        self.declare_parameter('weights', os.path.join(models, 'kumawatch_bear_yolov8n.pt'))
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('imgsz', 416)
        self.declare_parameter('rate', 5.0)                 # 推論の頻度 [Hz]
        self.declare_parameter('min_consecutive', 3)        # 連続検出フレーム数で確定
        self.declare_parameter('report_distance', 4.0)      # この距離以内なら停止させる [m]
        self.declare_parameter('hfov', 1.02974)             # camera_info が無いときの水平画角
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('device', 'cpu')

        weights = self.get_parameter('weights').value
        try:
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().fatal('ultralytics がありません: pip3 install ultralytics')
            raise
        if not os.path.exists(weights):
            self.get_logger().fatal('重みファイルがありません: %s' % weights)
            raise FileNotFoundError(weights)
        self.model = YOLO(weights)
        names = self.model.names
        self.bear_classes = [i for i, n in names.items() if n == 'bear']
        if not self.bear_classes:
            raise ValueError('モデルに bear クラスがありません: %s' % names)
        self.get_logger().info('YOLO を読み込みました: %s (bear = class %s)' % (
            os.path.basename(weights), self.bear_classes))

        self.tracker = BearTracker(min_hits=int(self.get_parameter('min_consecutive').value))
        self.reported = set()
        self.latest_image = None
        self.fx = self.fy = self.cx = None
        self.scan = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # カメラ画像は、送る側の QoS(Gazebo は Reliable)に合わせて購読する
        self.image_sub = None
        self.sub_timer = self.create_timer(0.5, self._try_subscribe)
        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value,
                                 self._on_info, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 self._on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(BearDetection, '/kumawatch/bear_detection', 10)
        self.img_pub = self.create_publisher(Image, '/kumawatch/detection_image', 2)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._process)

    def _try_subscribe(self):
        topic = self.get_parameter('image_topic').value
        qos = matched_image_qos(self, topic, depth=1)
        if qos is None:
            return
        self.image_sub = self.create_subscription(Image, topic, self._on_image, qos)
        self.sub_timer.cancel()
        self.get_logger().info('%s を購読しました(%s)' % (topic, qos.reliability.name))

    def _on_image(self, msg):
        self.latest_image = msg

    def _on_info(self, msg):
        if msg.k[0] > 0:
            self.fx, self.fy, self.cx = msg.k[0], msg.k[4], msg.k[2]

    def _on_scan(self, msg):
        self.scan = msg

    def _robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.get_parameter('map_frame').value,
                                                 self.get_parameter('robot_frame').value, Time())
        except TransformException:
            return None
        t = tf.transform
        return t.translation.x, t.translation.y, yaw_of(t.rotation)

    def _process(self):
        msg = self.latest_image
        if msg is None:
            return
        self.latest_image = None
        try:
            img = image_to_bgr(msg)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=10.0)
            return
        h, w = img.shape[:2]
        fx = self.fx or focal_length_px(w, float(self.get_parameter('hfov').value))
        fy = self.fy or fx
        cx = self.cx or w / 2.0
        result = self.model.predict(img, imgsz=int(self.get_parameter('imgsz').value),
                                    conf=float(self.get_parameter('conf_threshold').value),
                                    classes=self.bear_classes, verbose=False,
                                    device=self.get_parameter('device').value)[0]
        pose = self._robot_pose()
        detections = []
        drawn = []
        for box, conf in zip(result.boxes.xyxy.tolist(), result.boxes.conf.tolist()):
            x0, y0, x1, y1 = box
            bearing = bearing_from_bbox((x0 + x1) / 2.0, w, fx, cx)
            r_box = range_from_bbox_height(y1 - y0, fy)
            r_scan = None
            if self.scan is not None:
                s = self.scan
                r_scan = range_from_scan(list(s.ranges), s.angle_min, s.angle_increment, bearing,
                                         range_min=s.range_min, range_max=s.range_max)
            dist = estimate_range(r_scan, r_box)
            drawn.append((x0, y0, x1, y1, conf, dist))
            if pose is not None and math.isfinite(dist):
                bx, by = to_map(pose[0], pose[1], pose[2], bearing, dist)
                detections.append((bx, by, conf, dist))

        confirmed = self.tracker.update([(d[0], d[1], d[2]) for d in detections]) if pose else []
        limit = float(self.get_parameter('report_distance').value)
        for t in confirmed:
            dist = math.hypot(t['x'] - pose[0], t['y'] - pose[1])
            if dist > limit:
                continue
            det = BearDetection()
            det.header = msg.header
            det.header.frame_id = self.get_parameter('map_frame').value
            det.detected = True
            det.bear_id = t['id']
            det.robot_position = Point(x=pose[0], y=pose[1], z=0.0)
            det.bear_position = Point(x=float(t['x']), y=float(t['y']), z=0.0)
            det.distance = float(dist)
            det.confidence = float(t['conf'])
            det.source = 'camera_yolo'
            self.pub.publish(det)
            if t['id'] not in self.reported:
                self.reported.add(t['id'])
                self.get_logger().warn('熊を検出: %s 信頼度 %.2f 距離 %.1f m 位置 (%.1f, %.1f)' % (
                    t['id'], t['conf'], dist, t['x'], t['y']))

        if self.img_pub.get_subscription_count() > 0:
            for x0, y0, x1, y1, conf, dist in drawn:
                p0, p1 = (int(x0), int(y0)), (int(x1), int(y1))
                cv2.rectangle(img, p0, p1, (0, 0, 255), max(2, w // 320))
                label = 'bear %.2f  %.1fm' % (conf, dist) if math.isfinite(dist) else \
                    'bear %.2f' % conf
                scale = max(0.6, w / 1000.0)
                cv2.putText(img, label, (p0[0], max(20, p0[1] - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                            scale, (0, 0, 255), max(2, w // 480), cv2.LINE_AA)
            self.img_pub.publish(bgr_to_image_msg(img, msg.header))


def main(args=None):
    rclpy.init(args=args)
    node = BearDetector()
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
