"""image_recorder ノード: ロボットのカメラ映像を MP4 に録画する(動画素材づくり用).

- 画像の時刻(シミュレーション時間)に合わせてフレームを並べるので、PC が遅くて画像が
  まばらにしか届かなくても、動画の長さは撮影していた時間と同じになる(frame_pacer.py)。
- 画像を送る側の QoS に合わせて受信する(送る側が Reliable なら取りこぼさない Reliable で受ける)。
  1920×1080 の大きな画像を Best Effort で受けると、DDS の設定によってはほとんど届かないため。
- 横幅が max_width を超える画像は縮小して保存する(既定 1280 px)。
- 映像の上部にミッションの進み具合(Mission 1/3: P2 Forest edge / AI risk 0.98 / Navigating)を重ねる
  (/kumawatch/mission_status)。熊を検知したときは大きな赤いパネル(BEAR DETECTED / Confidence /
  Location / Action)を重ね、検知の瞬間の静止画(JPEG)も保存する。
- 10 秒ごとに「受信フレーム数・書き込みフレーム数・受信頻度」を表示する。

  ros2 run kumawatch_mission image_recorder --ros-args -p use_sim_time:=true
  ros2 run kumawatch_mission image_recorder --ros-args -p use_sim_time:=true \\
      -p image_topic:=/kumawatch/detection_image -p label:=detection -p fps:=5.0

出力: ~/kumawatch_patrol_output/videos/<日時>_<label>.mp4 と bear_<id>_<日時>.jpg
"""

from datetime import datetime
import json
import os
import signal
import time

import cv2
from kumawatch_ai.image_utils import image_to_bgr, matched_image_qos
from kumawatch_interfaces.msg import BearDetection
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from kumawatch_mission.frame_pacer import FramePacer
from kumawatch_mission.hud import draw_hud


class ImageRecorder(Node):

    def __init__(self):
        super().__init__('image_recorder')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('output_dir',
                               os.path.expanduser('~/kumawatch_patrol_output/videos'))
        self.declare_parameter('overlay', True)
        self.declare_parameter('banner_seconds', 6.0)
        self.declare_parameter('label', 'camera')         # 出力ファイル名の末尾
        self.declare_parameter('max_width', 1280)         # これより大きい画像は縮小(0 で縮小しない)
        self.declare_parameter('qos', 'auto')             # auto / reliable / best_effort

        self.out_dir = self.get_parameter('output_dir').value
        os.makedirs(self.out_dir, exist_ok=True)
        self.stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = os.path.join(self.out_dir, '%s_%s.mp4' % (
            self.stamp, self.get_parameter('label').value))
        self.topic = self.get_parameter('image_topic').value
        self.fps = float(self.get_parameter('fps').value)
        self.pacer = FramePacer(self.fps)
        self.writer = None
        self.size = None
        self.received = 0
        self.frames = 0
        self.first_rx = None
        self.state = '-'
        self.status = None        # /kumawatch/mission_status の JSON
        self.bear = None          # (bear_id, distance, 受信時刻[s])
        self.snapshotted = set()
        self.last_frame = None
        self.image_sub = None

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/kumawatch/mission_state', self._on_state, latched)
        self.create_subscription(String, '/kumawatch/mission_status', self._on_status, latched)
        self.create_subscription(BearDetection, '/kumawatch/bear_detection', self._on_bear, 10)
        self.get_logger().info('録画を待機中: %s → %s' % (self.topic, self.path))
        # 画像を送る側が現れるのを待ってから、その QoS に合わせて購読する
        self.sub_timer = self.create_timer(0.5, self._try_subscribe)
        self.wait_since = time.time()
        self.create_timer(10.0, self._report)

    # ------------------------------------------------------------ 購読の準備
    def _try_subscribe(self):
        qos = matched_image_qos(self, self.topic, depth=5, mode=self.get_parameter('qos').value)
        if qos is None:
            if time.time() - self.wait_since > 10.0:
                self.get_logger().warn(
                    '%s を送っているノードが見つかりません(ros2 topic list で確認してください)'
                    % self.topic, throttle_duration_sec=30.0)
            return
        self.image_sub = self.create_subscription(Image, self.topic, self._on_image, qos)
        self.sub_timer.cancel()
        self.get_logger().info('%s を購読しました(QoS: %s)' % (
            self.topic, 'Reliable' if qos.reliability == ReliabilityPolicy.RELIABLE
            else 'Best Effort'))

    # ------------------------------------------------------------ コールバック
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_state(self, msg):
        self.state = msg.data

    def _on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except ValueError:
            pass

    def _on_bear(self, msg):
        if not msg.detected:
            return
        self.bear = (msg.bear_id, msg.distance, self._now(), msg.confidence)
        if msg.bear_id not in self.snapshotted and self.last_frame is not None:
            self.snapshotted.add(msg.bear_id)
            path = os.path.join(self.out_dir, 'bear_%s_%s.jpg' % (
                msg.bear_id, datetime.now().strftime('%Y%m%d_%H%M%S')))
            cv2.imwrite(path, self._decorate(self.last_frame.copy()))
            self.get_logger().info('熊検知時の画像を保存: %s' % path)

    def _decorate(self, img):
        if not self.get_parameter('overlay').value:
            return img
        bear = None
        if self.bear and self._now() - self.bear[2] < self.get_parameter('banner_seconds').value:
            # 信頼度・距離は届いた検知メッセージの値、地点と行動は mission_planner の状況から
            st = self.status or {}
            last = st.get('last_bear') or {}
            bear = {'bear_id': self.bear[0], 'distance_m': self.bear[1],
                    'confidence': self.bear[3],
                    'point_id': last.get('point_id') or st.get('point_id', ''),
                    'action': last.get('action', 'Stop and report')}
        return draw_hud(img, self.status or {'state': self.state}, bear)

    def _on_image(self, msg):
        self.received += 1
        if self.first_rx is None:
            self.first_rx = time.time()
        try:
            img = image_to_bgr(msg)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return
        max_w = int(self.get_parameter('max_width').value)
        if max_w > 0 and img.shape[1] > max_w:
            h = int(round(img.shape[0] * max_w / img.shape[1]))
            img = cv2.resize(img, (max_w, h - h % 2), interpolation=cv2.INTER_AREA)
        self.last_frame = img

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0:
            stamp = self._now()
        n = self.pacer.frames_for(stamp)
        if n == 0:
            return
        if self.writer is None:
            self.size = (img.shape[1], img.shape[0])
            self.writer = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*'mp4v'),
                                          self.fps, self.size)
            if not self.writer.isOpened():
                self.get_logger().error('VideoWriter を開けませんでした: %s' % self.path)
                self.writer = None
                return
            self.get_logger().info('録画開始 %dx%d @ %.0f fps' % (self.size + (self.fps,)))
            self._write_sidecar(stamp)
        if (img.shape[1], img.shape[0]) != self.size:
            img = cv2.resize(img, self.size)
        frame = self._decorate(img.copy())
        for _ in range(n):
            self.writer.write(frame)
        self.frames += n

    def _write_sidecar(self, first_stamp):
        """動画の 0 秒が何時の画像かを <動画名>.json に書く(動画の自動編集で使う)."""
        info = {'first_stamp_s': first_stamp, 'first_wall': time.time(), 'fps': self.fps,
                'topic': self.topic, 'size': list(self.size), 'max_repeat_s': 2.0}
        try:
            with open(os.path.splitext(self.path)[0] + '.json', 'w', encoding='utf-8') as f:
                json.dump(info, f)
        except OSError as e:
            self.get_logger().warn('時刻ファイルを書けません: %s' % e)

    def _report(self):
        if self.received == 0:
            if self.image_sub is not None:
                self.get_logger().warn('%s の画像がまだ 1 枚も届いていません' % self.topic)
            return
        dt = max(1e-3, time.time() - self.first_rx)
        self.get_logger().info('受信 %d 枚(%.1f 枚/秒)、動画 %d フレーム = %.1f 秒' % (
            self.received, self.received / dt, self.frames, self.frames / self.fps))

    def close(self):
        if self.writer is not None:
            self.writer.release()
            self.writer = None
            print('[image_recorder] 保存しました: %s(受信 %d 枚、%d フレーム = %.1f 秒)' % (
                self.path, self.received, self.frames, self.frames / self.fps), flush=True)
        else:
            print('[image_recorder] 画像を 1 枚も受信しなかったので、動画は作りませんでした(%s)'
                  % self.topic, flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = ImageRecorder()
    # launch の停止時に送られる SIGTERM でも MP4 を閉じられるよう KeyboardInterrupt に変換
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()  # Ctrl+C でも MP4 を正しく閉じる(閉じないと再生できないファイルになる)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
