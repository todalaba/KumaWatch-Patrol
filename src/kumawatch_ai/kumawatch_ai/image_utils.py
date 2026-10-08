"""sensor_msgs/Image と OpenCV 画像の変換(cv_bridge に依存しない)."""

import cv2
import numpy as np


def image_to_bgr(msg):
    """sensor_msgs/Image(rgb8 / bgr8 / mono8 / rgba8 / bgra8)を BGR の ndarray に変換する."""
    enc = msg.encoding.lower()
    channels = {'rgb8': 3, 'bgr8': 3, 'mono8': 1, 'rgba8': 4, 'bgra8': 4}.get(enc)
    if channels is None:
        raise ValueError('unsupported encoding: %s' % msg.encoding)
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    img = buf.reshape(msg.height, msg.step)[:, :msg.width * channels]
    img = img.reshape(msg.height, msg.width, channels)
    if enc == 'rgb8':
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if enc == 'rgba8':
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    if enc == 'bgra8':
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    if enc == 'mono8':
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img.copy()


def bgr_to_image_msg(img, header=None):
    """BGR の ndarray を sensor_msgs/Image(bgr8)にする."""
    from sensor_msgs.msg import Image

    msg = Image()
    if header is not None:
        msg.header = header
    msg.height, msg.width = img.shape[:2]
    msg.encoding = 'bgr8'
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(img).tobytes()
    return msg


def matched_image_qos(node, topic, depth=5, mode='auto'):
    """画像トピックを送っている側の QoS に合わせた購読用 QoS を返す(送る側がまだいなければ None).

    Gazebo のカメラは Reliable で送っている。1920×1080 のような大きな画像を Best Effort で
    受けると、DDS の設定(講義環境の CycloneDDS など)によってはほとんど届かないので、
    送る側が Reliable なら Reliable で受ける。送る側が Best Effort なら Best Effort で受ける
    (Reliable で受けると互換性がなく 1 枚も届かない)。
    """
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

    if mode == 'reliable':
        rel = ReliabilityPolicy.RELIABLE
    elif mode == 'best_effort':
        rel = ReliabilityPolicy.BEST_EFFORT
    else:
        pubs = node.get_publishers_info_by_topic(topic)
        if not pubs:
            return None
        rel = (ReliabilityPolicy.BEST_EFFORT
               if any(p.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT for p in pubs)
               else ReliabilityPolicy.RELIABLE)
    return QoSProfile(depth=depth, history=HistoryPolicy.KEEP_LAST, reliability=rel)
