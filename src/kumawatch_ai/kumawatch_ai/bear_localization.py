"""カメラの検出枠から熊の位置を推定し、同じ熊を追跡する(ROS 非依存).

- 方位: 検出枠の中心の横位置とカメラの焦点距離から求める(カメラは前向き・水平)
- 距離: その方位の LiDAR 距離を優先し、取れなければ枠の高さ(熊の体高)から推定する
- 追跡: 地図上の推定位置が近い検出を同じ熊とみなして ID を付ける
- 確定: 誤検出で止まらないよう、一定数のフレームで連続して検出したときだけ「検知」とする
"""

import math

BEAR_HEIGHT_M = 1.09     # Gazebo の熊モデルの体高(world_builder.bear_model と同じ)


def focal_length_px(width_px, hfov_rad):
    return (width_px / 2.0) / math.tan(hfov_rad / 2.0)


def bearing_from_bbox(u_center, width_px, fx, cx=None):
    """検出枠の中心 u から、ロボット正面を 0 とした方位 [rad](左が正)."""
    cx = width_px / 2.0 if cx is None else cx
    return math.atan2(cx - u_center, fx)


def range_from_bbox_height(h_px, fy, object_height_m=BEAR_HEIGHT_M):
    """検出枠の高さから距離 [m] を推定する(ピンホールモデル)."""
    if h_px <= 1:
        return float('inf')
    return fy * object_height_m / h_px


def range_from_scan(ranges, angle_min, angle_increment, bearing, window_rad=math.radians(4.0),
                    range_min=0.1, range_max=10.0):
    """方位 bearing 付近(±window)の LiDAR 距離の最小値. 有効な値がなければ None."""
    if not ranges or angle_increment == 0:
        return None
    n = len(ranges)
    best = None
    steps = int(math.ceil(window_rad / abs(angle_increment)))
    center = (bearing - angle_min) / angle_increment
    for k in range(int(round(center)) - steps, int(round(center)) + steps + 1):
        r = ranges[k % n]
        if r is None or math.isnan(r) or math.isinf(r) or not range_min <= r <= range_max:
            continue
        best = r if best is None else min(best, r)
    return best


def estimate_range(scan_range, bbox_range, max_ratio=1.8):
    """LiDAR の距離と枠からの距離を組み合わせる.

    LiDAR は熊の手前の障害物(木など)を拾うことがあるので、枠からの推定と大きく違う場合は
    枠からの推定を使う。
    """
    if scan_range is None:
        return bbox_range
    if math.isinf(bbox_range):
        return scan_range
    ratio = max(scan_range, bbox_range) / max(1e-6, min(scan_range, bbox_range))
    return scan_range if ratio <= max_ratio else bbox_range


def to_map(robot_x, robot_y, robot_yaw, bearing, distance):
    a = robot_yaw + bearing
    return robot_x + distance * math.cos(a), robot_y + distance * math.sin(a)


class BearTracker:
    """地図上の位置で熊を対応付け、連続して検出されたものだけを確定させる."""

    def __init__(self, match_radius=2.5, min_hits=3, max_miss=5, id_prefix='bear_cam_'):
        self.match_radius = match_radius
        self.min_hits = min_hits
        self.max_miss = max_miss
        self.id_prefix = id_prefix
        self.tracks = []       # dict(id, x, y, hits, miss, confirmed)
        self._next = 1

    def update(self, detections):
        """detections: [(x, y, conf), ...](このフレームの検出). 確定した track のリストを返す."""
        used = set()
        for x, y, conf in sorted(detections, key=lambda d: -d[2]):
            best, best_d = None, self.match_radius
            for t in self.tracks:
                d = math.hypot(t['x'] - x, t['y'] - y)
                if id(t) not in used and d <= best_d:
                    best, best_d = t, d
            if best is None:
                best = {'id': None, 'x': x, 'y': y, 'hits': 0, 'miss': 0, 'confirmed': False}
                self.tracks.append(best)
            else:
                best['x'] = 0.7 * best['x'] + 0.3 * x
                best['y'] = 0.7 * best['y'] + 0.3 * y
            best['hits'] += 1
            best['miss'] = 0
            best['conf'] = conf
            used.add(id(best))
            if not best['confirmed'] and best['hits'] >= self.min_hits:
                best['confirmed'] = True
                best['id'] = '%s%d' % (self.id_prefix, self._next)
                self._next += 1
        for t in self.tracks:
            if id(t) not in used:
                t['miss'] += 1
                if not t['confirmed']:
                    t['hits'] = 0          # 未確定のものは連続して見えなければやり直し
        # 確定済みの熊は ID を覚えておく(同じ熊を何度も「新しい熊」と数えない)
        self.tracks = [t for t in self.tracks if t['confirmed'] or t['miss'] <= self.max_miss]
        return [t for t in self.tracks if t['confirmed'] and id(t) in used]
