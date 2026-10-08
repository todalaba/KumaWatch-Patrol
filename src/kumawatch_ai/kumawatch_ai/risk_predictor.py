"""risk_predictor ノード: 巡回候補地点ごとの熊出没リスクを配信する.

mode:=ttm       (既定) KumaWatch の TTM(IBM Granite TinyTimeMixer)予測を読み込む
                 (data/kumawatch_ttm_scores.csv。scripts/import_kumawatch_ttm.py で作成)
mode:=simulated 模擬データで学習したロジスティック回帰の事前計算スコア(旧レベル1)
mode:=model     ロジスティック回帰(risk_model.json)で地点特徴量からその場で推定する
mode:=csv       scores_file で指定した任意のスコア CSV(point_id, risk 列)

出力:
  /kumawatch/risk_scores   kumawatch_interfaces/RiskScoreArray (transient_local)
  /kumawatch/risk_markers  visualization_msgs/MarkerArray (RViz 用のリスク棒グラフ)
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from kumawatch_interfaces.msg import RiskScore, RiskScoreArray
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from kumawatch_ai.risk_model import (
    load_patrol_points, read_point_features, read_risk_scores, read_score_model_name, RiskModel)


def latched_qos(depth=1):
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def risk_color(risk):
    """低リスク=緑 → 高リスク=赤."""
    r = max(0.0, min(1.0, risk))
    return (min(1.0, 2 * r), min(1.0, 2 * (1 - r)), 0.1)


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class RiskPredictor(Node):

    def __init__(self):
        super().__init__('risk_predictor')
        data_dir = os.path.join(get_package_share_directory('kumawatch_ai'), 'data')
        self.declare_parameter('mode', 'ttm')
        self.declare_parameter('points_file', os.path.join(data_dir, 'patrol_points.yaml'))
        self.declare_parameter('ttm_file', os.path.join(data_dir, 'kumawatch_ttm_scores.csv'))
        self.declare_parameter('simulated_file',
                               os.path.join(data_dir, 'simulated_risk_scores.csv'))
        self.declare_parameter('scores_file', '')
        self.declare_parameter('features_file', os.path.join(data_dir, 'point_features.csv'))
        self.declare_parameter('model_file', os.path.join(data_dir, 'risk_model.json'))
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('republish_period', 2.0)

        self.msg = self._compute()
        self.pub = self.create_publisher(RiskScoreArray, '/kumawatch/risk_scores', latched_qos())
        self.marker_pub = self.create_publisher(MarkerArray, '/kumawatch/risk_markers', latched_qos())
        self._publish()
        period = self.get_parameter('republish_period').value
        if period > 0:
            self.create_timer(period, self._publish)

    def _compute(self):
        mode = self.get_parameter('mode').value
        points = load_patrol_points(self.get_parameter('points_file').value)
        if mode in ('ttm', 'simulated', 'csv'):
            key = {'ttm': 'ttm_file', 'simulated': 'simulated_file', 'csv': 'scores_file'}[mode]
            path = self.get_parameter(key).value
            if not path:
                raise ValueError('mode=csv では scores_file を指定してください')
            risks = read_risk_scores(path)
            model_name = read_score_model_name(path) or ('precomputed:' + os.path.basename(path))
        elif mode == 'model':
            model = RiskModel.load(self.get_parameter('model_file').value)
            feats = read_point_features(self.get_parameter('features_file').value)
            risks = {pid: model.predict_proba(raw) for pid, raw in feats.items()}
            model_name = model.name
        else:
            raise ValueError('mode must be ttm / simulated / model / csv, got %r' % mode)

        msg = RiskScoreArray()
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.model_name = model_name
        self.get_logger().info('リスク推定 (%s):' % model_name)
        for p in points:
            if p['id'] not in risks:
                self.get_logger().warn('地点 %s のスコアがありません。スキップします' % p['id'])
                continue
            s = RiskScore()
            s.point_id = p['id']
            s.name = p['name']
            s.category = p['category']
            s.pose = Pose()
            s.pose.position.x = float(p['x'])
            s.pose.position.y = float(p['y'])
            q = yaw_to_quaternion(float(p['yaw']))
            s.pose.orientation.x, s.pose.orientation.y = q[0], q[1]
            s.pose.orientation.z, s.pose.orientation.w = q[2], q[3]
            s.risk = float(risks[p['id']])
            s.importance_weight = float(p['importance_weight'])
            msg.scores.append(s)
            self.get_logger().info('  %s %-8s risk=%.3f %s' % (
                s.point_id, s.name, s.risk, '#' * int(s.risk * 30)))
        return msg

    def _publish(self):
        now = self.get_clock().now().to_msg()
        self.msg.header.stamp = now
        self.pub.publish(self.msg)
        self.marker_pub.publish(self._markers(now))

    def _markers(self, stamp):
        arr = MarkerArray()
        frame = self.msg.header.frame_id
        for i, s in enumerate(self.msg.scores):
            h = 0.2 + 2.0 * s.risk
            bar = Marker()
            bar.header.frame_id = frame
            bar.header.stamp = stamp
            bar.ns = 'risk_bar'
            bar.id = i
            bar.type = Marker.CYLINDER
            bar.action = Marker.ADD
            bar.pose.position.x = s.pose.position.x
            bar.pose.position.y = s.pose.position.y
            bar.pose.position.z = h / 2.0
            bar.pose.orientation.w = 1.0
            bar.scale.x = bar.scale.y = 0.35
            bar.scale.z = h
            bar.color.r, bar.color.g, bar.color.b = risk_color(s.risk)
            bar.color.a = 0.8
            arr.markers.append(bar)

            label = Marker()
            label.header.frame_id = frame
            label.header.stamp = stamp
            label.ns = 'risk_label'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = s.pose.position.x
            label.pose.position.y = s.pose.position.y
            label.pose.position.z = h + 0.35
            label.pose.orientation.w = 1.0
            label.scale.z = 0.35
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = '%s %s\nrisk %.2f' % (s.point_id, s.category, s.risk)
            arr.markers.append(label)
        return arr


def main(args=None):
    rclpy.init(args=args)
    node = RiskPredictor()
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
