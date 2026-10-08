"""mission_planner ノード: AIリスクから巡回地点を選び、Nav2 で順に巡回する.

状態遷移:
  RISK_ASSESSMENT → ROUTE_PLANNING → WAIT_NAV2 → PATROL ⇄ OBSERVE → ... → (RETURN) → FINISHED
                                                  PATROL/OBSERVE → EMERGENCY_STOP(熊検知) → REPORT → PATROL
                                                  (on_bear=retreat: EMERGENCY_STOP → RETREAT → PATROL)

入力:
  /kumawatch/risk_scores     kumawatch_interfaces/RiskScoreArray
  /kumawatch/bear_detection  kumawatch_interfaces/BearDetection
  /odom                      nav_msgs/Odometry(移動距離の積算)
  /amcl_pose                 geometry_msgs/PoseWithCovarianceStamped(自己位置推定の準備確認)
  navigate_to_pose           nav2_msgs/action/NavigateToPose(アクションクライアント)
出力:
  /kumawatch/mission_state   std_msgs/String(現在の状態)
  /kumawatch/patrol_summary  std_msgs/String(巡回結果の JSON)
  /kumawatch/mission_status  std_msgs/String(動画の字幕・HUD 用の現在状況 JSON。巡回 i/N・地点・リスク・状態)
  /kumawatch/mission_events  std_msgs/String(出来事 1 件ごとの JSON。mission_console・動画編集で使う)
  <output_dir>/<日時>_<strategy>/mission_events.csv(出来事の時刻表。壁時計とシミュレーション時刻)
  /kumawatch/patrol_markers  visualization_msgs/MarkerArray(巡回順・結果・熊の位置)
  /cmd_vel                   geometry_msgs/Twist(緊急停止時のゼロ速度)
"""

import csv
from datetime import datetime
from functools import partial
import json
import math
import os
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Twist
from kumawatch_interfaces.msg import BearDetection, RiskScoreArray
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from kumawatch_mission.route_planning import format_candidate_table, plan_patrol, retreat_point

RISK_ASSESSMENT = 'RISK_ASSESSMENT'
ROUTE_PLANNING = 'ROUTE_PLANNING'
WAIT_NAV2 = 'WAIT_NAV2'
PATROL = 'PATROL'
OBSERVE = 'OBSERVE'
EMERGENCY_STOP = 'EMERGENCY_STOP'
RETREAT = 'RETREAT'
RETURN = 'RETURN_TO_BASE'
FINISHED = 'FINISHED'

RESULT_FIELDS = ['order', 'point_id', 'name', 'category', 'x', 'y', 'yaw', 'risk', 'utility',
                 'status', 'arrived', 'arrival_ros_time_s', 'bear_id']
EVENT_FIELDS = ['time', 'ros_time_s', 'point_id', 'bear_id', 'robot_x', 'robot_y',
                'bear_x', 'bear_y', 'distance_m', 'confidence', 'source']
TIMELINE_FIELDS = ['wall_time', 'ros_time_s', 'event', 'index', 'point_id', 'detail']

STATUS_COLORS = {
    'pending': (0.7, 0.7, 0.7),
    'clear': (0.1, 0.8, 0.2),
    'bear_found': (1.0, 0.1, 0.1),
    'unreachable': (0.4, 0.4, 0.4),
    'timeout': (0.4, 0.4, 0.4),
}


def latched_qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class MissionPlanner(Node):

    def __init__(self):
        super().__init__('mission_planner')
        points_file = os.path.join(get_package_share_directory('kumawatch_ai'), 'data',
                                   'patrol_points.yaml')
        self.declare_parameter('points_file', points_file)
        self.declare_parameter('strategy', 'utility')          # utility / topk / fixed
        self.declare_parameter('budget_k', 3)                  # 巡回可能な地点数 K
        self.declare_parameter('lambda_distance', 0.02)        # 移動コストの重み λ [1/m]
        self.declare_parameter('fixed_route', ['P1', 'P2', 'P3'])
        self.declare_parameter('observe_duration', 3.0)        # 到着後の確認時間 [s]
        self.declare_parameter('bear_stop_duration', 5.0)      # 熊検知時の停止時間 [s]
        self.declare_parameter('on_bear', 'continue')          # continue / retreat / finish
        self.declare_parameter('retreat_distance', 3.0)        # 退避で熊から下がる距離 [m]
        self.declare_parameter('goal_timeout', 240.0)          # 1地点あたりの制限時間 [s]
        self.declare_parameter('return_to_base', True)
        self.declare_parameter('output_dir', os.path.expanduser('~/kumawatch_patrol_output'))
        self.declare_parameter('initial_pose_timeout', 10.0)

        with open(self.get_parameter('points_file').value, encoding='utf-8') as f:
            points_cfg = yaml.safe_load(f)
        self.base = points_cfg['base']
        self.name_en = {p['id']: p.get('name_en', p['id']) for p in points_cfg.get('points', [])}
        self.strategy = self.get_parameter('strategy').value
        self.k = int(self.get_parameter('budget_k').value)
        self.lam = float(self.get_parameter('lambda_distance').value)

        self.state = None
        self.state_since = self.get_clock().now()
        self.scores_msg = None
        self.candidates = []
        self.plan = None
        self.results = []          # 巡回予定地点ごとの結果 dict
        self.target_idx = -1
        self.events = []           # 熊検知イベント
        self.handled_bears = set()
        self.goal_seq = 0
        self.goal_handle = None
        self.goal_sent_at = None
        self.distance_m = 0.0
        self.last_odom_xy = None
        self.trajectory = []
        self.trajectory_t = []     # 軌跡の各点の ROS 時刻(動画の再生・編集用)
        self.amcl_ready = False
        self.initial_pose_sent = False
        self.nav2_active = False
        self.lifecycle_future = None
        self.started_at = None
        self.finished = False
        self.last_wait_log = None
        self.last_bear = None      # HUD 用の直近の熊検知
        self.retreat_goal = None
        self.run_dir = os.path.join(self.get_parameter('output_dir').value, '%s_%s' % (
            datetime.now().strftime('%Y%m%d_%H%M%S'), self.strategy))
        self.timeline_path = None

        self.state_pub = self.create_publisher(String, '/kumawatch/mission_state', latched_qos())
        self.summary_pub = self.create_publisher(String, '/kumawatch/patrol_summary', latched_qos())
        self.marker_pub = self.create_publisher(MarkerArray, '/kumawatch/patrol_markers',
                                                latched_qos())
        self.status_pub = self.create_publisher(String, '/kumawatch/mission_status', latched_qos())
        self.event_pub = self.create_publisher(String, '/kumawatch/mission_events',
                                               QoSProfile(depth=50))
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.initpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.create_subscription(RiskScoreArray, '/kumawatch/risk_scores', self._on_scores,
                                 latched_qos())
        self.create_subscription(BearDetection, '/kumawatch/bear_detection', self._on_bear, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl,
                                 latched_qos())

        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.lifecycle_client = self.create_client(GetState, '/bt_navigator/get_state')

        self._set_state(RISK_ASSESSMENT)
        self.create_timer(0.1, self._tick)
        self.create_timer(1.0, self._publish_markers)  # 走行軌跡の更新
        self.create_timer(0.5, self._publish_status)

    # ------------------------------------------------------------------ utils
    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _elapsed(self):
        return (self.get_clock().now() - self.state_since).nanoseconds * 1e-9

    def _set_state(self, state):
        if state != self.state:
            self.get_logger().info('[STATE] %s -> %s' % (self.state, state))
        self.state = state
        self.state_since = self.get_clock().now()
        self.state_pub.publish(String(data=state))
        self._publish_status()

    # ------------------------------------------------- 動画用: 状況と出来事の記録
    def _event(self, event, point_id='', detail=None):
        """出来事を 1 件記録する(トピック配信 + mission_events.csv に追記)."""
        row = {
            'wall_time': round(time.time(), 3),
            'ros_time_s': round(self._now_s(), 2),
            'event': event,
            'index': self.target_idx + 1 if 0 <= self.target_idx < len(self.results) else '',
            'point_id': point_id,
            'detail': json.dumps(detail or {}, ensure_ascii=False),
        }
        msg = dict(row)
        msg['detail'] = detail or {}
        msg['total'] = len(self.results)
        self.event_pub.publish(String(data=json.dumps(msg, ensure_ascii=False)))
        try:
            if self.timeline_path is None:
                os.makedirs(self.run_dir, exist_ok=True)
                self.timeline_path = os.path.join(self.run_dir, 'mission_events.csv')
                with open(self.timeline_path, 'w', newline='', encoding='utf-8') as f:
                    csv.DictWriter(f, fieldnames=TIMELINE_FIELDS).writeheader()
            with open(self.timeline_path, 'a', newline='', encoding='utf-8') as f:
                csv.DictWriter(f, fieldnames=TIMELINE_FIELDS).writerow(row)
        except OSError as e:
            self.get_logger().error('mission_events.csv に書けません: %s' % e,
                                    throttle_duration_sec=30.0)
        self._publish_status()

    def _publish_status(self):
        cur = self._current()
        status = {
            'state': self.state,
            'strategy': self.strategy,
            'model': self.scores_msg.model_name if self.scores_msg else '',
            'route': [r['point_id'] for r in self.results],
            'index': self.target_idx + 1 if cur else 0,
            'total': len(self.results),
            'point_id': cur['point_id'] if cur else '',
            'name': cur['name'] if cur else '',
            'name_en': self.name_en.get(cur['point_id'], '') if cur else '',
            'risk': cur['risk'] if cur else None,
            'point_status': cur['status'] if cur else '',
            'bears_found': len(self.events),
            'arrived': sum(1 for r in self.results if r['arrived']),
            'distance_m': round(self.distance_m, 1),
            'elapsed_s': round(self._now_s() - self.started_at, 1) if self.started_at else 0.0,
            'last_bear': self.last_bear,
        }
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))

    def _current(self):
        if 0 <= self.target_idx < len(self.results):
            return self.results[self.target_idx]
        return None

    # -------------------------------------------------------------- callbacks
    def _on_scores(self, msg):
        if self.scores_msg is None:
            self.get_logger().info('リスクスコアを受信 (%d 地点, model=%s)' % (
                len(msg.scores), msg.model_name))
        self.scores_msg = msg

    def _on_amcl(self, _msg):
        if not self.amcl_ready:
            self.get_logger().info('自己位置推定(AMCL)の準備完了')
        self.amcl_ready = True

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        xy = (p.x, p.y)
        if self.started_at is not None and not self.finished and self.last_odom_xy is not None:
            step = math.hypot(xy[0] - self.last_odom_xy[0], xy[1] - self.last_odom_xy[1])
            if step < 1.0:  # シミュレーションのリセット等による跳びは無視
                self.distance_m += step
            if not self.trajectory or math.hypot(
                    xy[0] - self.trajectory[-1][0], xy[1] - self.trajectory[-1][1]) > 0.1:
                self.trajectory.append(xy)
                self.trajectory_t.append(self._now_s())
        self.last_odom_xy = xy

    def _on_bear(self, msg):
        if not msg.detected or msg.bear_id in self.handled_bears:
            return
        if self.state not in (PATROL, OBSERVE, RETURN, RETREAT):
            return
        self.handled_bears.add(msg.bear_id)
        cur = self._current() if self.state != RETURN else None
        # EMERGENCY_STOP: ナビゲーションを中止してその場で停止
        self._cancel_goal()
        self._publish_stop()
        self._set_state(EMERGENCY_STOP)
        # REPORT: 位置・時刻・信頼度を記録
        event = {
            'time': datetime.now().isoformat(timespec='seconds'),
            'ros_time_s': round(self._now_s(), 2),
            'point_id': cur['point_id'] if cur else '',
            'bear_id': msg.bear_id,
            'robot_x': round(msg.robot_position.x, 2),
            'robot_y': round(msg.robot_position.y, 2),
            'bear_x': round(msg.bear_position.x, 2),
            'bear_y': round(msg.bear_position.y, 2),
            'distance_m': round(msg.distance, 2),
            'confidence': round(msg.confidence, 3),
            'source': msg.source,
        }
        self.events.append(event)
        if cur is not None:
            cur['status'] = 'bear_found'
            cur['bear_id'] = msg.bear_id
        self.last_bear = {
            'bear_id': msg.bear_id, 'point_id': event['point_id'],
            'confidence': event['confidence'], 'distance_m': event['distance_m'],
            'bear_x': event['bear_x'], 'bear_y': event['bear_y'],
            'robot_x': event['robot_x'], 'robot_y': event['robot_y'],
            'source': msg.source, 'action': self._bear_action(),
        }
        self._event('bear_detected', event['point_id'], self.last_bear)
        self.get_logger().warn(
            '!!! 熊を検知: %s (地点 %s) 距離 %.2f m, 信頼度 %.2f → 停止します' % (
                msg.bear_id, event['point_id'] or '-', msg.distance, msg.confidence))
        self._publish_markers()

    def _bear_action(self):
        return {'retreat': 'Stop, report and retreat', 'finish': 'Stop, report and return',
                }.get(self.get_parameter('on_bear').value, 'Stop and report')

    # ------------------------------------------------------------ navigation
    def _send_goal(self, x, y, yaw):
        self.goal_seq += 1
        seq = self.goal_seq
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        q = yaw_to_quaternion(float(yaw))
        goal.pose.pose.orientation.z = q[2]
        goal.pose.pose.orientation.w = q[3]
        self.goal_sent_at = self._now_s()
        self.nav.send_goal_async(goal).add_done_callback(partial(self._on_goal_response, seq))

    def _on_goal_response(self, seq, future):
        handle = future.result()
        if seq != self.goal_seq:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.get_logger().error('Nav2 がゴールを拒否しました')
            self._on_nav_finished(seq, GoalStatus.STATUS_ABORTED)
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self._on_nav_finished(seq, f.result().status))

    def _on_nav_finished(self, seq, status):
        if seq != self.goal_seq:
            return  # 取り消し済みの古いゴール
        self.goal_handle = None
        if self.state == PATROL:
            cur = self._current()
            if status == GoalStatus.STATUS_SUCCEEDED:
                cur['arrived'] = True
                cur['arrival_ros_time_s'] = round(self._now_s(), 2)
                self.get_logger().info('地点 %s に到着。周囲を確認します (OBSERVE)' % cur['point_id'])
                self._set_state(OBSERVE)
                self._event('arrived', cur['point_id'])
            else:
                cur['status'] = 'unreachable'
                self.get_logger().error('地点 %s に到達できませんでした (status=%d)' % (
                    cur['point_id'], status))
                self._event('unreachable', cur['point_id'])
                self._next_target()
        elif self.state == RETREAT:
            ok = status == GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info('安全地点への退避を完了しました' if ok
                                   else '退避地点に到達できませんでした (status=%d)。巡回を続けます' % status)
            self._event('retreat_done' if ok else 'retreat_failed',
                        self.last_bear['point_id'] if self.last_bear else '')
            self._after_bear()
        elif self.state == RETURN:
            self.get_logger().info('詰所に帰還しました' if status == GoalStatus.STATUS_SUCCEEDED
                                   else '詰所への帰還に失敗しました (status=%d)' % status)
            self._event('returned' if status == GoalStatus.STATUS_SUCCEEDED else 'return_failed')
            self._finish()

    def _cancel_goal(self):
        self.goal_seq += 1  # 以降に届く古いゴールの結果を無視する
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    def _publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(self.base['x'])
        msg.pose.pose.position.y = float(self.base['y'])
        q = yaw_to_quaternion(float(self.base.get('yaw', 0.0)))
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        self.initpose_pub.publish(msg)

    def _check_nav2_active(self):
        if self.nav2_active:
            return True
        if self.lifecycle_future is not None:
            if self.lifecycle_future.done():
                res = self.lifecycle_future.result()
                self.lifecycle_future = None
                if res is not None and res.current_state.label == 'active':
                    self.nav2_active = True
                    self.get_logger().info('Nav2 (bt_navigator) が active になりました')
                    return True
            return False
        if self.lifecycle_client.service_is_ready():
            self.lifecycle_future = self.lifecycle_client.call_async(GetState.Request())
        return False

    # ------------------------------------------------------------------ tick
    def _tick(self):
        if self.state == RISK_ASSESSMENT:
            if self.scores_msg is not None:
                self._set_state(ROUTE_PLANNING)
        elif self.state == ROUTE_PLANNING:
            self._plan_route()
            self._set_state(WAIT_NAV2)
        elif self.state == WAIT_NAV2:
            if (not self.amcl_ready and not self.initial_pose_sent
                    and self._elapsed() > self.get_parameter('initial_pose_timeout').value):
                self.get_logger().warn('AMCL の初期位置が未設定のため /initialpose を送信します')
                self._publish_initial_pose()
                self.initial_pose_sent = True
            ready = self._check_nav2_active() and self.nav.server_is_ready() and self.amcl_ready
            if ready:
                self.started_at = self._now_s()
                self._event('mission_started', detail={
                    'route': [r['point_id'] for r in self.results], 'strategy': self.strategy})
                self._next_target()
            elif self.last_wait_log is None or self._now_s() - self.last_wait_log > 5.0:
                self.last_wait_log = self._now_s()
                self.get_logger().info(
                    'Nav2 と自己位置推定の起動を待っています... (bt_navigator=%s, action=%s, amcl=%s)'
                    % (self.nav2_active, self.nav.server_is_ready(), self.amcl_ready))
        elif self.state == PATROL:
            timeout = self.get_parameter('goal_timeout').value
            if self.goal_sent_at is not None and self._now_s() - self.goal_sent_at > timeout:
                cur = self._current()
                cur['status'] = 'timeout'
                self.get_logger().error('地点 %s への移動がタイムアウトしました' % cur['point_id'])
                self._event('timeout', cur['point_id'])
                self._cancel_goal()
                self._next_target()
        elif self.state == OBSERVE:
            if self._elapsed() >= self.get_parameter('observe_duration').value:
                cur = self._current()
                if cur['status'] == 'pending':
                    cur['status'] = 'clear'
                    self.get_logger().info('地点 %s: 熊は見つかりませんでした' % cur['point_id'])
                    self._event('clear', cur['point_id'])
                self._next_target()
        elif self.state == EMERGENCY_STOP:
            self._publish_stop()
            if self._elapsed() >= self.get_parameter('bear_stop_duration').value:
                if self.get_parameter('on_bear').value == 'retreat' and self.last_bear:
                    self._start_retreat()
                else:
                    self._after_bear()
        elif self.state == RETREAT:
            timeout = self.get_parameter('goal_timeout').value
            if self.goal_sent_at is not None and self._now_s() - self.goal_sent_at > timeout:
                self.get_logger().error('退避がタイムアウトしました。巡回を続けます')
                self._cancel_goal()
                self._after_bear()

    def _start_retreat(self):
        b = self.last_bear
        x, y, yaw = retreat_point((b['robot_x'], b['robot_y']), (b['bear_x'], b['bear_y']),
                                  float(self.get_parameter('retreat_distance').value))
        self.retreat_goal = (x, y)
        self.get_logger().warn('熊から離れて安全地点 (%.1f, %.1f) へ退避します' % (x, y))
        self._set_state(RETREAT)
        self._event('retreat', b['point_id'], {'x': round(x, 2), 'y': round(y, 2)})
        self._send_goal(x, y, yaw)
        self._publish_markers()

    def _after_bear(self):
        """熊への対応(停止・記録・退避)を終えて、巡回を続けるか打ち切る."""
        self.retreat_goal = None
        if self.get_parameter('on_bear').value == 'finish':
            self.target_idx = len(self.results)  # 残りの地点は巡回しない
        self._next_target()

    # -------------------------------------------------------------- planning
    def _plan_route(self):
        self.candidates = []
        for s in self.scores_msg.scores:
            q = s.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.candidates.append({
                'id': s.point_id, 'name': s.name, 'category': s.category,
                'x': s.pose.position.x, 'y': s.pose.position.y, 'yaw': yaw,
                'risk': s.risk, 'importance_weight': s.importance_weight,
            })
        start = (float(self.base['x']), float(self.base['y']))
        self.plan = plan_patrol(
            self.candidates, self.k, self.strategy, start, self.lam,
            fixed_ids=list(self.get_parameter('fixed_route').value),
            return_to_start=bool(self.get_parameter('return_to_base').value))
        route = self.plan['route']
        self.results = [{
            'order': i + 1, 'point_id': c['id'], 'name': c['name'], 'category': c['category'],
            'x': c['x'], 'y': c['y'], 'yaw': c['yaw'], 'risk': c['risk'],
            'utility': round(c['utility'], 4), 'status': 'pending', 'arrived': False,
            'arrival_ros_time_s': None, 'bear_id': '',
        } for i, c in enumerate(route)]
        self.get_logger().info(
            '\n=== 巡回計画 (strategy=%s, K=%d, λ=%.3f) ===\n%s\n巡回順: 詰所 → %s%s\n'
            '計画距離(直線): %.1f m' % (
                self.strategy, self.k, self.lam,
                format_candidate_table(self.candidates, start, self.lam,
                                       {c['id'] for c in route}),
                ' → '.join(c['id'] for c in route),
                ' → 詰所' if self.get_parameter('return_to_base').value else '',
                self.plan['planned_length_m']))
        self._publish_markers()

    def _next_target(self):
        self.target_idx += 1
        if self.target_idx < len(self.results):
            cur = self._current()
            self.get_logger().info('巡回 %d/%d: 地点 %s (%s, risk=%.2f) へ移動します' % (
                cur['order'], len(self.results), cur['point_id'], cur['name'], cur['risk']))
            self._set_state(PATROL)
            self._event('navigate', cur['point_id'], {
                'name_en': self.name_en.get(cur['point_id'], ''), 'risk': cur['risk']})
            self._send_goal(cur['x'], cur['y'], cur['yaw'])
            self._publish_markers()
        elif self.get_parameter('return_to_base').value and self.state != RETURN:
            self.get_logger().info('全地点の巡回を終えました。詰所へ戻ります')
            self._set_state(RETURN)
            self._event('return_to_base')
            self._send_goal(self.base['x'], self.base['y'], self.base.get('yaw', 0.0))
        else:
            self._finish()

    # ---------------------------------------------------------------- finish
    def _finish(self):
        if self.finished:
            return
        self.finished = True
        self._set_state(FINISHED)
        summary = self._summary()
        self.summary_pub.publish(String(data=json.dumps(summary, ensure_ascii=False)))
        self._publish_markers()
        out = self._save(summary)
        self._event('mission_completed', detail={
            k: summary[k] for k in ('num_points', 'num_arrived', 'num_checked', 'bears_found',
                                    'distance_m',
                                    'duration_s', 'route', 'strategy')})
        lines = ['', '=' * 64, ' KumaWatch Patrol 巡回結果',
                 ' strategy=%s  K=%d  model=%s' % (self.strategy, self.k, summary['model']),
                 '-' * 64,
                 '  #  地点  %-13s risk   結果' % 'category']
        for r in self.results:
            lines.append('  %d  %-4s  %-13s %.2f   %s%s' % (
                r['order'], r['point_id'], r['category'], r['risk'], r['status'],
                ' (%s)' % r['bear_id'] if r['bear_id'] else ''))
        lines += ['-' * 64,
                  '  巡回地点数 : %d' % summary['num_points'],
                  '  到着地点数 : %d' % summary['num_arrived'],
                  '  熊発見数   : %d' % summary['bears_found'],
                  '  移動距離   : %.1f m' % summary['distance_m'],
                  '  所要時間   : %.1f s' % summary['duration_s'],
                  '  保存先     : %s' % out,
                  '=' * 64]
        self.get_logger().info('\n'.join(lines))

    def _summary(self):
        return {
            'strategy': self.strategy,
            'budget_k': self.k,
            'lambda_distance': self.lam,
            'model': self.scores_msg.model_name if self.scores_msg else '',
            'route': [r['point_id'] for r in self.results],
            'num_points': len(self.results),
            'num_arrived': sum(1 for r in self.results if r['arrived']),
            # 確認できた地点(到着して熊なし、または手前で熊を見つけた地点)
            'num_checked': sum(1 for r in self.results if r['status'] in ('clear', 'bear_found')),
            'bears_found': len(self.events),
            'distance_m': round(self.distance_m, 2),
            'planned_length_m': round(self.plan['planned_length_m'], 2) if self.plan else 0.0,
            'duration_s': round(self._now_s() - self.started_at, 1) if self.started_at else 0.0,
            'results': self.results,
            'events': self.events,
        }

    def _save(self, summary):
        out = self.run_dir
        try:
            os.makedirs(out, exist_ok=True)
            with open(os.path.join(out, 'patrol_summary.json'), 'w', encoding='utf-8') as f:
                json.dump(summary, f, indent=2, ensure_ascii=False)
            with open(os.path.join(out, 'patrol_results.csv'), 'w', newline='',
                      encoding='utf-8') as f:
                w = csv.DictWriter(f, fieldnames=RESULT_FIELDS, extrasaction='ignore')
                w.writeheader()
                w.writerows(self.results)
            with open(os.path.join(out, 'bear_events.csv'), 'w', newline='',
                      encoding='utf-8') as f:
                w = csv.DictWriter(f, fieldnames=EVENT_FIELDS, extrasaction='ignore')
                w.writeheader()
                w.writerows(self.events)
            with open(os.path.join(out, 'trajectory.csv'), 'w', newline='',
                      encoding='utf-8') as f:   # 走行軌跡(odom 座標、0.1 m ごと、時刻付き)
                w = csv.writer(f)
                w.writerow(['x', 'y', 'ros_time_s'])
                w.writerows((round(x, 3), round(y, 3), round(t, 2))
                            for (x, y), t in zip(self.trajectory, self.trajectory_t))
        except (OSError, ValueError) as e:
            self.get_logger().error('結果の保存に失敗しました: %s' % e)
        return out

    # --------------------------------------------------------------- markers
    def _publish_markers(self):
        if self.plan is None:
            return
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        def marker(ns, mid, mtype, frame='map'):
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = stamp
            m.ns = ns
            m.id = mid
            m.type = mtype
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        # 計画ルート(詰所 → 地点 ...)
        route = marker('planned_route', 0, Marker.LINE_STRIP)
        route.scale.x = 0.06
        route.color.r, route.color.g, route.color.b, route.color.a = 0.2, 0.6, 1.0, 0.9
        pts = [(self.base['x'], self.base['y'])] + [(r['x'], r['y']) for r in self.results]
        for x, y in pts:
            route.points.append(Point(x=float(x), y=float(y), z=0.05))
        arr.markers.append(route)

        for i, r in enumerate(self.results):
            is_cur = (i == self.target_idx and self.state in (PATROL, OBSERVE))
            s = marker('patrol_status', i, Marker.SPHERE)
            s.pose.position.x, s.pose.position.y, s.pose.position.z = r['x'], r['y'], 0.15
            s.scale.x = s.scale.y = s.scale.z = 0.55 if is_cur else 0.4
            c = STATUS_COLORS.get(r['status'], (0.7, 0.7, 0.7))
            if is_cur:
                c = (1.0, 0.8, 0.0)
            s.color.r, s.color.g, s.color.b = c
            s.color.a = 0.9
            arr.markers.append(s)

            t = marker('patrol_order', i, Marker.TEXT_VIEW_FACING)
            t.pose.position.x, t.pose.position.y, t.pose.position.z = r['x'] + 0.6, r['y'], 0.6
            t.scale.z = 0.4
            t.color.r = t.color.g = t.color.a = 1.0
            t.color.b = 0.3
            t.text = '#%d %s %s' % (r['order'], r['point_id'],
                                     '' if r['status'] == 'pending' else r['status'])
            arr.markers.append(t)

        for i, e in enumerate(self.events):
            b = marker('bear_found', i, Marker.CUBE)
            b.pose.position.x, b.pose.position.y, b.pose.position.z = e['bear_x'], e['bear_y'], 0.5
            b.scale.x, b.scale.y, b.scale.z = 1.2, 0.6, 1.0
            b.color.r, b.color.a = 0.9, 0.8
            arr.markers.append(b)
            t = marker('bear_label', i, Marker.TEXT_VIEW_FACING)
            t.pose.position.x, t.pose.position.y, t.pose.position.z = e['bear_x'], e['bear_y'], 1.6
            t.scale.z = 0.45
            t.color.r, t.color.a = 1.0, 1.0
            t.text = 'BEAR %s (%.1fm)' % (e['bear_id'], e['distance_m'])
            arr.markers.append(t)

        if self.retreat_goal is not None:
            s = marker('retreat', 0, Marker.CYLINDER)
            s.pose.position.x, s.pose.position.y, s.pose.position.z = (
                self.retreat_goal[0], self.retreat_goal[1], 0.05)
            s.scale.x, s.scale.y, s.scale.z = 0.8, 0.8, 0.1
            s.color.g, s.color.b, s.color.a = 0.8, 1.0, 0.8
            arr.markers.append(s)
            t = marker('retreat_label', 0, Marker.TEXT_VIEW_FACING)
            t.pose.position.x, t.pose.position.y, t.pose.position.z = (
                self.retreat_goal[0], self.retreat_goal[1], 0.9)
            t.scale.z = 0.4
            t.color.g, t.color.b, t.color.a = 1.0, 1.0, 1.0
            t.text = 'SAFE POINT'
            arr.markers.append(t)
        else:
            d = marker('retreat', 0, Marker.CYLINDER)
            d.action = Marker.DELETE
            arr.markers.append(d)
            d2 = marker('retreat_label', 0, Marker.TEXT_VIEW_FACING)
            d2.action = Marker.DELETE
            arr.markers.append(d2)

        if len(self.trajectory) >= 2:
            tr = marker('trajectory', 0, Marker.LINE_STRIP, frame='odom')
            tr.scale.x = 0.04
            tr.color.r, tr.color.g, tr.color.b, tr.color.a = 1.0, 0.5, 0.0, 0.9
            for x, y in self.trajectory:
                tr.points.append(Point(x=float(x), y=float(y), z=0.03))
            arr.markers.append(tr)

        if self.state == FINISHED:
            summary = marker('summary', 0, Marker.TEXT_VIEW_FACING)
            summary.pose.position.x, summary.pose.position.y = 0.0, -9.5
            summary.pose.position.z = 2.0
            summary.scale.z = 0.6
            summary.color.r = summary.color.g = summary.color.b = summary.color.a = 1.0
            summary.text = 'PATROL DONE [%s]  points=%d  bears=%d  dist=%.1fm' % (
                self.strategy, len(self.results), len(self.events), self.distance_m)
            arr.markers.append(summary)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlanner()
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
