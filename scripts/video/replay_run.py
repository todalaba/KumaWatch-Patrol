#!/usr/bin/env python3
"""記録した巡回を Gazebo で再生する(カメラの角度を変えて撮り直す・下見用).

mission_planner の結果(trajectory.csv と mission_events.csv)を読み、Gazebo に「見た目だけの
ロボット」(センサーなし)を出して、走った軌跡どおりに動かす。同時に次を配信するので、
本番と同じ撮影用ノードがそのまま使える。
  - ロボットの前向きカメラの映像   /replay_cam/image_raw(bear_detector に渡すと YOLO で判定)
  - ロボットの位置               TF map → base_footprint
  - ミッションの状況と出来事      /kumawatch/mission_status, /kumawatch/mission_events

「Nav2 がこの場で走っている」わけではない(記録の再生)。動画に使うときはそう明記すること。
Gazebo(kumawatch_patrol.world)を起動した状態で:

  python3 scripts/video/replay_run.py --run ~/kumawatch_patrol_output/<日時>_utility
  ros2 run kumawatch_mission chase_camera --ros-args -p robot_name:=replay_robot
  ros2 run kumawatch_ai bear_detector --ros-args -p image_topic:=/replay_cam/image_raw \\
      -p camera_info_topic:=/replay_cam/camera_info
"""

import argparse
import bisect
import csv
import json
import math
import os
import re
import sys
import time

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import GetEntityState, SetEntityState, SpawnEntity
from geometry_msgs.msg import TransformStamped
import rclpy
import rclpy.parameter
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, 'src', 'kumawatch_mission'))
from kumawatch_mission.chase_math import camera_sdf, quaternion  # noqa: E402

STATE_OF = {'mission_started': 'PATROL', 'navigate': 'PATROL', 'arrived': 'OBSERVE',
            'clear': 'OBSERVE', 'bear_detected': 'EMERGENCY_STOP', 'retreat': 'RETREAT',
            'retreat_done': 'PATROL', 'retreat_failed': 'PATROL', 'return_to_base': 'RETURN_TO_BASE',
            'returned': 'RETURN_TO_BASE', 'mission_completed': 'FINISHED',
            'safe_hold': 'SAFE_HOLD'}


def load(run):
    with open(os.path.join(run, 'trajectory.csv'), encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    with open(os.path.join(run, 'mission_events.csv'), encoding='utf-8') as f:
        events = list(csv.DictReader(f))
    for e in events:
        e['ros_time_s'] = float(e['ros_time_s'])
        e['detail'] = json.loads(e['detail'] or '{}')
    traj = [(float(r['x']), float(r['y'])) for r in rows]
    if rows and rows[0].get('ros_time_s'):
        times = [float(r['ros_time_s']) for r in rows]
    else:
        times = times_from_events(traj, events)
    return traj, times, events


def times_from_events(traj, events):
    """古い記録(時刻なし)用: 出来事の時刻と位置から、軌跡の各点の時刻を距離で割り振る."""
    cum = [0.0]
    for a, b in zip(traj, traj[1:]):
        cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    import yaml
    with open(os.path.join(ROOT, 'src', 'kumawatch_ai', 'data', 'patrol_points.yaml'),
              encoding='utf-8') as f:
        pts = {p['id']: (p['x'], p['y']) for p in yaml.safe_load(f)['points']}
    anchors = [(0, events[0]['ros_time_s'])]
    last, retreat_to = 0, None

    def nearest(x, y):
        return min(range(last, len(traj)), key=lambda i: math.hypot(traj[i][0] - x, traj[i][1] - y))
    for e in events[1:]:
        d, kind = e['detail'], e['event']
        if kind == 'bear_detected':
            last = nearest(d['robot_x'], d['robot_y'])
        elif kind == 'retreat':
            retreat_to = (d['x'], d['y'])
        elif kind == 'retreat_done' and retreat_to:
            last = nearest(*retreat_to)
        elif kind == 'arrived' and e['point_id'] in pts:
            last = nearest(*pts[e['point_id']])
        elif kind == 'returned':
            last = len(traj) - 1
        elif kind not in ('clear', 'return_to_base', 'mission_completed', 'navigate'):
            continue      # 位置の分からない出来事は目印にしない
        anchors.append((last, e['ros_time_s']))
    anchors.append((len(traj) - 1, events[-1]['ros_time_s']))
    anchors.sort()
    times = []
    for i in range(len(traj)):
        k = max(j for j in range(len(anchors)) if anchors[j][0] <= i)
        i0, t0 = anchors[k]
        i1, t1 = anchors[min(k + 1, len(anchors) - 1)]
        f = 0.0 if cum[i1] == cum[i0] else (cum[i] - cum[i0]) / (cum[i1] - cum[i0])
        times.append(t0 + f * (t1 - t0))
    return times


def visual_only_robot(model):
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models',
                        'turtlebot3_' + model, 'model.sdf')
    with open(path, encoding='utf-8') as f:
        sdf = f.read()
    sdf = re.sub(r'<sensor\b.*?</sensor>', '', sdf, flags=re.S)
    sdf = re.sub(r'<plugin\b.*?</plugin>', '', sdf, flags=re.S)
    sdf = re.sub(r'<static>\s*\w+\s*</static>', '', sdf)
    return re.sub(r'(<model name="[^"]*">)', r'\1<static>true</static>', sdf, count=1)


class Replay(Node):

    def __init__(self, args):
        super().__init__('replay_run')
        self.a = args
        self.traj, self.times, self.events = load(args.run)
        self.t0 = self.events[0]['ros_time_s']
        self.t_end = self.events[-1]['ros_time_s'] + 3.0
        self.start_wall = None
        self.next_ev = 0
        self.status = {'state': 'PATROL', 'strategy': '', 'index': 0, 'total': 0,
                       'bears_found': 0, 'distance_m': 0.0}
        self.busy = 0
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/kumawatch/mission_status', latched)
        self.event_pub = self.create_publisher(String, '/kumawatch/mission_events', 50)
        self.tf = TransformBroadcaster(self)
        self.spawn = self.create_client(SpawnEntity, '/spawn_entity')
        self.set = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.get = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        with open(os.path.join(ROOT, 'src', 'kumawatch_ai', 'data', 'patrol_points.yaml'),
                  encoding='utf-8') as f:
            import yaml
            self.names = {p['id']: p.get('name_en', '') for p in yaml.safe_load(f)['points']}
        # タイマーは壁時計で回す(sim 時刻のタイマーは /clock が来るまで動かないため)
        self.create_timer(1.0 / args.rate, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def prepare(self):
        """Gazebo のサービスを待ってロボットとカメラを出す(spin する前に呼ぶ)."""
        for cli in (self.spawn, self.set):
            while not cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().info('Gazebo のサービス %s を待っています...' % cli.srv_name)
        self._spawn_all()

    def _spawn_all(self):
        for name, xml, z in (('replay_robot', visual_only_robot(self.a.model), 0.0),
                             ('replay_cam', camera_sdf('replay_cam', 640, 480, 10, 1.06), 0.25)):
            req = SpawnEntity.Request()
            req.name, req.xml = name, xml
            req.initial_pose.position.x, req.initial_pose.position.y = self.traj[0]
            req.initial_pose.position.z = z
            fut = self.spawn.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
            res = fut.result()
            if res is None or (not res.success and 'exist' not in res.status_message):
                raise SystemExit('%s を出せませんでした: %s' % (
                    name, res.status_message if res else 'timeout'))
        self.get_logger().info('再生を始めます(%.0f 秒、%.1f 倍速)' % (
            (self.t_end - self.t0) / self.a.speedup, self.a.speedup))

    def pose_at(self, t):
        i = bisect.bisect_right(self.times, t) - 1
        i = min(max(i, 0), len(self.traj) - 1)
        j = min(i + 1, len(self.traj) - 1)
        (x0, y0), (x1, y1) = self.traj[i], self.traj[j]
        span = self.times[j] - self.times[i]
        f = 0.0 if span <= 0 else min(1.0, max(0.0, (t - self.times[i]) / span))
        x, y = x0 + f * (x1 - x0), y0 + f * (y1 - y0)
        # 向きは少し先の点から(止まっているときは直前の向き)
        k = next((k for k in range(j, len(self.traj))
                  if math.hypot(self.traj[k][0] - x, self.traj[k][1] - y) > 0.15), None)
        if k is not None:
            self.yaw = math.atan2(self.traj[k][1] - y, self.traj[k][0] - x)
        return x, y, getattr(self, 'yaw', 0.0)

    def _set(self, name, x, y, z, pitch, yaw):
        req = SetEntityState.Request()
        req.state = EntityState(name=name, reference_frame='world')
        req.state.pose.position.x, req.state.pose.position.y, req.state.pose.position.z = x, y, z
        o = req.state.pose.orientation
        o.x, o.y, o.z, o.w = quaternion(pitch, yaw)
        self.busy += 1
        self.set.call_async(req).add_done_callback(lambda _f: setattr(self, 'busy', self.busy - 1))

    def _events_until(self, t):
        while self.next_ev < len(self.events) and self.events[self.next_ev]['ros_time_s'] <= t:
            e = self.events[self.next_ev]
            self.next_ev += 1
            st = self.status
            st['state'] = STATE_OF.get(e['event'], st['state'])
            if e['event'] == 'mission_started':
                st['route'] = e['detail'].get('route', [])
                st['total'] = len(st['route'])
                st['strategy'] = e['detail'].get('strategy', '')
            if e['event'] == 'navigate':
                st.update(index=int(e['index']), point_id=e['point_id'],
                          name_en=e['detail'].get('name_en', self.names.get(e['point_id'], '')),
                          risk=e['detail'].get('risk'))
            if e['event'] == 'bear_detected':
                st['bears_found'] += 1
                d = dict(e['detail'])
                st['last_bear'] = {'point_id': d.get('point_id'), 'action': d.get('action')}
            if e['event'] == 'mission_completed':
                st['arrived'] = e['detail'].get('num_arrived', 0)
            msg = {k: e[k] for k in ('event', 'index', 'point_id', 'detail')}
            msg.update(wall_time=time.time(), ros_time_s=e['ros_time_s'], total=st['total'])
            self.event_pub.publish(String(data=json.dumps(msg, ensure_ascii=False)))

    def _ros_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        if self.start_wall is None:
            if self._ros_now() <= 0.0:      # /clock がまだ届いていない
                self.get_logger().info('Gazebo の /clock を待っています...', throttle_duration_sec=5.0)
                return
            self.start_wall = self._ros_now() + self.a.start_delay
        if self._ros_now() < self.start_wall:
            return
        # 再生の時計は ROS 時刻(use_sim_time なら Gazebo のシミュレーション時刻)。
        # Gazebo が実時間より遅くても、カメラ映像の時刻と動きがずれない
        t = self.t0 + (self._ros_now() - self.start_wall) * self.a.speedup
        if t > self.t_end:
            self.get_logger().info('再生が終わりました')
            raise SystemExit(0)
        x, y, yaw = self.pose_at(t)
        if self.busy < 4:
            self._set('replay_robot', x, y, 0.0, 0.0, yaw)
            self._set('replay_cam', x + 0.1 * math.cos(yaw), y + 0.1 * math.sin(yaw), 0.25, 0.0, yaw)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = 'map', 'base_footprint'
        tf.transform.translation.x, tf.transform.translation.y = x, y
        q = quaternion(0.0, yaw)
        r = tf.transform.rotation
        r.x, r.y, r.z, r.w = q
        self.tf.sendTransform(tf)
        self._events_until(t)
        i = bisect.bisect_right(self.times, t)
        self.status['distance_m'] = round(sum(
            math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(self.traj[:i], self.traj[1:i])), 1)
        self.status_pub.publish(String(data=json.dumps(self.status, ensure_ascii=False)))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--run', required=True, help='mission_planner の結果フォルダ')
    ap.add_argument('--model', default=os.environ.get('TURTLEBOT3_MODEL', 'waffle'),
                    help='見た目に使う TurtleBot3 のモデル')
    ap.add_argument('--rate', type=float, default=20.0)
    ap.add_argument('--speedup', type=float, default=1.0, help='再生の倍速')
    ap.add_argument('--start-delay', type=float, default=3.0, help='出してから動き出すまで [s]')
    ap.add_argument('--no-sim-time', dest='sim_time', action='store_false',
                    help='Gazebo の /clock を使わず壁時計で再生する')
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = Replay(args)
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=args.sim_time)])
    try:
        node.prepare()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
