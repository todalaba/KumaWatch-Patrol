"""chase_camera ノード: Gazebo にロボットを追いかけるカメラを置き、その映像を配信する(動画撮影用).

俯瞰だとロボットが小さく、向きや到着が分かりにくい。このノードは Gazebo にカメラを 1 台出し、
ロボットの後ろ斜め上(mode=chase)や真上(mode=top)に置き続ける。映像は /chase_cam/image_raw に
出るので、image_recorder で録画すると、ミッションの字幕(HUD)入りの追従映像になる。

  ros2 run kumawatch_mission chase_camera --ros-args -p use_sim_time:=true
  ros2 run kumawatch_mission image_recorder --ros-args -p use_sim_time:=true \\
      -p image_topic:=/chase_cam/image_raw -p label:=chase

必要なもの: ワールドの gazebo_ros_state プラグイン(kumawatch_patrol.world に入っている)と
gzserver の gazebo_ros_factory(gazebo_ros の gzserver.launch.py が読み込む)。
"""

import math
import os

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import DeleteEntity, GetEntityState, SetEntityState, SpawnEntity
import rclpy
from rclpy.node import Node

from kumawatch_mission.chase_math import (camera_sdf, chase_pose, MODES, quaternion,
                                         ROBOT_CANDIDATES, smooth_angle)

class ChaseCamera(Node):

    def __init__(self):
        super().__init__('chase_camera')
        self.declare_parameter('robot_name', os.environ.get('TURTLEBOT3_MODEL', ''))
        self.declare_parameter('mode', 'chase')       # chase / far / top
        self.declare_parameter('camera_name', 'chase_cam')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 15)
        self.declare_parameter('hfov', 1.2)
        self.declare_parameter('rate', 20.0)          # カメラを動かす頻度 [Hz]
        self.declare_parameter('smoothing', 0.25)     # 0〜1。小さいほど滑らか(遅れる)

        self.cam = self.get_parameter('camera_name').value
        self.robot = self.get_parameter('robot_name').value
        self.candidates = [self.robot] if self.robot else list(ROBOT_CANDIDATES)
        self.spawned = False
        self.pose = None          # カメラの現在の (x, y, z, pitch, yaw)
        self.busy_get = self.busy_set = False

        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')
        self.get_cli = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        self.set_cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._tick)
        self.get_logger().info('追従カメラ: mode=%s → /%s/image_raw' % (
            self.get_parameter('mode').value, self.cam))

    def _spawn(self):
        if not self.spawn_cli.service_is_ready():
            self.get_logger().info('/spawn_entity を待っています(Gazebo は起動していますか)',
                                   throttle_duration_sec=10.0)
            return
        req = SpawnEntity.Request()
        req.name = self.cam
        req.xml = camera_sdf(self.cam, int(self.get_parameter('width').value),
                             int(self.get_parameter('height').value),
                             int(self.get_parameter('fps').value),
                             float(self.get_parameter('hfov').value))
        req.initial_pose.position.z = 3.0
        self.spawned = True
        self.spawn_cli.call_async(req).add_done_callback(self._on_spawned)

    def _on_spawned(self, future):
        res = future.result()
        if res is None or not res.success:
            msg = res.status_message if res else 'no response'
            if 'already exist' in msg:
                self.get_logger().info('カメラは既にあります。そのまま使います')
                return
            self.get_logger().error('カメラを出せませんでした: %s' % msg)
            self.spawned = False

    def _tick(self):
        if not self.spawned:
            self._spawn()
            return
        if not self.get_cli.service_is_ready():
            self.get_logger().error(
                '/gazebo/get_entity_state がありません。ワールドに gazebo_ros_state プラグインが'
                '必要です(kumawatch_patrol.world を作り直してください)', throttle_duration_sec=15.0)
            return
        if self.busy_get:
            return
        req = GetEntityState.Request()
        req.name = self.candidates[0]
        req.reference_frame = 'world'
        self.busy_get = True
        self.get_cli.call_async(req).add_done_callback(self._on_robot_state)

    def _on_robot_state(self, future):
        self.busy_get = False
        res = future.result()
        if res is None or not res.success:
            if len(self.candidates) > 1:           # モデル名を順に試す
                self.candidates = self.candidates[1:] + self.candidates[:1]
            self.get_logger().warn('ロボット %s が見つかりません(-p robot_name:=... で指定)'
                                   % (self.robot or '/'.join(ROBOT_CANDIDATES)),
                                   throttle_duration_sec=10.0)
            return
        if len(self.candidates) > 1:
            self.candidates = self.candidates[:1]
            self.get_logger().info('ロボット %s を追いかけます' % self.candidates[0])
        p, q = res.state.pose.position, res.state.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        back, height, pitch = MODES.get(self.get_parameter('mode').value, MODES['chase'])
        target = chase_pose(p.x, p.y, yaw, back, height, pitch)
        a = float(self.get_parameter('smoothing').value)
        if self.pose is None:
            self.pose = target
        else:
            x, y, z, _, cyaw = self.pose
            self.pose = (x + a * (target[0] - x), y + a * (target[1] - y), target[2], pitch,
                         smooth_angle(cyaw, target[4], a))
        self._move()

    def _move(self):
        if self.busy_set or not self.set_cli.service_is_ready():
            return
        x, y, z, pitch, yaw = self.pose
        qx, qy, qz, qw = quaternion(pitch, yaw)
        req = SetEntityState.Request()
        req.state = EntityState(name=self.cam, reference_frame='world')
        req.state.pose.position.x, req.state.pose.position.y = x, y
        req.state.pose.position.z = z
        o = req.state.pose.orientation
        o.x, o.y, o.z, o.w = qx, qy, qz, qw
        self.busy_set = True
        self.set_cli.call_async(req).add_done_callback(
            lambda _f: setattr(self, 'busy_set', False))

    def remove(self):
        if self.spawned and self.delete_cli.service_is_ready():
            req = DeleteEntity.Request()
            req.name = self.cam
            fut = self.delete_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = ChaseCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.remove()     # Gazebo からカメラを消す
        except Exception:  # noqa: BLE001 - 終了時は失敗しても構わない
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
