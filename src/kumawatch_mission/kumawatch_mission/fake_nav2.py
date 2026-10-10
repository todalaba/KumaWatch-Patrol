"""fake_nav2 ノード: Gazebo/Nav2 なしでミッションを試すための簡易シミュレータ.

Nav2 の代わりに NavigateToPose アクションサーバを提供し、ロボットを目標へ直線で
一定速度で動かす(障害物は無視)。TF(map→odom→base_footprint)、/odom、/amcl_pose、
/bt_navigator/get_state(常に active)も配信するので、mission_planner と
bear_proximity_monitor をそのまま動かせる。CI やノートPCでの動作確認用。

  ros2 launch kumawatch_mission dry_run.launch.py
"""

import math
import threading
import time

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FakeNav2(Node):

    def __init__(self):
        super().__init__('fake_nav2')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', -8.5)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('speed', 0.26)        # [m/s]
        self.declare_parameter('time_scale', 1.0)    # >1 で早送り
        self.declare_parameter('rate', 20.0)
        # 試験用: 受け取った順で何番目のゴールを「経路なし」で失敗させるか(例 "2,3,4,5,6")
        self.declare_parameter('abort_goals', '')
        self.abort_goals = {int(v) for v in str(self.get_parameter('abort_goals').value).split(',')
                            if v.strip()}
        self.goal_count = 0

        self.x = float(self.get_parameter('start_x').value)
        self.y = float(self.get_parameter('start_y').value)
        self.yaw = float(self.get_parameter('start_yaw').value)
        self.v = 0.0
        self.lock = threading.Lock()
        self.active_goal = None
        cb = ReentrantCallbackGroup()

        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self._send_map_to_odom()
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.amcl_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', latched)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10, callback_group=cb)
        self.create_service(GetState, '/bt_navigator/get_state', self._on_get_state,
                            callback_group=cb)
        ActionServer(self, NavigateToPose, 'navigate_to_pose', self._execute,
                     goal_callback=self._on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                     callback_group=cb)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._publish_state,
                          callback_group=cb)
        self._publish_amcl()
        self.get_logger().info('fake_nav2 started at (%.2f, %.2f)' % (self.x, self.y))

    def _send_map_to_odom(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.rotation.w = 1.0
        self.static_tf.sendTransform(t)

    def _on_get_state(self, _req, res):
        res.current_state.id = State.PRIMARY_STATE_ACTIVE
        res.current_state.label = 'active'
        return res

    def _on_cmd_vel(self, msg):
        # mission_planner の緊急停止(ゼロ速度)を受け付ける
        if msg.linear.x == 0.0 and msg.angular.z == 0.0:
            with self.lock:
                self.v = 0.0

    def _on_goal(self, _goal):
        return GoalResponse.ACCEPT

    def _execute(self, handle):
        target = handle.request.pose.pose
        tx, ty = target.position.x, target.position.y
        tyaw = yaw_from_quaternion(target.orientation)
        speed = float(self.get_parameter('speed').value)
        scale = float(self.get_parameter('time_scale').value)
        dt = 1.0 / float(self.get_parameter('rate').value)
        self.active_goal = handle
        self.goal_count += 1
        self.get_logger().info('goal #%d (%.2f, %.2f)' % (self.goal_count, tx, ty))
        result = NavigateToPose.Result()
        if self.goal_count in self.abort_goals:   # 経路が作れなかった場合の模擬
            self.get_logger().warn('goal #%d aborted (simulated: no path)' % self.goal_count)
            handle.abort()
            return result
        while rclpy.ok():
            if handle.is_cancel_requested:
                with self.lock:
                    self.v = 0.0
                handle.canceled()
                self.get_logger().info('goal canceled at (%.2f, %.2f)' % (self.x, self.y))
                return result
            if self.active_goal is not handle:  # 新しいゴールに置き換えられた
                handle.abort()
                return result
            with self.lock:
                dx, dy = tx - self.x, ty - self.y
                d = math.hypot(dx, dy)
                step = speed * dt * scale
                if d <= step:
                    self.x, self.y, self.yaw, self.v = tx, ty, tyaw, 0.0
                else:
                    self.yaw = math.atan2(dy, dx)
                    self.x += dx / d * step
                    self.y += dy / d * step
                    self.v = speed
            if d <= step:
                break
            fb = NavigateToPose.Feedback()
            fb.distance_remaining = float(d)
            handle.publish_feedback(fb)
            time.sleep(dt)
        handle.succeed()
        self._publish_amcl()
        return result

    def _publish_amcl(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2)
        self.amcl_pub.publish(msg)

    def _publish_state(self):
        with self.lock:
            x, y, yaw, v = self.x, self.y, self.yaw, self.v
        stamp = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = math.sin(yaw / 2)
        t.transform.rotation.w = math.cos(yaw / 2)
        self.tf.sendTransform(t)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist.linear.x = v
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = FakeNav2()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
