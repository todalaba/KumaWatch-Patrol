"""chase_camera の計算(ROS 非依存): カメラの位置・向き、カメラモデルの SDF."""

import math

# (後ろへの距離 [m], 高さ [m], 見下ろす角度 [rad])
MODES = {
    'chase': (1.6, 1.1, 0.42),
    'far': (3.2, 2.4, 0.55),
    'top': (0.01, 6.0, 1.5),
}
ROBOT_CANDIDATES = ('happy_mini', 'waffle', 'waffle_pi', 'burger')


def camera_sdf(name, width, height, fps, hfov):
    """映像を /<name>/image_raw(と /<name>/camera_info)に出すカメラだけのモデル."""
    return f'''<?xml version="1.0"?>
<sdf version="1.6"><model name="{name}"><static>true</static>
  <link name="link"><gravity>false</gravity>
    <sensor name="cam" type="camera"><always_on>1</always_on><update_rate>{fps}</update_rate>
      <camera><horizontal_fov>{hfov}</horizontal_fov>
        <image><width>{width}</width><height>{height}</height></image>
        <clip><near>0.05</near><far>80</far></clip></camera>
      <plugin name="{name}_plugin" filename="libgazebo_ros_camera.so">
        <ros><namespace>/</namespace></ros><camera_name>{name}</camera_name>
        <frame_name>{name}_link</frame_name>
      </plugin>
    </sensor>
  </link></model></sdf>'''


def quaternion(pitch, yaw):
    """roll=0 のときの (x, y, z, w)."""
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (-sp * sy, sp * cy, cp * sy, cp * cy)


def chase_pose(robot_x, robot_y, robot_yaw, back, height, pitch):
    """ロボットの後ろ back [m]・高さ height [m] から見下ろすカメラ姿勢 (x, y, z, pitch, yaw)."""
    return (robot_x - back * math.cos(robot_yaw), robot_y - back * math.sin(robot_yaw),
            height, pitch, robot_yaw)


def smooth_angle(prev, new, alpha):
    d = math.atan2(math.sin(new - prev), math.cos(new - prev))
    return prev + alpha * d
