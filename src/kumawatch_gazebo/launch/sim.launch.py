"""Gazebo(Classic)に KumaWatch Patrol のワールドと TurtleBot3 を起動する.

  ros2 launch kumawatch_gazebo sim.launch.py                         # waffle
  ros2 launch kumawatch_gazebo sim.launch.py robot_model:=happy_mini # 講義環境の Happy Mini
  ros2 launch kumawatch_gazebo sim.launch.py sensor_viz:=false       # LiDAR の青い扇形を消す(動画向け)
"""

import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _robot(context):
    # turtlebot3_gazebo の launch は環境変数 TURTLEBOT3_MODEL でモデルを選ぶ。
    # 講義環境の ~/.bashrc は happy_mini を設定しているので、引数で明示的に上書きする。
    os.environ['TURTLEBOT3_MODEL'] = LaunchConfiguration('robot_model').perform(context)
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    def include(name, args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tb3_gazebo, 'launch', name)),
            launch_arguments=args.items())

    actions = [include('robot_state_publisher.launch.py', {'use_sim_time': 'true'})]
    pose = {'x_pose': LaunchConfiguration('x_pose'), 'y_pose': LaunchConfiguration('y_pose')}
    if LaunchConfiguration('sensor_viz').perform(context).lower() in ('true', '1'):
        actions.append(include('spawn_turtlebot3.launch.py', pose))
        return actions
    # LiDAR の青い扇形・カメラの視野表示を消したモデルを一時ファイルに書き出して置く
    model = os.environ['TURTLEBOT3_MODEL']
    with open(os.path.join(tb3_gazebo, 'models', 'turtlebot3_' + model, 'model.sdf'),
              encoding='utf-8') as f:
        sdf = re.sub(r'<visualize>\s*true\s*</visualize>', '<visualize>false</visualize>', f.read())
    tmp = tempfile.NamedTemporaryFile('w', suffix='_%s.sdf' % model, delete=False,
                                      encoding='utf-8')
    tmp.write(sdf)
    tmp.close()
    actions.append(Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-entity', model, '-file', tmp.name, '-x', pose['x_pose'],
                   '-y', pose['y_pose'], '-z', '0.01']))
    return actions


def generate_launch_description():
    gazebo_ros = get_package_share_directory('gazebo_ros')
    world = os.path.join(get_package_share_directory('kumawatch_gazebo'), 'worlds',
                         'kumawatch_patrol.world')
    points = os.path.join(get_package_share_directory('kumawatch_ai'), 'data',
                          'patrol_points.yaml')
    with open(points, encoding='utf-8') as f:
        base = yaml.safe_load(f)['base']

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('robot_model', default_value='waffle',
                              description='TurtleBot3 のモデル: waffle / waffle_pi / happy_mini'),
        DeclareLaunchArgument('x_pose', default_value=str(base['x'])),
        DeclareLaunchArgument('y_pose', default_value=str(base['y'])),
        DeclareLaunchArgument('sensor_viz', default_value='true',
                              description='false で LiDAR の青い扇形とカメラの視野表示を消す'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui'))),
        OpaqueFunction(function=_robot),
    ])
