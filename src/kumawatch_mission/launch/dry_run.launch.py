"""Gazebo・Nav2 なしの動作確認(fake_nav2 で直線移動を模擬).

  ros2 launch kumawatch_mission dry_run.launch.py                    # AI 巡回
  ros2 launch kumawatch_mission dry_run.launch.py strategy:=fixed    # 固定ルート
  ros2 launch kumawatch_mission dry_run.launch.py time_scale:=5.0    # 5倍速
  # 退避地点へ行けない場合(2〜6 番目のゴールを失敗させる → SAFE_HOLD で停止して指示待ち)
  ros2 launch kumawatch_mission dry_run.launch.py on_bear:=retreat abort_goals:=2,3,4,5,6
  ros2 topic pub --once /kumawatch/operator_command std_msgs/String "{data: return}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def generate_launch_description():
    points = os.path.join(get_package_share_directory('kumawatch_ai'), 'data',
                          'patrol_points.yaml')
    with open(points, encoding='utf-8') as f:
        base = yaml.safe_load(f)['base']
    nodes = os.path.join(get_package_share_directory('kumawatch_mission'), 'launch',
                         'patrol_nodes.launch.py')
    lc = LaunchConfiguration

    return LaunchDescription([
        DeclareLaunchArgument('strategy', default_value='utility'),
        DeclareLaunchArgument('budget_k', default_value='3'),
        DeclareLaunchArgument('lambda_distance', default_value='0.02'),
        DeclareLaunchArgument('risk_mode', default_value='ttm'),
        DeclareLaunchArgument('on_bear', default_value='continue'),
        DeclareLaunchArgument('time_scale', default_value='1.0', description='早送り倍率'),
        DeclareLaunchArgument('abort_goals', default_value='',
                              description='試験用: 失敗させるゴールの番号(例 "2,3,4,5,6")'),

        Node(package='kumawatch_mission', executable='fake_nav2', name='fake_nav2',
             output='screen', parameters=[{
                 'start_x': float(base['x']), 'start_y': float(base['y']),
                 'start_yaw': float(base.get('yaw', 0.0)),
                 'time_scale': ParameterValue(lc('time_scale'), value_type=float),
                 'abort_goals': ParameterValue(lc('abort_goals'), value_type=str),
             }]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(nodes), launch_arguments={
            'use_sim_time': 'false',
            'strategy': lc('strategy'),
            'budget_k': lc('budget_k'),
            'lambda_distance': lc('lambda_distance'),
            'risk_mode': lc('risk_mode'),
            'on_bear': lc('on_bear'),
            'detector': 'proximity',   # fake_nav2 にはカメラがないので既知座標で判定
        }.items()),
    ])
