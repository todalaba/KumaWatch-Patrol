"""KumaWatch Patrol 一括起動: Gazebo + TurtleBot3 + Nav2 + RViz + AI/ミッション.

  ros2 launch kumawatch_mission kumawatch_patrol.launch.py
  ros2 launch kumawatch_mission kumawatch_patrol.launch.py robot_model:=happy_mini  # 講義環境
  ros2 launch kumawatch_mission kumawatch_patrol.launch.py strategy:=fixed   # 固定ルートと比較
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            LogInfo, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(pkg, name, args):
    """launch を include する. Humble の include は引数のスコープを分けないため、
    GroupAction(scoped=True) で包んで、include 先で宣言された引数(gazebo_ros の params_file など)が
    ほかの include に漏れないようにする。"""
    path = os.path.join(get_package_share_directory(pkg), 'launch', name)
    return GroupAction(scoped=True, actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                 launch_arguments=args.items())])


def generate_launch_description():
    lc = LaunchConfiguration

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo の GUI を表示'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz を表示'),
        DeclareLaunchArgument('robot_model', default_value='waffle',
                              description='TurtleBot3 のモデル: waffle / waffle_pi / happy_mini'),
        DeclareLaunchArgument('strategy', default_value='utility',
                              description='utility / topk / fixed'),
        DeclareLaunchArgument('budget_k', default_value='3'),
        DeclareLaunchArgument('lambda_distance', default_value='0.02'),
        DeclareLaunchArgument('risk_mode', default_value='ttm'),
        DeclareLaunchArgument('on_bear', default_value='retreat'),
        DeclareLaunchArgument('detector', default_value='camera',
                              description='熊の判定: camera(YOLO) / proximity(既知座標)'),
        DeclareLaunchArgument('record', default_value='false',
                              description='true でロボットのカメラ映像を MP4 に録画'),
        DeclareLaunchArgument('sensor_viz', default_value='true',
                              description='false で Gazebo の LiDAR の青い扇形を消す(動画向け)'),
        DeclareLaunchArgument('nav2_delay', default_value='5.0',
                              description='Gazebo 起動から Nav2 起動までの待ち時間 [s]'),

        # この表示が出ない場合は、古いビルドが動いている(docs/lecture_env.md のトラブル対処)
        LogInfo(msg=['[KumaWatch Patrol] robot_model=', lc('robot_model'),
                     '。Gazebo の起動から ', lc('nav2_delay'),
                     ' 秒後に Nav2・AI ノードを起動します']),
        _include('kumawatch_gazebo', 'sim.launch.py', {
            'gui': lc('gui'), 'robot_model': lc('robot_model'), 'sensor_viz': lc('sensor_viz')}),
        TimerAction(period=lc('nav2_delay'), actions=[
            _include('kumawatch_navigation', 'navigation.launch.py', {'rviz': lc('rviz')}),
            _include('kumawatch_mission', 'patrol_nodes.launch.py', {
                'strategy': lc('strategy'),
                'budget_k': lc('budget_k'),
                'lambda_distance': lc('lambda_distance'),
                'risk_mode': lc('risk_mode'),
                'on_bear': lc('on_bear'),
                'record': lc('record'),
                'detector': lc('detector'),
            }),
        ]),
    ])
