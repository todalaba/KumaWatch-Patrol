"""Nav2(事前地図 + AMCL)と RViz を起動する.

  ros2 launch kumawatch_navigation navigation.launch.py

引数名は nav2_map / nav2_params としている。ROS 2 Humble では include した launch の
引数が呼び出し元に漏れるため、gazebo_ros の gzserver.launch.py が宣言する params_file(既定 '')
と同じ名前にすると、既定値が効かずに空のパスが Nav2 に渡ってしまう。
さらに念のため、パスは起動時に実在を確かめ、空・存在しない場合は同梱の地図・設定に戻す
(どこから空の値が漏れてきても Nav2 に '' を渡さない)。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve(context, name, default):
    value = LaunchConfiguration(name).perform(context).strip()
    if value and os.path.isfile(value):
        return value, None
    if value:
        return default, '%s=%s が見つからないので同梱のファイルを使います' % (name, value)
    return default, None


def _nav2(context, share):
    nav2_map, warn_map = _resolve(context, 'nav2_map',
                                  os.path.join(share, 'maps', 'kumawatch_map.yaml'))
    nav2_params, warn_params = _resolve(context, 'nav2_params',
                                        os.path.join(share, 'config', 'nav2_params.yaml'))
    bringup = os.path.join(get_package_share_directory('nav2_bringup'), 'launch',
                           'bringup_launch.py')
    actions = [LogInfo(msg=w) for w in (warn_map, warn_params) if w]
    actions += [
        LogInfo(msg='Nav2 を起動します: map=%s params=%s' % (nav2_map, nav2_params)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments={
                'map': nav2_map,
                'params_file': nav2_params,
                'use_sim_time': 'true',
                'autostart': 'true',
                'namespace': '',
                'use_namespace': 'false',
                'slam': 'False',
            }.items()),
    ]
    return actions


def generate_launch_description():
    share = get_package_share_directory('kumawatch_navigation')

    return LaunchDescription([
        DeclareLaunchArgument('nav2_map', default_value=os.path.join(share, 'maps',
                                                                'kumawatch_map.yaml')),
        DeclareLaunchArgument('nav2_params',
                              default_value=os.path.join(share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),

        OpaqueFunction(function=_nav2, args=[share]),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            arguments=['-d', os.path.join(share, 'rviz', 'kumawatch_patrol.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
