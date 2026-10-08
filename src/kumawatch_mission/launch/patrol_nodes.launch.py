"""AI・ミッション系ノードの起動(Gazebo と Nav2 は別途起動済みであること).

  ros2 launch kumawatch_mission patrol_nodes.launch.py strategy:=utility budget_k:=3
  ros2 launch kumawatch_mission patrol_nodes.launch.py detector:=proximity   # レベル1の判定

detector:=camera    (既定) カメラ画像を YOLO で判定する bear_detector(AI)
detector:=proximity 熊の既知座標とロボットの距離で判定する bear_proximity_monitor(レベル1)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(get_package_share_directory('kumawatch_mission'), 'config',
                          'mission_params.yaml')
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('strategy', default_value='utility',
                              description='utility / topk / fixed'),
        DeclareLaunchArgument('budget_k', default_value='3', description='巡回地点数 K'),
        DeclareLaunchArgument('lambda_distance', default_value='0.02',
                              description='移動コストの重み λ [1/m]'),
        DeclareLaunchArgument('risk_mode', default_value='ttm',
                              description='ttm(KumaWatch TTM) / simulated / model'),
        DeclareLaunchArgument('on_bear', default_value='retreat',
                              description='熊検知後: continue / retreat(安全地点へ退避) / finish'),
        DeclareLaunchArgument('record', default_value='false',
                              description='true でロボットのカメラ映像を MP4 に録画'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('detector', default_value='camera',
                              description='camera(YOLO) / proximity(既知座標)'),
        DeclareLaunchArgument('detector_rate', default_value='5.0',
                              description='YOLO の推論頻度 [Hz]'),

        Node(
            package='kumawatch_ai', executable='risk_predictor', name='risk_predictor',
            output='screen',
            parameters=[params, {
                'use_sim_time': use_sim_time,
                'mode': LaunchConfiguration('risk_mode'),
            }],
        ),
        Node(
            package='kumawatch_ai', executable='bear_detector', name='bear_detector',
            output='screen',
            parameters=[params, {
                'use_sim_time': use_sim_time,
                'rate': ParameterValue(LaunchConfiguration('detector_rate'), value_type=float),
            }],
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('detector'), "' == 'camera'"])),
        ),
        Node(
            package='kumawatch_mission', executable='bear_proximity_monitor',
            name='bear_proximity_monitor', output='screen',
            parameters=[params, {'use_sim_time': use_sim_time}],
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('detector'), "' == 'proximity'"])),
        ),
        Node(
            package='kumawatch_mission', executable='mission_planner', name='mission_planner',
            output='screen', emulate_tty=True,
            parameters=[params, {
                'use_sim_time': use_sim_time,
                'strategy': LaunchConfiguration('strategy'),
                'budget_k': ParameterValue(LaunchConfiguration('budget_k'), value_type=int),
                'lambda_distance': ParameterValue(LaunchConfiguration('lambda_distance'),
                                                  value_type=float),
                'on_bear': LaunchConfiguration('on_bear'),
            }],
        ),
        Node(
            package='kumawatch_mission', executable='image_recorder', name='image_recorder',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'image_topic': LaunchConfiguration('image_topic')}],
            condition=IfCondition(LaunchConfiguration('record')),
        ),
        # 検出枠入りの映像(bear_detector の推論頻度で届く)も別ファイルに録画する
        Node(
            package='kumawatch_mission', executable='image_recorder',
            name='detection_recorder', output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'image_topic': '/kumawatch/detection_image',
                         'label': 'detection',
                         'fps': ParameterValue(LaunchConfiguration('detector_rate'),
                                               value_type=float)}],
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('record'), "'.lower() == 'true' and '",
                 LaunchConfiguration('detector'), "' == 'camera'"])),
        ),
    ])
