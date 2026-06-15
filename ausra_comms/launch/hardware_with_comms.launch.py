import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nudge_robot = LaunchConfiguration('nudge_robot')
    use_zenoh = LaunchConfiguration('use_zenoh')

    pkg_lidar_slam = get_package_share_directory('lidar_slam_pkg')
    pkg_ausra_comms = get_package_share_directory('ausra_comms')

    zenoh_cfg = os.path.join(pkg_ausra_comms, 'config',
                             'zenoh_bridge_jetson.json5')

    zenoh_bin = os.environ.get(
        'ZENOH_BRIDGE_BIN',
        '/opt/zenoh-bridge/zenoh-bridge-ros2dds')

    return LaunchDescription([
        SetEnvironmentVariable(
            name='CYCLONEDDS_URI',
            value=(
                '<CycloneDDS>'
                  '<Domain>'
                    '<Discovery>'
                      '<MaxAutoParticipantIndex>500</MaxAutoParticipantIndex>'
                    '</Discovery>'
                  '</Domain>'
                '</CycloneDDS>'
            ),
        ),

        DeclareLaunchArgument('robot_name', default_value='ausra_1',
                               description='Robot name (e.g. ausra_1, ausra_2)'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                               description='Use simulation clock'),
        DeclareLaunchArgument('nudge_robot', default_value='false',
                               description='Auto-nudge robot to seed SLAM'),
        DeclareLaunchArgument('use_zenoh', default_value='true',
                               description='Start Zenoh cross-WiFi bridge'),
        DeclareLaunchArgument('enable_compression', default_value='false',
                               description='Enable zlib map compression'),

        LogInfo(msg='\n'
            '╔══════════════════════════════════════════════════════════════╗\n'
            '║         JETSON BRINGUP — Hardware + AUSRA Comms             ║\n'
            '╚══════════════════════════════════════════════════════════════╝\n'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_lidar_slam, 'launch',
                             'hardware_full_stack.launch.py')
            ),
            launch_arguments={
                'robot_name': robot_name,
                'use_sim_time': use_sim_time,
                'nudge_robot': nudge_robot,
            }.items(),
        ),

        # relay_node starts 10s after hardware to give SLAM time to begin publishing
        TimerAction(
            period=10.0,
            actions=[
                LogInfo(msg='>>> Starting relay_node...'),
                Node(
                    package='ausra_comms',
                    executable='relay_node',
                    name='relay_node',
                    output='screen',
                    parameters=[{
                        'robot_name': robot_name,
                        'map_interval_sec': 5.0,
                        'base_station_ip': '192.168.0.105',
                        'enable_compression': False,
                        'enable_adaptive_throttle': True,
                        'enable_delta_detection': True,
                        'delta_threshold': 0.01,
                    }],
                ),
            ]
        ),

        # Zenoh bridge starts 12s after hardware
        TimerAction(
            period=12.0,
            condition=IfCondition(use_zenoh),
            actions=[
                LogInfo(msg=[
                    '>>> Starting zenoh-bridge-ros2dds for ',
                    robot_name,
                    ' ...',
                ]),
                ExecuteProcess(
                    cmd=[
                        zenoh_bin,
                        '-c', zenoh_cfg,
                    ],
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0,
                    name='zenoh_bridge',
                ),
            ]
        ),
    ])
