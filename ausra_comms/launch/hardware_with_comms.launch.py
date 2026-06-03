# ============================================================
# FILE: hardware_with_comms.launch.py
# RUNS ON: Jetson (e.g. ausra_1, ausra_2)
# PURPOSE: Top-level launch that starts EVERYTHING on the Jetson:
#          1. hardware_full_stack (drivers, EKF, SLAM, Nav2, explore)
#          2. relay_node (namespaces /map → /<robot_name>/map, etc.)
#
#          The relay starts 10s after hardware to give SLAM
#          time to begin publishing /map.
#
# LAUNCH ARGUMENTS:
#   robot_name   — robot name string (default: ausra_1)
#   use_sim_time — false for hardware (default: false)
#   nudge_robot  — auto-nudge to seed SLAM (default: false)
#
# PREREQUISITE: lidar_slam_pkg must be built on the Jetson.
# ============================================================

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction, LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nudge_robot = LaunchConfiguration('nudge_robot')

    pkg_lidar_slam = get_package_share_directory('lidar_slam_pkg')

    return LaunchDescription([
        # --- Arguments ---
        DeclareLaunchArgument('robot_name', default_value='ausra_1',
                              description='Robot name (e.g. ausra_1, ausra_2)'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation clock'),
        DeclareLaunchArgument('nudge_robot', default_value='false',
                              description='Auto-nudge robot to seed SLAM'),

        # --- Stage A: Hardware Full Stack (immediate) ---
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

        # --- Stage B: Relay Node (10s delay — SLAM needs time) ---
        TimerAction(
            period=10.0,
            actions=[
                LogInfo(msg='>>> Starting relay_node (AUSRA comms layer)...'),
                Node(
                    package='ausra_comms',
                    executable='relay_node',
                    name='relay_node',
                    output='screen',
                    parameters=[{
                        'robot_name': robot_name,
                        'map_interval_sec': 5.0,
                    }],
                ),
            ]
        ),
    ])
