# ============================================================
# FILE: robot_comms.launch.py
# RUNS ON: Jetson (e.g. ausra_1, ausra_2)
# PURPOSE: Launches relay_node to namespace SLAM topics.
#          /map → /<robot_name>/map, /pose → /<robot_name>/pose
#          + publishes /<robot_name>/heartbeat at 1 Hz.
#
#          Cross-machine transport is handled by ROS2 DDS
#          natively — no bridge needed. Both machines must
#          be on the same WiFi + same ROS_DOMAIN_ID.
#
# LAUNCH ARGUMENTS:
#   robot_name — robot name string (e.g. 'ausra_1', 'ausra_2')
# ============================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='ausra_1',
            description='Robot name (e.g. ausra_1, ausra_2)',
        ),

        # --- Relay node: /map → /<robot_name>/map, /pose → /<robot_name>/pose ---
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
    ])
