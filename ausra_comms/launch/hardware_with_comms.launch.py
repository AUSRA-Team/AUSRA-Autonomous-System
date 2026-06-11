# ============================================================
# FILE: hardware_with_comms.launch.py
# RUNS ON: Jetson (e.g. ausra_1, ausra_2)
# PURPOSE: Top-level launch that starts EVERYTHING on the Jetson:
#          1. hardware_full_stack (drivers, EKF, SLAM, Nav2, explore)
#          2. relay_node (namespaces /map → /<robot_name>/map, etc.)
#          3. zenoh-bridge-ros2dds (cross-WiFi transport, allowlist-only)
#
#          The relay starts 10s after hardware to give SLAM
#          time to begin publishing /map.
#          The Zenoh bridge starts 12s after hardware (just after the
#          relay) so its discovery sees the namespaced topics.
#
# LAUNCH ARGUMENTS:
#   robot_name   — robot name string (default: ausra_1)
#   use_sim_time — false for hardware (default: false)
#   nudge_robot  — auto-nudge to seed SLAM (default: false)
#   use_zenoh    — start the Zenoh cross-WiFi bridge (default: true)
#                  Set false to revert to plain DDS (also requires
#                  unsetting ROS_LOCALHOST_ONLY=1 — see ZENOH_GUIDE.md).
#
# ENVIRONMENT:
#   ROS_LOCALHOST_ONLY=1  — enforced when use_zenoh=true. The Zenoh bridge
#                            becomes the ONLY cross-machine channel.
#   ZENOH_BRIDGE_BIN      — override path to zenoh-bridge-ros2dds binary.
#                            Defaults to /opt/zenoh-bridge/zenoh-bridge-ros2dds.
#
# PREREQUISITE: lidar_slam_pkg must be built on the Jetson.
#               zenoh-bridge-ros2dds must be installed (see ZENOH_GUIDE.md).
# ============================================================

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

    # Bridge binary location — override via ZENOH_BRIDGE_BIN env var.
    zenoh_bin = os.environ.get(
        'ZENOH_BRIDGE_BIN',
        '/opt/zenoh-bridge/zenoh-bridge-ros2dds')

    return LaunchDescription([
        # --- CycloneDDS: raise participant index limit ---
        # The full hardware stack spawns 15+ ROS nodes, exhausting the
        # default 120-slot limit.  Without this the Zenoh bridge crashes
        # with "Failed to find a free participant index for domain 0".
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

        # --- Arguments ---
        DeclareLaunchArgument('robot_name', default_value='ausra_1',
                              description='Robot name (e.g. ausra_1, ausra_2)'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation clock'),
        DeclareLaunchArgument('nudge_robot', default_value='false',
                              description='Auto-nudge robot to seed SLAM'),
        DeclareLaunchArgument('use_zenoh', default_value='true',
                              description='Start Zenoh cross-WiFi bridge'),

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

        # --- Stage C: Zenoh Bridge (12s delay — relay must be publishing) ---
        # Decoupled from relay/SLAM/Nav2 lifecycle:
        #   - respawn=True: bridge restarts on its own if it crashes
        #   - on_exit not bound to anything: relay/SLAM keep running on
        #     bridge crash; the robot keeps mapping/navigating without WiFi.
        #
        # relay_node already namespaces topics (e.g. /ausra_1/map), so the
        # bridge forwards them as-is — no extra -n namespace flag needed.
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
