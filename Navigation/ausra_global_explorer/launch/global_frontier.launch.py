#!/usr/bin/env python3
"""
global_frontier.launch.py
Launches the frontier_coordinator on the base station (or single machine in sim).

This launch file is SEPARATE from the per-robot launch.
Run AFTER all robots are up and /map_merged is publishing.

Usage (simulation — single machine):
  ros2 launch ausra_global_explorer global_frontier.launch.py

Usage (override robots / map topic):
  ros2 launch ausra_global_explorer global_frontier.launch.py \\
    robot_names:=ausra_1,ausra_2,ausra_3 \\
    map_topic:=/map_merged

Usage (hardware — base station):
  ros2 launch ausra_global_explorer global_frontier.launch.py \\
    robot_names:=ausra_1,ausra_2
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ausra_global_explorer')
    params_file = os.path.join(pkg_share, 'config', 'coordinator_params.yaml')

    robot_names = LaunchConfiguration('robot_names')
    map_topic = LaunchConfiguration('map_topic')
    planning_rate_hz = LaunchConfiguration('planning_rate_hz')
    min_frontier_cells = LaunchConfiguration('min_frontier_cells')
    blacklist_radius_m = LaunchConfiguration('blacklist_radius_m')
    progress_timeout_s = LaunchConfiguration('progress_timeout_s')
    use_sim_time = LaunchConfiguration('use_sim_time')

    coordinator_node = Node(
        package='ausra_global_explorer',
        executable='frontier_coordinator',
        name='frontier_coordinator',
        output='screen',
        parameters=[
            params_file,
            {
                'robot_names': robot_names,
                'map_topic': map_topic,
                'planning_rate_hz': planning_rate_hz,
                'min_frontier_cells': min_frontier_cells,
                'blacklist_radius_m': blacklist_radius_m,
                'progress_timeout_s': progress_timeout_s,
                'use_sim_time': use_sim_time,
            },
        ],
    )

    # Option 1 — Robot obstacle publisher
    # Broadcasts each robot's position as a PointCloud2 halo into every other
    # robot's /neighbor_obstacles topic, feeding:
    #   • Local + global costmaps  → planners route around fleet
    #   • Collision Monitor        → hard stop before robot-robot contact
    obstacle_publisher_node = Node(
        package='ausra_global_explorer',
        executable='robot_obstacle_publisher',
        name='robot_obstacle_publisher',
        output='screen',
        parameters=[
            params_file,
            {
                'robot_names': robot_names,
                'use_sim_time': use_sim_time,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_names',
            default_value='ausra_1,ausra_2,ausra_3',
            description='Comma-separated robot namespaces (must match spawn_ausra_full.launch.py robot_id values)'
        ),
        DeclareLaunchArgument(
            'map_topic',
            default_value='/map_merged',
            description='Merged map topic (from ausra_map_merge/config/map_merge_params.yaml)'
        ),
        DeclareLaunchArgument(
            'planning_rate_hz',
            default_value='0.5',
            description='How often to recheck frontiers (Hz). 0.5 = every 2s for simulation'
        ),
        DeclareLaunchArgument(
            'min_frontier_cells',
            default_value='5',
            description='Minimum cluster size to consider a valid frontier'
        ),
        DeclareLaunchArgument(
            'blacklist_radius_m',
            default_value='0.5',
            description='Radius (m) around failed goals to avoid re-assigning'
        ),
        DeclareLaunchArgument(
            'progress_timeout_s',
            default_value='30.0',
            description='Cancel a navigation goal after this many seconds (sim: 30s)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use /clock from Gazebo (true for simulation, false for hardware)'
        ),
        LogInfo(msg='[ausra_global_explorer] Launching frontier_coordinator + robot_obstacle_publisher...'),
        LogInfo(msg='  Ensure robots were launched with use_exploration:=false'),
        LogInfo(msg='  Ensure /map_merged is publishing (ros2 topic hz /map_merged)'),
        coordinator_node,
        obstacle_publisher_node,
    ])

