#!/usr/bin/env python3
"""
==============================================================================
AUSRA Exploration Visualization Launch
==============================================================================

This launch file starts RViz2 with proper configuration for visualizing:
- SLAM map building in real-time
- Frontier exploration markers (goals)
- Local and global costmaps
- Global and local path planning
- LiDAR scans and robot models
- TF frames for all robots

Usage:
    # Single robot mode (works with namespaced topics):
    ros2 launch ausra_spawner visualization.launch.py mode:=single robot_id:=1
    
    # Multi-robot mode (shows both robots):
    ros2 launch ausra_spawner visualization.launch.py mode:=multi

Arguments:
    mode: 'single' for one robot, 'multi' for multiple robots
    robot_id: Robot ID for single mode (creates namespace ausra_<id>)
    
==============================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('ausra_spawner')
    
    # Declare arguments
    declare_mode = DeclareLaunchArgument(
        'mode',
        default_value='single',
        description='Visualization mode: single or multi robot'
    )
    
    declare_robot_id = DeclareLaunchArgument(
        'robot_id',
        default_value='1',
        description='Robot ID for single mode (creates namespace ausra_<id>)'
    )
    
    # Get configurations
    mode = LaunchConfiguration('mode')
    robot_id = LaunchConfiguration('robot_id')
    
    # RViz config paths
    single_rviz = os.path.join(pkg_dir, 'rviz', 'single_robot_exploration.rviz')
    multi_rviz = os.path.join(pkg_dir, 'rviz', 'exploration_visualization.rviz')
    
    # Single robot RViz (namespaced)
    single_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=PythonExpression(["'ausra_' + '", robot_id, "'"]),
        arguments=['-d', single_rviz],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'single'"]))
    )
    
    # Multi robot RViz (global)
    multi_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', multi_rviz],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'multi'"]))
    )
    
    return LaunchDescription([
        declare_mode,
        declare_robot_id,
        single_rviz_node,
        multi_rviz_node,
    ])
