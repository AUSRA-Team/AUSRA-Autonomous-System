#!/usr/bin/env python3
"""
==============================================================================
TF Debug Launch File
==============================================================================
This launch file helps diagnose TF frame issues by:
1. Launching RViz with TF visualization
2. Running tf2_monitor to check TF tree health
3. Displaying TF frames and their publish rates

Usage:
    ros2 launch ausra_spawner debug_tf.launch.py robot_id:=1
==============================================================================
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id',
            default_value='1',
            description='Robot ID to monitor (for frame naming)'
        ),
        
        # RViz with TF debug visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='tf_debug_rviz',
            arguments=['-d', os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                'rviz', 'tf_debug.rviz'
            )],
            output='screen'
        ),
        
        # TF2 monitor to check TF tree health
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'tf2_ros', 'tf2_monitor'],
                    output='screen',
                    name='tf2_monitor'
                )
            ]
        ),
        
        # Echo TF between map and odom for robot 1
        # This helps verify the map->odom transform is being published by SLAM
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'tf2_ros', 'tf2_echo', 'map', 'ausra_1_odom'],
                    output='screen',
                    name='tf_echo_map_odom'
                )
            ]
        ),
    ])
