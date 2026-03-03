#!/usr/bin/env python3
"""
==============================================================================
AUSRA Multi-Robot Visualization Launch
==============================================================================

Launches RViz2 configured for multi-robot visualization.

Usage:
    ros2 launch ausra_spawner rviz_multirobot.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ausra_spawner = get_package_share_directory('ausra_spawner')
    
    rviz_config = LaunchConfiguration('rviz_config')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_ausra_spawner, 'rviz', 'multirobot_nav.rviz'),
            description='Path to RViz config file'
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])
