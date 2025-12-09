#!/usr/bin/env python3
# ============================================================================
# Cartographer Launch File for Omni-Wheel Robot in Gazebo
# ============================================================================
# Directory structure:
#   cartographer/
#   ├── config/Robot.lua
#   └── launch/cartographer_mapping.launch.py (this file)
# ============================================================================

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Resolution of occupancy grid map (meters per cell)'
    )
    
    publish_period_sec_arg = DeclareLaunchArgument(
        'publish_period_sec',
        default_value='1.0',
        description='How often to publish the occupancy grid map (seconds)'
    )
    
    # ========================================================================
    # CONFIGURATION FILE PATHS (RELATIVE - MORE PORTABLE)
    # ========================================================================
    
    # Get the directory where THIS launch file is located
    # This file is in: ~/SLAM/AUSRA-Autonomous-System/SLAM/cartographer/launch/
    launch_file_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Configuration directory is ../config/ (one level up, then into config/)
    configuration_directory = os.path.join(
        os.path.dirname(launch_file_dir),  # Go up one level to cartographer/
        'config'                            # Then into config/
    )
    
    configuration_basename = 'Robot.lua'
    
    # ========================================================================
    # NODES
    # ========================================================================
    
    # Cartographer SLAM node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename
        ],
        remappings=[
            ('scan', '/scan'),
           # ('odom', '/odom'),
        ]
    )
    
    # Occupancy grid node
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'resolution': LaunchConfiguration('resolution'),
            'publish_period_sec': LaunchConfiguration('publish_period_sec')
        }]
    )
    
    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        resolution_arg,
        publish_period_sec_arg,
        
        # Info messages
        LogInfo(msg='Starting Cartographer SLAM for omni-wheel robot...'),
        LogInfo(msg=['Configuration directory: ', configuration_directory]),
        LogInfo(msg=['Configuration file: ', configuration_basename]),
        
        # Nodes
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])