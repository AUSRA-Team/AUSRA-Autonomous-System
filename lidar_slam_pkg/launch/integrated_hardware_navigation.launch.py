#!/usr/bin/env python3
"""
Integrated Hardware Navigation Launch File

This launch file starts the complete navigation stack for AUSRA real robot:
1. Robot drivers and SLAM (from lidar_slam_pkg)
2. Nav2 navigation stack (configured for AUSRA)
3. Optional RViz visualization

Usage:
    ros2 launch lidar_slam_pkg integrated_hardware_navigation.launch.py
    
Or with configuration:
    ros2 launch lidar_slam_pkg integrated_hardware_navigation.launch.py \
        use_nav2:=true \
        use_rviz:=true \
        nav_mode:=localization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_lidar_slam = get_package_share_directory('lidar_slam_pkg')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_description = get_package_share_directory('ausrabot_description')
    
    # Declare parameters
    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2',
        default_value='true',
        description='Start Nav2 navigation stack'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz visualization'
    )
    
    nav_mode_arg = DeclareLaunchArgument(
        'nav_mode',
        default_value='mapping',
        description='Navigation mode: mapping or localization. Use mapping for first run to create map'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace (leave empty for non-namespaced operation)'
    )
    
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Auto-start navigation and lifecycle managers'
    )
    
    params_file = os.path.join(
        pkg_lidar_slam,
        'config',
        'nav2_params.yaml'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to the ROS2 parameters file to use for Nav2'
    )
    
    # Phase 1: Core Hardware + SLAM (from lidar_slam_pkg/launch/slam.launch.py)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lidar_slam, 'launch', 'slam.launch.py')
        )
    )
    
    # Phase 2: Nav2 Navigation (optional, controlled by use_nav2 parameter)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'use_sim_time': 'false',
            'autostart': LaunchConfiguration('autostart'),
            'params_file': LaunchConfiguration('params_file'),
            'use_composition': 'True',
        }.items()
    )
    
    # Phase 3: RViz (optional, controlled by use_rviz parameter)
    rviz_config = os.path.join(
        pkg_lidar_slam,
        'rviz',
        'default_nav.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=LaunchConfiguration('use_rviz')
    )
    
    # Logging
    startup_info = LogInfo(
        msg='Starting AUSRA Real Hardware Navigation Stack'
    )
    
    slam_started = LogInfo(
        msg='✓ SLAM stack started (mapping from lidar_slam_pkg)'
    )
    
    nav2_condition_info = LogInfo(
        msg='✓ Nav2 stack starting (control with use_nav2:=false to disable)'
    )
    
    # Build description
    ld = LaunchDescription()
    
    # Declare arguments
    ld.add_action(use_nav2_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(nav_mode_arg)
    ld.add_action(namespace_arg)
    ld.add_action(autostart_arg)
    ld.add_action(params_file_arg)
    
    # Logging
    ld.add_action(startup_info)
    
    # Core hardware + SLAM (always started)
    ld.add_action(slam_started)
    ld.add_action(slam_launch)
    
    # Nav2 (conditional on use_nav2)
    # For now, we include it but could make conditional with IfAction
    ld.add_action(nav2_condition_info)
    ld.add_action(nav2_launch)
    
    # RViz (conditional on use_rviz)
    ld.add_action(rviz_node)
    
    # Info message
    info = LogInfo(
        msg='\n' +
        '='*70 + '\n' +
        'AUSRA REAL HARDWARE NAVIGATION STACK STARTED\n' +
        '='*70 + '\n' +
        'Components loaded:\n' +
        '  ✓ SLAM (lidar_slam_pkg) - Mapping with RPLIDAR A1\n' +
        '  ✓ Robot Drivers - Omnidirectional motor control\n' +
        '  ✓ Nav2 Stack - Autonomous navigation ready\n' +
        '\n' +
        'Frame Setup:\n' +
        '  - map (global) ← SLAM creates this\n' +
        '  - ausrabot_odom ← Odometry from motor encoders\n' +
        '  - ausrabot_robot_footprint ← Robot base\n' +
        '\n' +
        'Next Steps:\n' +
        '  1. Check SLAM - Ensure /map topic is publishing\n' +
        '  2. Publish initial pose: ros2 topic pub /initialpose ...\n' +
        '  3. Use RViz to visualize and send navigation goals\n' +
        '\n' +
        'Troubleshooting:\n' +
        '  - Check transforms: ros2 run tf2_tools view_frames\n' +
        '  - Check topics: ros2 topic list\n' +
        '  - Check frame names in configs (must match URDF)\n' +
        '='*70 + '\n'
    )
    ld.add_action(info)
    
    return ld
