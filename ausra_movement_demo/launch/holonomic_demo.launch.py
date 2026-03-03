#!/usr/bin/env python3
"""
Launch file for the Holonomic Movement Demo node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare parameters
    linear_velocity_arg = DeclareLaunchArgument(
        'linear_velocity',
        default_value='0.2',
        description='Linear velocity for movements (m/s)'
    )

    angular_velocity_arg = DeclareLaunchArgument(
        'angular_velocity',
        default_value='0.3',
        description='Angular velocity for rotations (rad/s)'
    )

    movement_distance_arg = DeclareLaunchArgument(
        'movement_distance',
        default_value='1.0',
        description='Distance to move in each direction (meters)'
    )

    # Demo node
    holonomic_demo_node = Node(
        package='ausra_movement_demo',
        executable='holonomic_demo',
        name='holonomic_movement_demo',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'linear_velocity': LaunchConfiguration('linear_velocity'),
            'angular_velocity': LaunchConfiguration('angular_velocity'),
            'movement_distance': LaunchConfiguration('movement_distance'),
        }]
    )

    return LaunchDescription([
        linear_velocity_arg,
        angular_velocity_arg,
        movement_distance_arg,
        holonomic_demo_node,
    ])
