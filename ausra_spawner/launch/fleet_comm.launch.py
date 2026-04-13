#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    fleet_comm_node = Node(
        package='ausra_spawner',
        executable='fleet_comm_node',
        name='fleet_comm_node',
        output='screen',
        parameters=[{
            'namespaces': ['ausra_1', 'ausra_2', 'ausra_3'],
            'comm_range': 20.0,
            'update_rate': 2.0,
            'robot_radius': 0.2
        }]
    )

    return LaunchDescription([
        fleet_comm_node
    ])
