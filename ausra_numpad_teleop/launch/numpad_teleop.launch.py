import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='ausrabot',
            description='Name of the robot (namespace)'
        ),
        Node(
            package='ausra_numpad_teleop',
            executable='numpad_teleop',
            name='numpad_teleop',
            output='screen',
            parameters=[{
                'robot_name': LaunchConfiguration('robot_name')
            }]
        )
    ])
