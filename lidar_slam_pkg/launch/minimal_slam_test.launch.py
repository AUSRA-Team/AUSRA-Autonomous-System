from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    pkg_lidar = get_package_share_directory('sllidar_ros2')
    slam_config = os.path.join(get_package_share_directory('lidar_slam_pkg'), 'config', 'slam_toolbox_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='ausra_1'),
        
        # 1. LiDAR Driver
        Node(package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
             parameters=[{'frame_id': [robot_name, '_lidar']}]),
             
        # 2. Static TF (Must be different for each robot!)
        # Use X=0 for ausra_1, X=1.0 for ausra_2 to keep them from overlapping
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['1.0', '0', '0', '0', '0', '0', 'map', [robot_name, '_map']]),

        # 3. Decentralized SLAM
        Node(package='slam_toolbox', executable='decentralized_multirobot_slam_toolbox_node',
             name='slam_toolbox', parameters=[slam_config, {
                 'odom_frame': [robot_name, '_odom'],
                 'map_frame': 'map',
                 'base_frame': [robot_name, '_base_link'],
                 'scan_topic': 'scan'
             }])
    ])