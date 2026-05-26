import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 1. Get package paths
    pkg_description = get_package_share_directory('ausrabot_description')
    pkg_lidar = get_package_share_directory('sllidar_ros2')
    pkg_slam = get_package_share_directory('lidar_slam_pkg')

    # 2. Launch Configurations
    use_rviz = LaunchConfiguration('use_rviz', default='false')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')

    # 3. Paths to Configs and URDF
    config_hardware = os.path.join(pkg_description, 'config', 'hardware_params.yaml')
    config_slam = os.path.join(pkg_slam, 'config', 'slam_toolbox_config.yaml')
    xacro_file = os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')

    # 4. Robot State Publisher
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}]
    )

    # 5. Omnidirectional Driver
    omni_driver = Node(
        package='omnidirectional_driver',
        executable='omni_driver',
        name='omnidirectional_driver', 
        output='screen',
        parameters=[config_hardware]
    )

    # 6. RPLIDAR A1 Driver
    lidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lidar, 'launch', 'sllidar_a1_launch.py')
        ),
        launch_arguments={
            'serial_port': serial_port,
            'serial_baudrate': '115200',
            'frame_id': 'ausrabot_lidar'
        }.items()
    )

    # 7. SLAM Toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            config_slam, 
            {
            'use_sim_time': False,
            'odom_frame': LaunchConfiguration('odom_frame', default='odom'),
            'base_frame': LaunchConfiguration('base_frame', default='base_link'),
            'scan_topic': 'scan'
            }
        ]
    )

    # 8. Optional RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('odom_frame', default_value='odom', description='Odometry frame ID'),
        DeclareLaunchArgument('base_frame', default_value='base_link', description='Robot base frame ID'),
        robot_state_publisher,
        omni_driver,
        lidar_driver,
        slam_toolbox,
        rviz
    ])
