import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 1. Get package paths
    pkg_description = get_package_share_directory('ausrabot_description')
    pkg_lidar = get_package_share_directory('sllidar_ros2')
    pkg_slam = get_package_share_directory('lidar_slam_pkg')

    # 2. Paths to Configs and URDF
    # We point to the hardware config in the description package
    config_hardware = os.path.join(pkg_description, 'config', 'hardware_params.yaml')
    config_slam = os.path.join(pkg_slam, 'config', 'slam_toolbox_config.yaml')
    xacro_file = os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')

    # 3. Robot State Publisher (The "Skeleton")
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}]
    )

    # 4. Omnidirectional Driver (The "Muscles")
    # NAME MUST MATCH YAML: 'omnidirectional_driver'
    omni_driver = Node(
        package='omnidirectional_driver',
        executable='omni_driver',
        name='omnidirectional_driver', 
        output='screen',
        parameters=[config_hardware]
    )

    # 5. RPLIDAR A1 Driver (The "Eyes")
    lidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lidar, 'launch', 'sllidar_a1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': '115200',
            'frame_id': 'ausrabot_lidar' # Matches your sensors.xacro
        }.items()
    )

    # 6. SLAM Toolbox (The "Brain")
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            config_slam, 
            {
            'use_sim_time': False,
            'odom_frame': 'ausrabot_odom',             # Match your driver
            'base_frame': 'ausrabot_robot_footprint',  # Match your URDF root
            'scan_topic': '/scan'
            }
        ]
    )


    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2'
    )

    return LaunchDescription([
        robot_state_publisher,
        omni_driver,
        lidar_driver,
        slam_toolbox,
        rviz
    ])