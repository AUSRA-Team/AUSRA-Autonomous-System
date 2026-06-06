import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('ausra_localization')
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Configs
    ekf_params = LaunchConfiguration('ekf_params', 
        default=os.path.join(pkg_dir, 'config', 'ekf.yaml'))
    imu_params = LaunchConfiguration('imu_params',
        default=os.path.join(pkg_dir, 'config', 'imu_complimentary_filter.yaml'))
    speckle_params = LaunchConfiguration('speckle_params',
        default=os.path.join(pkg_dir, 'config', 'speckle_filter.yaml'))

    # Nodes
    
    # IMU Filter (Complementary)
    imu_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        parameters=[imu_params, {'use_sim_time': use_sim_time}],
        remappings=[('/imu','/imu/data_filtered')]
    )

    # Robot Localization (EKF)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': use_sim_time}],
        remappings=[('/odometry/filtered', '/filtered_odometry')],
    )
    mpu6050_node = Node(
        package='mpu6050driver',
        executable='mpu6050driver',
        name='mpu6050publisher',
        output='screen',
        parameters=['/home/ausranano/ausra_ws2/src/ros2_mpu6050_driver/params/mpu6050.yaml'],
    )
    # Laser Filter (Optional, commented out in original but good to have ready)
    # speckle_filter_node = Node(
    #     package='laser_filters',
    #     executable="scan_to_scan_filter_chain",
    #     output='screen',
    #     parameters=[speckle_params, {'use_sim_time': use_sim_time}],
    #     remappings=[('scan', 'scan'), ('scan_filtered', 'scan_filtered')]
    # )

    # Ensure IMU starts before EKF to prevent timeout warnings
    start_ekf_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=imu_filter_node,
            on_start=[ekf_node]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'),
            
        # imu_filter_node,
        # start_ekf_event
        mpu6050_node,
        ekf_node
    ])
