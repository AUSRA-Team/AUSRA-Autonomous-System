import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('slam_explorer')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    explore_params_file = os.path.join(pkg_dir, 'config', 'explore.yaml')
    slam_params_file = os.path.join(pkg_dir, 'config', 'slam_omni_single_robot.yaml')
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    ekf_params = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    imu_params = os.path.join(pkg_dir, 'config', 'imu_complimentary_filter.yaml')
    # speckle_filter_params = os.path.join(pkg_dir, 'config', 'speckle_filter.yaml')

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'true', # Change to 'false' for real robot
            'slam_params_file': slam_params_file
        }.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file
        }.items()
    )

    # speckle_filter_node = Node(
    #     package='laser_filters',
    #     executable="scan_to_scan_filter_chain",
    #     output='screen',
    #     parameters=[speckle_filter_params, {'use_sim_time': True}],
    #     remappings=[('scan', 'scan'), ('scan_filtered', 'scan_filtered')]
    # )

    imu_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        parameters=[imu_params, {'use_sim_time': True}],
        remappings=[('imu/data', '/imu/data_filtered')]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': True}],
        remappings=[('odometry/filtered', '/filtered_odometry')],
    )

    explore_node = Node(
        package='explore_lite',
        name='explore_node',
        executable='explore',
        parameters=[explore_params_file, {'use_sim_time': True}],
        output='screen',
    )

    # Event Handlers
    start_ekf_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=imu_filter_node,
            on_start=[ekf_node]
        )
    )

    start_slam_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ekf_node,
            on_start=[slam_launch]
        )
    )

    start_nav2_event = TimerAction(
        period=3.0,
        actions=[nav2_launch]
    )

    start_explore_event = TimerAction(
        period=5.0,
        actions=[explore_node]
    )

    return LaunchDescription([
        # speckle_filter_node,
        imu_filter_node,
        start_ekf_event,
        start_slam_event,
        start_nav2_event,
        start_explore_event
    ])