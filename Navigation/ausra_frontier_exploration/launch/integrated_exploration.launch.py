import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction, DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('ausra_frontier_exploration')
    slam_explorer_dir = get_package_share_directory('slam_explorer')
    localization_dir = get_package_share_directory('ausra_localization')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    ausra_spawner_dir = get_package_share_directory('ausra_spawner')
    
    # Use unified Nav2 params from ausra_spawner (single source of truth)
    nav2_params_file = os.path.join(ausra_spawner_dir, 'config', 'nav2_params.yaml')
    
    # Params
    slam_params_file = os.path.join(slam_explorer_dir, 'config', 'slam_omni_single_robot.yaml')
    ekf_params = os.path.join(localization_dir, 'config', 'ekf.yaml')
    imu_params = os.path.join(localization_dir, 'config', 'imu_complimentary_filter.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file
        }.items()
    )

    # Nav2 Bringup (using MY new params)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file
        }.items()
    )



    namespace = LaunchConfiguration('namespace')
    
    # Namespaced group
    exploration_group = GroupAction([
        PushRosNamespace(namespace),
        
        # IMU Filter
        Node(
            package='imu_complementary_filter',
            executable='complementary_filter_node',
            name='complementary_filter_gain_node',
            output='screen',
            parameters=[imu_params, {'use_sim_time': use_sim_time}],
            remappings=[('imu/data', 'imu/data_filtered')]
        ),

        # EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', 'filtered_odometry')],
        ),
        
        # SLAM Toolbox (Must be carefully namespaced or strictly isolated in configs)
        # Note: SLAM Toolbox often needs specific config tweaks for multi-robot map topics
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': slam_params_file
            }.items()
        ),

        # Nav2 Bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_file,
                'use_namespace': 'true',
                'namespace': namespace
            }.items()
        ),

        # Exploration Node
        Node(
            package='ausra_frontier_exploration',
            executable='exploration_server',
            name='exploration_server',
            output='screen',
            parameters=[
                nav2_params_file, 
                {
                    'use_sim_time': use_sim_time,
                    'robot_base_frame': PythonExpression(["'", namespace, "/base_link' if '", namespace, "' != '' else 'base_link'"])
                }
            ],
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'),
            
        exploration_group
    ])