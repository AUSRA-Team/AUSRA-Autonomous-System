import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, LogInfo
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    # 1. Get package directories
    pkg_exploration = get_package_share_directory('ausra_frontier_exploration')
    pkg_lidar_slam = get_package_share_directory('lidar_slam_pkg')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # 2. Config paths
    nav2_params_file = os.path.join(pkg_lidar_slam, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(pkg_lidar_slam, 'config', 'slam_toolbox_config.yaml')
    
    # 3. Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespace = LaunchConfiguration('namespace', default='')

    # 4. Robot Drivers + SLAM (Core System)
    # This starts motors, lidar, and slam_toolbox
    core_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_lidar_slam, 'launch', 'slam.launch.py'))
    )

    # 5. Nav2 Stack
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'use_composition': 'True',
        }.items()
    )

    # 6. Exploration Server
    exploration_server = Node(
        package='ausra_frontier_exploration',
        executable='exploration_server',
        name='exploration_server',
        output='screen',
        parameters=[
            nav2_params_file, 
            {
                'use_sim_time': use_sim_time,
                # Ensure frame matches your hardware setup (ausrabot_robot_footprint)
                'robot_base_frame': 'ausrabot_robot_footprint'
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('namespace', default_value=''),
        
        LogInfo(msg='Starting Integrated Autonomous Exploration (Mapping + Navigation)'),
        
        core_system_launch,
        nav2_launch,
        exploration_server
    ])
