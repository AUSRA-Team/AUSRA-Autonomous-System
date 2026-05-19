import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, 
                            TimerAction, LogInfo, OpaqueFunction, ExecuteProcess)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_full_hardware_stack(context):
    # 1. Get package directories
    pkg_lidar_slam = get_package_share_directory('lidar_slam_pkg')
    pkg_localization = get_package_share_directory('ausra_localization')
    # pkg_exploration = get_package_share_directory('ausra_frontier_exploration')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_description = get_package_share_directory('ausrabot_description')
    pkg_lidar = get_package_share_directory('sllidar_ros2')
    
    # 2. Config paths
    # We use the holonomic params derived from simulation for better omni performance
    nav2_params_file = os.path.join(pkg_lidar_slam, 'config', 'nav2_holonomic_params.yaml')
    slam_config_file = os.path.join(pkg_lidar_slam, 'config', 'slam_toolbox_config.yaml')
    hardware_params = os.path.join(pkg_description, 'config', 'hardware_params.yaml')
    xacro_file = os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')
    explore_params_file = os.path.join(pkg_lidar_slam, 'config', 'explore_params.yaml')
    
    # 3. Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    nudge_robot = LaunchConfiguration('nudge_robot', default='false')

    # --- Stage 0: Core Hardware & Description ---
    
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )

    omni_driver = Node(
        package='omnidirectional_driver',
        executable='omni_driver',
        name='omnidirectional_driver', 
        output='screen',
        parameters=[hardware_params, {'use_sim_time': use_sim_time}]
    )

    lidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lidar, 'launch', 'sllidar_a1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': '115200',
            'frame_id': 'ausrabot_lidar'
        }.items()
    )

    # --- Stage 1: Localization (EKF) + SLAM (Starts after 5s) ---
    
    # EKF from ausra_localization
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, 'launch', 'localization.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # SLAM Toolbox in Mapping Mode
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_file, 
            {
                'use_sim_time': use_sim_time,
                'odom_frame': 'ausrabot_odom',
                'base_frame': 'ausrabot_robot_footprint',
                'scan_topic': '/scan'
            }
        ]
    )

    # --- Stage 2: Nav2 Navigation (Starts after 15s) ---
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true',
        }.items()
    )

    # --- Stage 3: Exploration (Starts after 30s) ---
    
    # exploration_server = Node(
    #     package='ausra_frontier_exploration',
    #     executable='exploration_server_enhanced', # Using the enhanced version from simulation
    #     name='exploration_server',
    #     output='screen',
    #     parameters=[
    #         {
    #             'use_sim_time': use_sim_time,
    #             'robot_base_frame': 'ausrabot_robot_footprint',
    #             'global_frame': 'map',
    #             'visualize_frontiers': True,
    #             # Additional params from simulation
    #             'robot_radius': 0.15,
    #             'inflation_radius': 0.4,
    #             'min_frontier_size': 4,  # Minimum number of cells to consider a frontier valid
    #         }
    #     ],
    # )
    exploration_server = Node(
        package='explore_lite',
        name='explore_node',
        executable='explore',
        parameters=[explore_params_file, {'use_sim_time': False}],
        output='screen',
    )

    # --- Assembly with Simulation-Style Sequencing ---
    
    return [
        LogInfo(msg='!!! AUSRA HARDWARE FULL STACK STARTING !!!'),
        
        # Stage 0: Core (Immediate)
        robot_state_publisher,
        omni_driver,
        lidar_driver,
        
        # Stage 1: Localization & SLAM (5s delay)
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg='>>> Stage 1: Starting EKF and SLAM...'),
                localization_launch,
                slam_toolbox
            ]
        ),
        
        # Stage 2: Nav2 (15s delay - gives SLAM time to publish 'map' TF)
        TimerAction(
            period=15.0,
            actions=[
                LogInfo(msg='>>> Stage 2: Starting Nav2 Navigation...'),
                nav2_launch
            ]
        ),
        
        # Stage 3: Exploration (30s delay - gives Nav2 time to activate)
        TimerAction(
            period=30.0,
            actions=[
                LogInfo(msg='>>> Stage 3: Starting Frontier Exploration...'),
                exploration_server
            ]
        ),
        
        # Stage 4: Nudge (Optional, Starts after 35s)
        TimerAction(
            period=35.0,
            actions=[
                LogInfo(msg='>>> Stage 4: Optional Nudging robot to seed SLAM...'),
                ExecuteProcess(
                    cmd=['ros2', 'topic', 'pub', '--once', '/cmd_vel', 'geometry_msgs/msg/Twist',
                         '"{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"'],
                    output='screen'
                ),
                TimerAction(
                    period=2.0,
                    actions=[
                        ExecuteProcess(
                            cmd=['ros2', 'topic', 'pub', '--once', '/cmd_vel', 'geometry_msgs/msg/Twist',
                                 '"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'],
                            output='screen'
                        )
                    ]
                )
            ]
        ) if nudge_robot.perform(context) == 'true' else LogInfo(msg='>>> Skipping Nudge...')
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'nudge_robot',
            default_value='false',
            description='Automatically nudge the robot to seed SLAM'),
            
        OpaqueFunction(function=generate_full_hardware_stack)
    ])
