import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, 
                            TimerAction, LogInfo, OpaqueFunction, ExecuteProcess,
                            GroupAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

def generate_nav2_config(robot_name: str, base_config_path: str) -> str:
    """Generate Nav2 config with robot-specific frame names, written to a temp file."""
    with open(base_config_path, 'r') as f:
        content = f.read()
    
    content = content.replace('<robot_namespace>', robot_name)
    
    config_dir = tempfile.mkdtemp(prefix='ausra_nav2_')
    config_path = os.path.join(config_dir, f'{robot_name}_nav2.yaml')
    with open(config_path, 'w') as f:
        f.write(content)
    
    return config_path

def generate_hardware_params(robot_name: str, base_config_path: str) -> str:
    """Rewrite <robot_namespace> placeholders in hardware_params.yaml."""
    with open(base_config_path, 'r') as f:
        content = f.read()

    content = content.replace('<robot_namespace>', robot_name)

    config_dir = tempfile.mkdtemp(prefix='ausra_hw_')
    config_path = os.path.join(config_dir, f'{robot_name}_hardware_params.yaml')
    with open(config_path, 'w') as f:
        f.write(content)

    return config_path

def generate_full_hardware_stack(context):
    # ── Resolve robot_name at context time ────────────────────────────────
    robot_name = LaunchConfiguration('robot_name').perform(context)

    # ── Dynamically generated TF frame names ──────────────────────────────
    base_frame = f'{robot_name}_robot_footprint'
    odom_frame = f'{robot_name}_odom'
    lidar_frame = f'{robot_name}_lidar'
    imu_frame  = f'{robot_name}_imu_link'
    map_frame  = f'{robot_name}_map'
    
    # ── Package directories ───────────────────────────────────────────────
    pkg_lidar_slam  = get_package_share_directory('lidar_slam_pkg')
    pkg_localization = get_package_share_directory('ausra_localization')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_description  = get_package_share_directory('ausrabot_description')
    pkg_lidar        = get_package_share_directory('sllidar_ros2')
    pkg_imu          = get_package_share_directory('mpu6050driver')

    # ── Config paths ──────────────────────────────────────────────────────
    nav2_params_file   = os.path.join(pkg_lidar_slam, 'config', 'nav2_holonomic_params.yaml')
    slam_config_file   = os.path.join(pkg_lidar_slam, 'config', 'slam_toolbox_config.yaml')
    xacro_file         = os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')
    explore_params_file = os.path.join(pkg_lidar_slam, 'config', 'explore_params.yaml')
    ekf_params_file    = os.path.join(pkg_localization, 'config', 'ekf.yaml')
    mpu6050_params_file = os.path.join(pkg_imu, 'params', 'mpu6050.yaml')

    # ── Launch Configurations ─────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    nudge_robot  = LaunchConfiguration('nudge_robot', default='false')

    # Get spawn position
    x = LaunchConfiguration('x', default='0.0').perform(context)
    y = LaunchConfiguration('y', default='0.0').perform(context)
    yaw = LaunchConfiguration('yaw', default='0.0').perform(context)

    # ══════════════════════════════════════════════════════════════════════
    # Stage 0: Core Hardware & Description
    # ══════════════════════════════════════════════════════════════════════

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' robot_name:=', robot_name]),
        value_type=str
    )

    send_ns = ExecuteProcess(
        cmd=['bash', '-c', f'stty -F /dev/ttyACM0 6000000 && echo "{robot_name}" > /dev/ttyACM0'],
        name='send_namespace',
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
            'ignore_timestamp': True,
        }],
    )

    hardware_params_base = os.path.join(pkg_description, 'config', 'hardware_params.yaml')
    hardware_params = generate_hardware_params(robot_name, hardware_params_base)
    omni_driver = Node(
        package='omnidirectional_driver',
        executable='omni_driver',
        name='omnidirectional_driver',
        output='screen',
        parameters=[hardware_params, {
            'use_sim_time': use_sim_time,
            'odom_frame_id': odom_frame,
            'base_frame_id': base_frame,
        }],
        remappings=[
            ('/odom', 'odom'),
            ('/cmd_vel', 'cmd_vel'),
            ('/joint_states', 'joint_states'),
            ('/joint_group_velocity_controller/commands', 'joint_group_velocity_controller/commands'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ],
    )

    lidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lidar, 'launch', 'sllidar_a1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': '115200',
            'frame_id': lidar_frame,
        }.items()
    )

    mpu6050_driver = Node(
        package='mpu6050driver',
        executable='mpu6050driver',
        name='mpu6050publisher',
        output='screen',
        emulate_tty=True,
        parameters=[mpu6050_params_file, {
            'frame_id': imu_frame, 
        }],
    )

    map_offset_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_offset_publisher',
        arguments=[
            x, y, '0.0', yaw, '0.0', '0.0',  
            'map', map_frame,
        ],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ],
        output='screen'
    )

    # ══════════════════════════════════════════════════════════════════════
    # Stage 1: Localization (IMU Filter + EKF) + SLAM
    # ══════════════════════════════════════════════════════════════════════

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {
            'use_sim_time': use_sim_time,
            'odom_frame': odom_frame,
            'base_link_frame': base_frame,
            'world_frame': odom_frame,
            'map_frame': map_frame,
            'odom0': 'odom',
            'imu0': 'imu',
        }],
        remappings=[
            ('odometry/filtered', 'filtered_odometry'),
        ],
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_file,
            {
                'use_sim_time': use_sim_time,
                'odom_frame': odom_frame,
                'base_frame': base_frame,
                'map_frame': map_frame,
                'scan_topic': 'scan',
            }
        ],
        remappings=[
            ('/scan', 'scan'),
            ('/map', 'map'),
        ],
    )

    # ══════════════════════════════════════════════════════════════════════
    # Stage 2: Nav2 Navigation
    # ══════════════════════════════════════════════════════════════════════

    nav2_nodes = []
    nav2_config_base = os.path.join(pkg_lidar_slam, 'config', 'nav2_holonomic_params.yaml')
    nav2_config = generate_nav2_config(robot_name, nav2_config_base)
    
    
    
    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': lifecycle_nodes,
            'bond_timeout': 10.0,
            'attempt_respawn_reconnection': True,
            'bond_respawn_max_duration': 10.0,
        }]
    )
    
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace=robot_name,
        output='screen',
        parameters=[nav2_config],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]
    )
    
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace=robot_name,
        output='screen',
        parameters=[nav2_config],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]
    )
    
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace=robot_name,
        output='screen',
        parameters=[nav2_config],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]
    )
    
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace=robot_name,
        output='screen',
        parameters=[nav2_config],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]
    )
    
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        namespace=robot_name,
        output='screen',
        parameters=[nav2_config],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]
    )
    
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        namespace=robot_name,
        output='screen',
        parameters=[nav2_config],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel'),
        ]
    )

    nav2_nodes.extend([
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        nav2_lifecycle_manager,
    ])

    # ══════════════════════════════════════════════════════════════════════
    # Stage 3: Exploration
    # ══════════════════════════════════════════════════════════════════════

    exploration_server = Node(
        package='explore_lite',
        name='explore_node',
        executable='explore',
        parameters=[explore_params_file, {
            'use_sim_time': use_sim_time,
            'robot_base_frame': base_frame,
            'costmap_topic': 'global_costmap/costmap',
            'costmap_updates_topic': 'global_costmap/costmap_updates',
        }],
        output='screen',
    )

    # ══════════════════════════════════════════════════════════════════════
    # Assemble — Wrap EVERYTHING in GroupAction + PushRosNamespace
    # ══════════════════════════════════════════════════════════════════════

    namespaced_actions = [
        PushRosNamespace(robot_name),

        LogInfo(msg=f'=== AUSRA HARDWARE FULL STACK [{robot_name}] STARTING ==='),

        # Stage 0: Core (Immediate)
        robot_state_publisher,
        omni_driver,
        lidar_driver,
        mpu6050_driver,
        map_offset_node,

        # Stage 1: IMU Filter + EKF + SLAM (5 s delay)
        TimerAction(
            period=5.0,
            actions=[
                GroupAction(actions=[
                    PushRosNamespace(robot_name),
                    LogInfo(msg=f'>>> [{robot_name}] Stage 1: Starting EKF and SLAM...'),
                    ekf_node,
                    slam_toolbox,
                ])
            ]
        ),
    ]

    # Stage 1.5: Nudge (10 s delay - 5s after SLAM starts)
    if nudge_robot.perform(context) == 'true':
        namespaced_actions.append(
            TimerAction(
                period=10.0,
                actions=[
                    LogInfo(msg=f'>>> [{robot_name}] Nudging robot to seed SLAM map...'),
                    ExecuteProcess(
                        cmd=['ros2', 'topic', 'pub', '-1',
                             f'/{robot_name}/cmd_vel', 'geometry_msgs/msg/Twist',
                             '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'],
                        output='screen'
                    ),
                    TimerAction(
                        period=2.0,
                        actions=[
                            ExecuteProcess(
                                cmd=['ros2', 'topic', 'pub', '-1',
                                     f'/{robot_name}/cmd_vel', 'geometry_msgs/msg/Twist',
                                     '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'],
                                output='screen'
                            )
                        ]
                    )
                ]
            )
        )
    else:
        namespaced_actions.append(LogInfo(msg='>>> Skipping Nudge...'))

    # # Stage 2: Nav2 (15 s delay - 5s after Nudge starts)
     namespaced_actions.append(
         TimerAction(
             period=25.0,
             actions=[
                 GroupAction(actions=[
                     PushRosNamespace(robot_name),
                     LogInfo(msg=f'>>> [{robot_name}] Stage 2: Starting Nav2 Navigation...'),
                 ])
             ] + nav2_nodes
         )
     )

    # # Stage 3: Exploration (22 s delay - 7s after Nav2 starts to allow costmaps to form)
     namespaced_actions.append(
         TimerAction(
             period=30.0,
             actions=[
                 GroupAction(actions=[
                     PushRosNamespace(robot_name),
                     LogInfo(msg=f'>>> [{robot_name}] Stage 3: Starting Frontier Exploration...'),
                     exploration_server,
                 ])
             ]
         )
     )

    return [GroupAction(actions=namespaced_actions)]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='ausrabot',
            description='Unique robot identifier — sets namespace AND TF frame prefix'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'nudge_robot',
            default_value='true',
            description='Automatically nudge the robot to seed SLAM'),
        DeclareLaunchArgument('x', default_value='0.0', description='X position'),
        DeclareLaunchArgument('y', default_value='0.0', description='Y position'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Yaw angle'),

        OpaqueFunction(function=generate_full_hardware_stack)
    ])