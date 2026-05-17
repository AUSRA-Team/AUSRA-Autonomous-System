#!/usr/bin/env python3
"""
Full-stack robot spawner launch file for multi-robot AUSRA simulation.

This launch file extends spawn_ausra.launch.py with optional:
- EKF Localization (robot_localization)
- SLAM (slam_toolbox)
- Navigation (Nav2)
- Exploration (ausra_frontier_exploration)

Usage:
    # Robot only (same as spawn_ausra.launch.py)
    ros2 launch ausra_spawner spawn_ausra_full.launch.py robot_id:=1 x:=0 y:=0

    # Robot + full navigation stack
    ros2 launch ausra_spawner spawn_ausra_full.launch.py robot_id:=1 x:=0 y:=0 \
        use_ekf:=true use_slam:=true use_nav2:=true

    # Robot + navigation + exploration
    ros2 launch ausra_spawner spawn_ausra_full.launch.py robot_id:=1 x:=0 y:=0 \
        use_ekf:=true use_slam:=true use_nav2:=true use_exploration:=true
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
    OpaqueFunction,
    IncludeLaunchDescription,
    GroupAction,
    LogInfo,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml


def get_omni_driver_params(robot_name: str) -> dict:
    """Return omnidirectional driver parameters as a dictionary."""
    return {
        'wheel_names': [
            f'{robot_name}_joint_1',
            f'{robot_name}_joint_2', 
            f'{robot_name}_joint_3'
        ],
        'robot_radius': 0.124,
        'wheel_radius': 0.0325,
        'wheel_angles_deg': [270.0, 30.0, 150.0],
        'roller_angle_deg': 0.0,
        'use_field_centric': False,
        'odom_frame_id': f'{robot_name}_odom',
        'base_frame_id': f'{robot_name}_robot_footprint',
        'use_sim_time': True
    }


def generate_controller_config(robot_name: str) -> str:
    """Generate a temporary controller config file with correct joint names."""
    config_content = f"""# Auto-generated controller config for {robot_name}

controller_manager:
  ros__parameters:
    update_rate: 20
    use_sim_time: true
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    joint_group_velocity_controller:
      type: velocity_controllers/JointGroupVelocityController

{robot_name}:
  controller_manager:
    ros__parameters:
      update_rate: 20
      use_sim_time: true
      joint_state_broadcaster:
        type: joint_state_broadcaster/JointStateBroadcaster
      joint_group_velocity_controller:
        type: velocity_controllers/JointGroupVelocityController

joint_state_broadcaster:
  ros__parameters:
    use_sim_time: true

joint_group_velocity_controller:
  ros__parameters:
    use_sim_time: true
    joints:
      - {robot_name}_joint_1
      - {robot_name}_joint_2
      - {robot_name}_joint_3

{robot_name}:
  joint_state_broadcaster:
    ros__parameters:
      use_sim_time: true
  joint_group_velocity_controller:
    ros__parameters:
      use_sim_time: true
      joints:
        - {robot_name}_joint_1
        - {robot_name}_joint_2
        - {robot_name}_joint_3
"""
    
    config_dir = tempfile.mkdtemp(prefix='ausra_controller_')
    config_path = os.path.join(config_dir, f'{robot_name}_controller.yaml')
    with open(config_path, 'w') as f:
        f.write(config_content)
    
    return config_path


def generate_slam_config(robot_name: str, base_config_path: str) -> str:
    """Generate SLAM config with robot-specific frame names."""
    with open(base_config_path, 'r') as f:
        content = f.read()
    
    # Replace all <robot_namespace> placeholders with actual robot name
    content = content.replace('<robot_namespace>', robot_name)
    
    config_dir = tempfile.mkdtemp(prefix='ausra_slam_')
    config_path = os.path.join(config_dir, f'{robot_name}_slam.yaml')
    with open(config_path, 'w') as f:
        f.write(content)
    
    return config_path


def generate_ekf_config(robot_name: str, base_config_path: str) -> str:
    """Generate EKF config with robot-specific frame names."""
    with open(base_config_path, 'r') as f:
        content = f.read()
    
    # Replace all <robot_namespace> placeholders with actual robot name
    content = content.replace('<robot_namespace>', robot_name)
    
    config_dir = tempfile.mkdtemp(prefix='ausra_ekf_')
    config_path = os.path.join(config_dir, f'{robot_name}_ekf.yaml')
    with open(config_path, 'w') as f:
        f.write(content)
    
    return config_path


def generate_full_stack(context, *args, **kwargs):
    """Generate robot description and optional navigation stack."""
    robot_id = LaunchConfiguration('robot_id').perform(context)
    robot_name = f"ausra_{robot_id}"
    
    use_ekf = LaunchConfiguration('use_ekf').perform(context).lower() == 'true'
    use_slam = LaunchConfiguration('use_slam').perform(context).lower() == 'true'
    use_nav2 = LaunchConfiguration('use_nav2').perform(context).lower() == 'true'
    use_exploration = LaunchConfiguration('use_exploration').perform(context).lower() == 'true'
    
    # Get package directories
    pkg_ausrabot_description = get_package_share_directory('ausrabot_description')
    
    # Optional package directories (only get if needed)
    pkg_nav2_bringup = None
    pkg_exploration = None
    
    if use_nav2:
        pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    if use_exploration:
        pkg_exploration = get_package_share_directory('ausra_frontier_exploration')
    
    xacro_file = os.path.join(pkg_ausrabot_description, 'urdf', 'robot.urdf.xacro')
    controller_config = generate_controller_config(robot_name)
    
    # Process xacro
    import subprocess
    xacro_cmd = [
        'xacro', xacro_file,
        f'robot_name:={robot_name}',
        f'controller_config:={controller_config}'
    ]
    result = subprocess.run(xacro_cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"Xacro processing failed: {result.stderr}")
    
    robot_description_content = result.stdout
    
    # Get spawn position
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    
    # ============== BASE ROBOT NODES ==============
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
            'frame_prefix': ''
        }],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-entity', robot_name,
            '-topic', f'/{robot_name}/robot_description',
            '-x', x,
            '-y', y,
            '-z', '0.2',
            '-Y', yaw,
            '-robot_namespace', robot_name
        ],
        output='screen'
    )
    
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='jsb_spawner',
        namespace=robot_name,
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', f'/{robot_name}/controller_manager',
            '--controller-manager-timeout', '30'
        ],
        output='screen'
    )
    
    joint_group_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='jgvc_spawner',
        namespace=robot_name,
        arguments=[
            'joint_group_velocity_controller',
            '--controller-manager', f'/{robot_name}/controller_manager',
            '--controller-manager-timeout', '30'
        ],
        output='screen'
    )
    
    omni_params = get_omni_driver_params(robot_name)
    omni_driver = Node(
        package='omnidirectional_driver',
        executable='omni_driver',
        name='omnidirectional_driver',
        namespace=robot_name,
        output='screen',
        parameters=[omni_params],
        remappings=[
            ('/cmd_vel', 'cmd_vel'),
            ('/joint_states', 'joint_states'),
            ('/odom', 'odom'),
            ('/joint_group_velocity_controller/commands', 'joint_group_velocity_controller/commands'),
            ('/twist_with_covariance', 'twist_with_covariance'),
            ('cmd_vel', 'cmd_vel'),
            ('joint_states', 'joint_states'),
            ('odom', 'odom'),
            ('joint_group_velocity_controller/commands', 'joint_group_velocity_controller/commands'),
            ('twist_with_covariance', 'twist_with_covariance'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )
    
    # ============== OPTIONAL STACKS ==============
    # Separated into stages: early_nodes (EKF+SLAM) start first,
    # then nav2_nodes start after a delay so the 'map' TF frame exists.
    
    early_nodes = []   # EKF + SLAM — need to run first
    nav2_nodes = []    # Nav2 — needs 'map' frame from SLAM
    exploration_nodes = []  # Exploration — needs Nav2 fully active
    
    # EKF Localization
    if use_ekf:
        pkg_ausra_spawner = get_package_share_directory('ausra_spawner')
        ekf_base_config = os.path.join(pkg_ausra_spawner, 'config', 'ekf_multirobot.yaml')
        if os.path.exists(ekf_base_config):
            ekf_config = generate_ekf_config(robot_name, ekf_base_config)
        else:
            # Fallback to localization package config
            try:
                pkg_localization = get_package_share_directory('ausra_localization')
                ekf_config = os.path.join(pkg_localization, 'config', 'ekf.yaml')
            except Exception:
                ekf_config = None
        
        if ekf_config:
            ekf_node = Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                namespace=robot_name,
                output='screen',
                parameters=[ekf_config, {'use_sim_time': True}],
                remappings=[
                    ('odometry/filtered', 'filtered_odometry'),
                    ('/tf', '/tf'),
                    ('/tf_static', '/tf_static'),

                ]
            )
            early_nodes.append(ekf_node)
            
        # VERY IMPORTANT: Map isolation for proper multi-robot fleet merging
        # SLAM Toolbox forces base_link to (0,0,0) in its map frame.
        # By giving each robot its own map frame (e.g., ausra_1_map), and hooking it to the global 'map' 
        # using the physics spawn coordinates, we maintain perfect true-world coordinate alignment!
        map_offset_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_offset_publisher',
            namespace=robot_name,
            arguments=[
                x, y, '0.0', yaw, '0.0', '0.0',  # x y z yaw pitch roll
                'map', f'{robot_name}_map'
            ],
            output='screen'
        )
        early_nodes.append(map_offset_node)
    
    # SLAM Toolbox — Direct Node launch with RewrittenYaml
    if use_slam:
        pkg_ausra_spawner = get_package_share_directory('ausra_spawner')
        slam_config_base = os.path.join(pkg_ausra_spawner, 'config', 'slam_multirobot.yaml')
        
        slam_rewritten = RewrittenYaml(
            source_file=slam_config_base,
            param_rewrites={
                '<robot_namespace>': robot_name,
            },
            namespace=robot_name,
            convert_types=True
        )
        slam_config = slam_rewritten.perform(context)
        
        slam_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=robot_name,
            output='screen',
            parameters=[slam_config],
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static'),
            ]
        )
        early_nodes.append(slam_node)
    
    # Nav2 Navigation Stack
    if use_nav2:
        pkg_ausra_spawner = get_package_share_directory('ausra_spawner')
        nav2_config_base = os.path.join(pkg_ausra_spawner, 'config', 'nav2_multirobot.yaml')
        
        # Create RewrittenYaml with namespace support and execute it to get the config file path
        # This replaces <robot_namespace> placeholders and prefixes node keys with the namespace
        nav2_rewritten = RewrittenYaml(
            source_file=nav2_config_base,
            param_rewrites={
                '<robot_namespace>': robot_name,
            },
            namespace=robot_name,
            convert_types=True
        )
        nav2_config = nav2_rewritten.perform(context)
        
        # Nav2 Lifecycle Manager
        lifecycle_nodes = [
            'controller_server',
            'planner_server',
            'behavior_server',
            'bt_navigator',
            'waypoint_follower',
            'velocity_smoother',
        ]
        
        nav2_lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': lifecycle_nodes,
                'bond_timeout': 10.0,
                'attempt_respawn_reconnection': True,
                'bond_respawn_max_duration': 10.0,
            }]
        )
        
        # Controller Server
        controller_server = Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace=robot_name,
            output='screen',
            parameters=[nav2_config],
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static'),
            ]
        )
        
        # Planner Server
        planner_server = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=robot_name,
            output='screen',
            parameters=[nav2_config],
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static'),
            ]
        )
        
        # Behavior Server
        behavior_server = Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace=robot_name,
            output='screen',
            parameters=[nav2_config],
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static'),
            ]
        )
        
        # BT Navigator
        bt_navigator = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace=robot_name,
            output='screen',
            parameters=[nav2_config],
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static'),
            ]
        )
        
        # Waypoint Follower
        waypoint_follower = Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            namespace=robot_name,
            output='screen',
            parameters=[nav2_config],
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static'),
            ]
        )
        
        # Velocity Smoother
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
    
    # Frontier Exploration
    if use_exploration and pkg_exploration:
        # Load and process exploration config
        exploration_base_config = os.path.join(
            get_package_share_directory('ausra_spawner'),
            'config', 'exploration_multirobot.yaml'
        )
        
        exploration_params = {
            'use_sim_time': True,
            'robot_base_frame': f'{robot_name}_robot_footprint',
            'global_frame': f'{robot_name}_map',
            'map_topic': 'map',  # Subscribe to isolated SLAM map
            'start_x': float(x),
            'start_y': float(y),
            'start_yaw': float(yaw),
            'robot_radius': 0.15,
            'inflation_radius': 0.35,
            'min_frontier_size': 4,
            'safety_ratio': 0.98,
            'coverage_threshold': 0.95,
            'blacklist_timeout': 30.0,
            'exploration_loop_rate': 2.0,
            'return_to_start_on_complete': True,
            'start_position_tolerance': 0.3,
            'visualize_frontiers': True,
            'save_map_on_complete': True,
            'map_save_path': f'/tmp/{robot_name}_exploration_map',
        }
        
        exploration_node = Node(
            package='ausra_frontier_exploration',
            executable='exploration_server_enhanced',
            name='exploration_server',
            namespace=robot_name,
            output='screen',
            parameters=[exploration_params]
        )
        exploration_nodes.append(exploration_node)
    
    # ============== EVENT CHAIN ==============
    # Staged launch order:
    #   1. RSP → Spawn → JSB → JGVC → OmniDriver
    #   2. After JGVC (+5s):  EKF + SLAM start
    #   3. After JGVC (+10s): Nudge — small movement so SLAM populates the map
    #   4. After JGVC (+15s): Nav2 starts (needs 'map' TF from SLAM)
    #   5. After JGVC (+30s): Exploration starts (needs Nav2 fully active)
    
    delayed_spawn = TimerAction(
        period=2.0,
        actions=[spawn_entity]
    )
    
    load_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[joint_state_broadcaster]
                )
            ]
        )
    )
    
    load_jgvc_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[joint_group_velocity_controller]
        )
    )
    
    load_driver_after_jgvc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_group_velocity_controller,
            on_exit=[
                TimerAction(
                    period=1.0,
                    actions=[omni_driver]
                )
            ]
        )
    )
    
    actions = [
        robot_state_publisher,
        delayed_spawn,
        load_jsb_after_spawn,
        load_jgvc_after_jsb,
        load_driver_after_jgvc
    ]
    
    # Stage 1: Start EKF + SLAM early (5s after controllers)
    if early_nodes:
        load_early_after_driver = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_group_velocity_controller,
                on_exit=[
                    TimerAction(
                        period=5.0,  # Give driver time to stabilize
                        actions=early_nodes
                    )
                ]
            )
        )
        actions.append(load_early_after_driver)
    
    # Stage 2: Nudge the robot slightly so SLAM can register initial scans
    # SLAM needs the robot to move to populate the map and publish the 'map' TF
    if use_slam:
        nudge_cmd = ExecuteProcess(
            cmd=['bash', '-c',
                 f'ros2 topic pub --once /{robot_name}/cmd_vel geometry_msgs/msg/Twist '
                 f'"{{linear: {{x: 0.1, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: 0.3}}}}" '
                 f'&& sleep 2 '
                 f'&& ros2 topic pub --once /{robot_name}/cmd_vel geometry_msgs/msg/Twist '
                 f'"{{linear: {{x: 0.0, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: 0.0}}}}"'
            ],
            output='screen'
        )
        load_nudge = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_group_velocity_controller,
                on_exit=[
                    TimerAction(
                        period=35.0,  # 5s after exploration starts
                        actions=[
                            LogInfo(msg='>>> Nudging robot to seed SLAM map...'),
                            nudge_cmd,
                        ]
                    )
                ]
            )
        )
        actions.append(load_nudge)
    
    # Stage 2: Start Nav2 (15s after controllers)
    # This gives SLAM ~10s to receive scans and publish the 'map' TF frame
    if nav2_nodes:
        load_nav2_after_slam = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_group_velocity_controller,
                on_exit=[
                    TimerAction(
                        period=15.0,  # 10s after SLAM starts
                        actions=[
                            LogInfo(msg='>>> Starting Nav2 navigation stack...'),
                        ] + nav2_nodes
                    )
                ]
            )
        )
        actions.append(load_nav2_after_slam)
    
    # Stage 3: Start Exploration (30s after controllers)
    # This gives Nav2 ~15s to fully configure and activate all lifecycle nodes
    if exploration_nodes:
        load_exploration_after_nav2 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_group_velocity_controller,
                on_exit=[
                    TimerAction(
                        period=30.0,  # 15s after Nav2 starts
                        actions=[
                            LogInfo(msg='>>> Starting frontier exploration...'),
                        ] + exploration_nodes
                    )
                ]
            )
        )
        actions.append(load_exploration_after_nav2)
    
    return actions


def generate_launch_description():
    return LaunchDescription([
        # Position arguments
        DeclareLaunchArgument(
            'robot_id',
            default_value='1',
            description='Unique integer ID for the robot instance'
        ),
        DeclareLaunchArgument(
            'x',
            default_value='0.0',
            description='Initial X spawn position'
        ),
        DeclareLaunchArgument(
            'y',
            default_value='0.0',
            description='Initial Y spawn position'
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='Initial yaw orientation (radians)'
        ),
        
        # Optional stack arguments
        DeclareLaunchArgument(
            'use_ekf',
            default_value='false',
            description='Enable EKF localization'
        ),
        DeclareLaunchArgument(
            'use_slam',
            default_value='false',
            description='Enable SLAM Toolbox'
        ),
        DeclareLaunchArgument(
            'use_nav2',
            default_value='false',
            description='Enable Nav2 navigation stack'
        ),
        DeclareLaunchArgument(
            'use_exploration',
            default_value='false',
            description='Enable frontier exploration'
        ),
        
        # Use OpaqueFunction to handle dynamic configuration
        OpaqueFunction(function=generate_full_stack)
    ])
