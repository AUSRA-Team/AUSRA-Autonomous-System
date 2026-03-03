#!/usr/bin/env python3
"""
==============================================================================
AUSRA Multi-Robot Full Stack Spawner
==============================================================================

This launch file spawns a single AUSRA robot with the complete navigation stack:
- Robot model and controllers
- Omni-directional driver
- SLAM Toolbox (optional)
- Nav2 Navigation Stack (optional)
- Frontier Exploration (optional)

The system is designed for multi-robot operation with proper namespace isolation.

Usage:
    # Terminal 1: Launch world
    ros2 launch ausra_simulation world.launch.py world:=room_1.sdf
    
    # Terminal 2: Spawn robot 1 with full stack
    ros2 launch ausra_spawner robot_bringup.launch.py robot_id:=1 x:=0.0 y:=0.0
    
    # Terminal 3: Spawn robot 2 with full stack
    ros2 launch ausra_spawner robot_bringup.launch.py robot_id:=2 x:=2.0 y:=0.0

Arguments:
    robot_id: Unique integer ID for the robot (creates namespace ausra_<id>)
    x, y, yaw: Spawn position
    use_slam: Enable SLAM Toolbox (default: true)
    use_nav2: Enable Nav2 navigation (default: true)
    use_exploration: Enable frontier exploration (default: true)
    
==============================================================================
"""

import os
import tempfile
import subprocess

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
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

# Import RewrittenYaml for proper namespaced parameter handling
from nav2_common.launch import RewrittenYaml


def get_ekf_params(robot_name: str) -> dict:
    """Return EKF filter parameters for sensor fusion.
    
    EKF fuses wheel odometry (position/velocity) + IMU (yaw orientation).
    EKF publishes the odom->base TF with IMU-corrected orientation.
    """
    return {
        'use_sim_time': True,
        'frequency': 10.0,
        'sensor_timeout': 0.5,
        'two_d_mode': True,
        'transform_time_offset': 0.0,
        'transform_timeout': 0.5,
        'print_diagnostics': False,
        'debug': False,
        'publish_acceleration': False,
        'permit_corrected_publication': False,
        # CRITICAL: EKF publishes odom->base TF (omni_driver TF is disabled)
        'publish_tf': True,
        
        # Frame Configuration
        'map_frame': 'map',
        'odom_frame': f'{robot_name}_odom',
        'base_link_frame': f'{robot_name}_robot_footprint',
        'world_frame': f'{robot_name}_odom',
        
        # Odometry Input (from omni_driver) - position and velocity
        'odom0': 'odom',
        'odom0_config': [True, True, False,    # x, y, z position from wheels
                         False, False, False,  # roll, pitch, yaw (from IMU!)
                         True, True, False,    # vx, vy, vz velocities
                         False, False, False,  # vroll, vpitch, vyaw (from IMU!)
                         False, False, False], # ax, ay, az
        'odom0_queue_size': 10,
        'odom0_nodelay': False,
        'odom0_differential': False,
        'odom0_relative': False,
        'odom0_pose_rejection_threshold': 5.0,
        'odom0_twist_rejection_threshold': 2.0,
        
        # IMU Input (directly from Gazebo - topic is 'imu')
        # Use yaw orientation ONLY - this is the truth source
        'imu0': 'imu',
        'imu0_config': [False, False, False,   # x, y, z position
                        False, False, True,    # roll, pitch, YAW from IMU
                        False, False, False,   # vx, vy, vz
                        False, False, True,    # vroll, vpitch, VYAW from IMU
                        False, False, False],  # ax, ay, az (disabled)
        'imu0_nodelay': False,
        'imu0_differential': False,
        'imu0_relative': False,
        'imu0_queue_size': 10,
        'imu0_pose_rejection_threshold': 0.8,
        'imu0_twist_rejection_threshold': 0.8,
        'imu0_linear_acceleration_rejection_threshold': 0.8,
        'imu0_remove_gravitational_acceleration': True,
        
        # Control command integration - DISABLED for simplicity
        'use_control': False,
    }


def get_imu_filter_params() -> dict:
    """Return IMU complementary filter parameters."""
    return {
        'use_sim_time': True,
        'gain_acc': 0.01,
        'gain_mag': 0.01,
        'bias_alpha': 0.01,
        'do_bias_estimation': True,
        'do_adaptive_gain': True,
        'use_mag': False,
        'fixed_frame': 'odom',
        'publish_tf': False,
        'reverse_tf': False,
        'constant_dt': 0.0,
        'publish_debug_topics': False,
    }


def get_omni_driver_params(robot_name: str, use_ekf: bool = False) -> dict:
    """Return omnidirectional driver parameters.
    
    Args:
        robot_name: Robot namespace
        use_ekf: If True, disable TF publishing (EKF will publish TF instead)
                 NOTE: Currently disabled - omni_driver always publishes TF
                 because EKF fusion was causing instability
    """
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
        'publish_tf': True,  # Always publish TF - EKF disabled for stability
        'use_sim_time': True
    }


def generate_controller_config(robot_name: str) -> str:
    """Generate controller configuration file.
    
    gazebo_ros2_control loads this YAML file and creates the controller_manager
    inside gzserver. The namespace is /robot_name. We provide parameters at
    multiple paths to ensure compatibility.
    """
    # Include parameters at BOTH namespaced and non-namespaced paths
    # This ensures gazebo_ros2_control can find them regardless of how it loads
    config_content = f"""# Auto-generated controller config for {robot_name}

# Non-namespaced path (standard ros2_control expectation)
controller_manager:
  ros__parameters:
    update_rate: 20
    use_sim_time: true
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    joint_group_velocity_controller:
      type: velocity_controllers/JointGroupVelocityController

# Fully namespaced path (for namespaced gazebo_ros2_control)
{robot_name}:
  controller_manager:
    ros__parameters:
      update_rate: 20
      use_sim_time: true
      joint_state_broadcaster:
        type: joint_state_broadcaster/JointStateBroadcaster
      joint_group_velocity_controller:
        type: velocity_controllers/JointGroupVelocityController

# Controller-specific parameters (non-namespaced)
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

# Controller-specific parameters (namespaced)
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


def spawn_robot_with_stack(context, *args, **kwargs):
    """Main function to spawn robot with complete navigation stack."""
    
    # Get launch configurations
    robot_id = LaunchConfiguration('robot_id').perform(context)
    robot_name = f"ausra_{robot_id}"
    
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    
    use_ekf = LaunchConfiguration('use_ekf').perform(context).lower() == 'true'
    use_slam = LaunchConfiguration('use_slam').perform(context).lower() == 'true'
    use_nav2 = LaunchConfiguration('use_nav2').perform(context).lower() == 'true'
    use_exploration = LaunchConfiguration('use_exploration').perform(context).lower() == 'true'
    
    # Package directories
    pkg_ausrabot_description = get_package_share_directory('ausrabot_description')
    pkg_ausra_spawner = get_package_share_directory('ausra_spawner')
    
    # =========================================================================
    # ROBOT DESCRIPTION
    # =========================================================================
    xacro_file = os.path.join(pkg_ausrabot_description, 'urdf', 'robot.urdf.xacro')
    controller_config = generate_controller_config(robot_name)
    
    # Process xacro
    xacro_cmd = [
        'xacro', xacro_file,
        f'robot_name:={robot_name}',
        f'controller_config:={controller_config}'
    ]
    result = subprocess.run(xacro_cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"Xacro processing failed: {result.stderr}")
    robot_description_content = result.stdout
    
    # =========================================================================
    # BASE ROBOT NODES
    # =========================================================================
    
    # Robot State Publisher
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
    
    # Spawn Entity in Gazebo
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
    
    # Joint State Broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='jsb_spawner',
        namespace=robot_name,
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', f'/{robot_name}/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen'
    )
    
    # Joint Group Velocity Controller
    joint_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='jgvc_spawner',
        namespace=robot_name,
        arguments=[
            'joint_group_velocity_controller',
            '--controller-manager', f'/{robot_name}/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen'
    )
    
    # Omni-directional Driver
    # When use_ekf=True, omni_driver only publishes odom topic (not TF)
    # EKF will publish the corrected odom->base TF instead
    omni_driver = Node(
        package='omnidirectional_driver',
        executable='omni_driver',
        name='omnidirectional_driver',
        namespace=robot_name,
        output='screen',
        parameters=[get_omni_driver_params(robot_name, use_ekf)],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )
    
    # =========================================================================
    # OPTIONAL NAVIGATION STACK
    # =========================================================================
    
    optional_nodes = []
    
    # EKF Localization - DISABLED for stability
    # The EKF was causing sudden jumps due to IMU/odometry fusion conflicts
    # Omni_driver's wheel odometry is stable enough for SLAM exploration
    # If you need EKF, set use_ekf=true, but note it may cause instability
    if use_ekf:
        # EKF is currently disabled - omni_driver publishes TF directly
        # Uncomment below to enable EKF (may cause instability)
        # ekf_node = Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     namespace=robot_name,
        #     output='screen',
        #     parameters=[get_ekf_params(robot_name)],
        #     remappings=[
        #         ('odometry/filtered', 'filtered_odometry'),
        #         ('/tf', '/tf'),
        #         ('/tf_static', '/tf_static'),
        #     ]
        # )
        # optional_nodes.append(ekf_node)
        pass  # EKF disabled - using omni_driver TF directly
    
    # SLAM Toolbox
    if use_slam:
        slam_config_base = os.path.join(pkg_ausra_spawner, 'config', 'slam_multirobot.yaml')
        
        # Create RewrittenYaml with namespace support for SLAM
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
                # Map topic stays namespaced (default behavior)
            ]
        )
        optional_nodes.append(slam_node)
    
    # Nav2 Navigation Stack
    if use_nav2:
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
        
        optional_nodes.extend([
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            nav2_lifecycle_manager,
        ])
    
    # Frontier Exploration
    if use_exploration:
        exploration_config_base = os.path.join(pkg_ausra_spawner, 'config', 'exploration_multirobot.yaml')
        
        # Create RewrittenYaml with namespace support for exploration
        exploration_rewritten = RewrittenYaml(
            source_file=exploration_config_base,
            param_rewrites={
                '<robot_namespace>': robot_name,
            },
            namespace=robot_name,
            convert_types=True
        )
        exploration_config = exploration_rewritten.perform(context)
        
        # Store start position for return-to-home
        exploration_params = {
            'use_sim_time': True,
            'start_x': float(x),
            'start_y': float(y),
            'start_yaw': float(yaw),
        }
        
        exploration_node = Node(
            package='ausra_frontier_exploration',
            executable='exploration_server_enhanced',
            name='exploration_server',
            namespace=robot_name,
            output='screen',
            parameters=[exploration_config, exploration_params],
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static'),
            ]
        )
        optional_nodes.append(exploration_node)
    
    # =========================================================================
    # LAUNCH SEQUENCE WITH EVENT HANDLERS
    # =========================================================================
    
    actions = [
        LogInfo(msg=f"Spawning robot: {robot_name} at position ({x}, {y})"),
        robot_state_publisher,
        TimerAction(period=2.0, actions=[spawn_entity]),
    ]
    
    # Chain: spawn -> JSB -> velocity controller -> omni driver
    actions.append(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    TimerAction(period=3.0, actions=[joint_state_broadcaster])
                ]
            )
        )
    )
    
    actions.append(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[joint_velocity_controller]
            )
        )
    )
    
    actions.append(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_velocity_controller,
                on_exit=[
                    TimerAction(period=1.0, actions=[omni_driver])
                ]
            )
        )
    )
    
    # Start optional stacks after omni driver
    if optional_nodes:
        actions.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_velocity_controller,
                    on_exit=[
                        TimerAction(period=5.0, actions=optional_nodes)
                    ]
                )
            )
        )
    
    return actions


def generate_launch_description():
    return LaunchDescription([
        # Robot identity
        DeclareLaunchArgument(
            'robot_id',
            default_value='1',
            description='Unique integer ID for the robot (creates namespace ausra_<id>)'
        ),
        
        # Spawn position
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
        
        # Optional stacks
        DeclareLaunchArgument(
            'use_ekf',
            default_value='true',
            description='Enable EKF sensor fusion (robot_localization)'
        ),
        DeclareLaunchArgument(
            'use_slam',
            default_value='true',
            description='Enable SLAM Toolbox for mapping'
        ),
        DeclareLaunchArgument(
            'use_nav2',
            default_value='true',
            description='Enable Nav2 navigation stack'
        ),
        DeclareLaunchArgument(
            'use_exploration',
            default_value='true',
            description='Enable frontier exploration'
        ),
        
        OpaqueFunction(function=spawn_robot_with_stack)
    ])
