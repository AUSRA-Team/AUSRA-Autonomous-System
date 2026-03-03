#!/usr/bin/env python3
"""
==============================================================================
AUSRA Robot Nav2 + EKF Test Launch (No Exploration)
==============================================================================

This launch file spawns a robot with Nav2 for testing on a known map.
Use this to test EKF and Nav2 functionality without frontier exploration.

Modes:
  - use_slam:=true  → Uses SLAM Toolbox (creates map while navigating)
  - use_slam:=false → Uses map_server + AMCL (localization on known map)

Usage:
    # Terminal 1: Launch world
    ros2 launch ausra_simulation world.launch.py world:=room_1.sdf
    
    # Terminal 2: Test with SLAM (no exploration - use rviz2 to send goals)
    ros2 launch ausra_spawner robot_nav2_test.launch.py robot_id:=1 x:=0.0 y:=0.0 use_slam:=true
    
    # Terminal 2: Test with known map (AMCL localization)
    ros2 launch ausra_spawner robot_nav2_test.launch.py robot_id:=1 x:=0.0 y:=0.0 use_slam:=false map:=/path/to/map.yaml

After launch, use rviz2 to send navigation goals:
    ros2 run rviz2 rviz2 -d /path/to/nav2_config.rviz
    
Or send goals via command line:
    ros2 action send_goal /ausra_1/navigate_to_pose nav2_msgs/action/NavigateToPose \
        "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"

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
    LogInfo,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml


def get_omni_driver_params(robot_name: str) -> dict:
    """Return omnidirectional driver parameters."""
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
    """Generate controller configuration file."""
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


def spawn_robot_nav2_test(context, *args, **kwargs):
    """Spawn robot with Nav2 for testing (no exploration)."""
    
    # Get launch configurations
    robot_id = LaunchConfiguration('robot_id').perform(context)
    robot_name = f"ausra_{robot_id}"
    
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    
    use_slam = LaunchConfiguration('use_slam').perform(context).lower() == 'true'
    map_file = LaunchConfiguration('map').perform(context)
    
    # Package directories
    pkg_ausrabot_description = get_package_share_directory('ausrabot_description')
    pkg_ausra_spawner = get_package_share_directory('ausra_spawner')
    
    # =========================================================================
    # ROBOT DESCRIPTION
    # =========================================================================
    xacro_file = os.path.join(pkg_ausrabot_description, 'urdf', 'robot.urdf.xacro')
    controller_config = generate_controller_config(robot_name)
    
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
            '--controller-manager-timeout', '60'
        ],
        output='screen'
    )
    
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
    
    omni_driver = Node(
        package='omnidirectional_driver',
        executable='omni_driver',
        name='omnidirectional_driver',
        namespace=robot_name,
        output='screen',
        parameters=[get_omni_driver_params(robot_name)],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )
    
    # =========================================================================
    # LOCALIZATION (SLAM or AMCL)
    # =========================================================================
    
    localization_nodes = []
    
    if use_slam:
        # Use SLAM Toolbox
        slam_config_base = os.path.join(pkg_ausra_spawner, 'config', 'slam_multirobot.yaml')
        slam_rewritten = RewrittenYaml(
            source_file=slam_config_base,
            param_rewrites={'<robot_namespace>': robot_name},
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
        localization_nodes.append(slam_node)
    else:
        # Use Map Server + AMCL for known map
        if not map_file or map_file == '':
            raise RuntimeError("map:=<path> is required when use_slam:=false")
        
        map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'yaml_filename': map_file,
                'topic_name': 'map',
                'frame_id': 'map',
            }]
        )
        
        nav2_config_base = os.path.join(pkg_ausra_spawner, 'config', 'nav2_multirobot.yaml')
        nav2_rewritten = RewrittenYaml(
            source_file=nav2_config_base,
            param_rewrites={'<robot_namespace>': robot_name},
            namespace=robot_name,
            convert_types=True
        )
        nav2_config = nav2_rewritten.perform(context)
        
        amcl_node = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace=robot_name,
            output='screen',
            parameters=[nav2_config, {
                'initial_pose.x': float(x),
                'initial_pose.y': float(y),
                'initial_pose.yaw': float(yaw),
            }],
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static'),
            ]
        )
        
        # Lifecycle manager for map_server and amcl
        localization_lifecycle = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
                'bond_timeout': 4.0,
            }]
        )
        
        localization_nodes.extend([map_server, amcl_node, localization_lifecycle])
    
    # =========================================================================
    # NAV2 NAVIGATION STACK
    # =========================================================================
    
    nav2_config_base = os.path.join(pkg_ausra_spawner, 'config', 'nav2_multirobot.yaml')
    nav2_rewritten = RewrittenYaml(
        source_file=nav2_config_base,
        param_rewrites={'<robot_namespace>': robot_name},
        namespace=robot_name,
        convert_types=True
    )
    nav2_config = nav2_rewritten.perform(context)
    
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
            'bond_timeout': 4.0,
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
    
    nav2_nodes = [
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        nav2_lifecycle_manager,
    ]
    
    # =========================================================================
    # LAUNCH SEQUENCE
    # =========================================================================
    
    all_optional_nodes = localization_nodes + nav2_nodes
    
    actions = [
        LogInfo(msg=f"=== NAV2 TEST MODE ==="),
        LogInfo(msg=f"Spawning robot: {robot_name} at ({x}, {y})"),
        LogInfo(msg=f"SLAM mode: {use_slam}"),
        LogInfo(msg=f"Use rviz2 to send navigation goals or:"),
        LogInfo(msg=f"  ros2 action send_goal /{robot_name}/navigate_to_pose nav2_msgs/action/NavigateToPose ..."),
        robot_state_publisher,
        TimerAction(period=2.0, actions=[spawn_entity]),
    ]
    
    # Chain: spawn -> JSB -> velocity controller -> omni driver
    actions.append(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[TimerAction(period=3.0, actions=[joint_state_broadcaster])]
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
                on_exit=[TimerAction(period=1.0, actions=[omni_driver])]
            )
        )
    )
    
    # Start localization + nav2 after controllers
    actions.append(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_velocity_controller,
                on_exit=[TimerAction(period=5.0, actions=all_optional_nodes)]
            )
        )
    )
    
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='1',
            description='Unique integer ID for the robot'),
        DeclareLaunchArgument('x', default_value='0.0',
            description='Initial X spawn position'),
        DeclareLaunchArgument('y', default_value='0.0',
            description='Initial Y spawn position'),
        DeclareLaunchArgument('yaw', default_value='0.0',
            description='Initial yaw orientation (radians)'),
        DeclareLaunchArgument('use_slam', default_value='true',
            description='Use SLAM (true) or AMCL with known map (false)'),
        DeclareLaunchArgument('map', default_value='',
            description='Path to map.yaml file (required when use_slam:=false)'),
        
        OpaqueFunction(function=spawn_robot_nav2_test)
    ])
