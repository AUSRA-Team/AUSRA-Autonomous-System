#!/usr/bin/env python3
"""
Simple Robot Test Launch File

This launch file spawns a single robot with controllers and omni_driver,
then optionally runs the holonomic movement demo.

NO SLAM, NO Nav2 - just the basic robot for testing transforms and movement.

Usage:
    ros2 launch ausra_spawner simple_robot_test.launch.py
    ros2 launch ausra_spawner simple_robot_test.launch.py run_demo:=true
    ros2 launch ausra_spawner simple_robot_test.launch.py robot_id:=2 x:=1.0 y:=2.0

Prerequisites:
    - Gazebo must be running with a world loaded:
      ros2 launch ausra_simulation room_1_world.launch.py
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


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


def spawn_robot_and_demo(context, *args, **kwargs):
    """Spawn robot with controllers, omni_driver, and optionally the demo."""
    robot_id = LaunchConfiguration('robot_id').perform(context)
    robot_name = f"ausra_{robot_id}"
    run_demo = LaunchConfiguration('run_demo').perform(context).lower() == 'true'
    
    pkg_ausrabot_description = get_package_share_directory('ausrabot_description')
    xacro_file = os.path.join(pkg_ausrabot_description, 'urdf', 'robot.urdf.xacro')
    
    # Generate controller config
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
            '--controller-manager-timeout', '30'
        ],
        output='screen'
    )
    
    # Joint Group Velocity Controller
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
    
    # Omnidirectional Driver with inline parameters
    omni_params = get_omni_driver_params(robot_name)
    
    omni_driver = Node(
        package='omnidirectional_driver',
        executable='omni_driver',
        name='omnidirectional_driver',
        namespace=robot_name,
        output='screen',
        parameters=[omni_params],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )
    
    # Holonomic Movement Demo (optional)
    holonomic_demo = Node(
        package='ausra_movement_demo',
        executable='holonomic_demo',
        name='holonomic_movement_demo',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_name': robot_name,
            'linear_velocity': 0.2,
            'angular_velocity': 0.3,
            'movement_distance': 1.0,
        }]
    )
    
    # Build the launch sequence
    actions = [
        robot_state_publisher,
        TimerAction(period=2.0, actions=[spawn_entity]),
    ]
    
    # After spawn completes, start controllers
    load_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                TimerAction(period=3.0, actions=[joint_state_broadcaster])
            ]
        )
    )
    actions.append(load_jsb_after_spawn)
    
    # After JSB, start velocity controller
    load_jgvc_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[joint_group_velocity_controller]
        )
    )
    actions.append(load_jgvc_after_jsb)
    
    # After velocity controller, start omni_driver
    load_driver_after_jgvc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_group_velocity_controller,
            on_exit=[
                TimerAction(period=1.0, actions=[omni_driver])
            ]
        )
    )
    actions.append(load_driver_after_jgvc)
    
    # If run_demo is true, start demo after omni_driver is up
    if run_demo:
        # Give omni_driver 3 seconds to fully initialize
        load_demo_after_driver = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_group_velocity_controller,
                on_exit=[
                    TimerAction(period=5.0, actions=[holonomic_demo])
                ]
            )
        )
        actions.append(load_demo_after_driver)
    
    return actions


def generate_launch_description():
    return LaunchDescription([
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
        DeclareLaunchArgument(
            'run_demo',
            default_value='false',
            description='Whether to run the holonomic movement demo after spawning'
        ),
        
        OpaqueFunction(function=spawn_robot_and_demo)
    ])
