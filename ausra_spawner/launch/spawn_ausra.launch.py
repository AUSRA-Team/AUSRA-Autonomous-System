#!/usr/bin/env python3
"""
Dynamic robot spawner launch file for multi-robot AUSRA simulation.

This launch file:
1. Accepts robot_id, x, y, yaw as arguments
2. Generates unique robot_name from robot_id
3. Processes Xacro with robot_name parameter
4. Spawns robot_state_publisher in namespace
5. Spawns entity in Gazebo
6. Loads namespaced controllers
7. Starts omnidirectional_driver in namespace
"""

import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
    OpaqueFunction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def create_namespaced_config(template_path: str, robot_name: str) -> str:
    """Read a YAML config template and substitute placeholders with robot-specific values.
    
    Returns a temporary file path with the substituted content.
    The YAML is restructured to ensure parameters load correctly for the namespaced node.
    """
    with open(template_path, 'r') as f:
        content = f.read()
    
    # Substitute placeholders
    substitutions = {
        '<robot_namespace>': robot_name,
    }
    
    for placeholder, value in substitutions.items():
        content = content.replace(placeholder, value)
    
    # Write to temp file
    temp_file = tempfile.NamedTemporaryFile(
        mode='w', 
        suffix='.yaml', 
        prefix=f'{robot_name}_omni_', 
        delete=False
    )
    temp_file.write(content)
    temp_file.close()
    
    return temp_file.name


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


def generate_robot_description(context, *args, **kwargs):
    """Generate robot description with dynamic robot_name."""
    robot_id = LaunchConfiguration('robot_id').perform(context)
    robot_name = f"ausra_{robot_id}"
    
    pkg_ausrabot_description = get_package_share_directory('ausrabot_description')
    pkg_ausra_spawner = get_package_share_directory('ausra_spawner')
    xacro_file = os.path.join(pkg_ausrabot_description, 'urdf', 'robot.urdf.xacro')
    
    # Generate controller config with correct joint names
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
    
    # Robot State Publisher (namespaced)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
            'frame_prefix': ''  # Frame names already include robot_name prefix
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
    
    # Controllers (namespaced via controller_manager)
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
    
    # Get Omnidirectional Driver parameters as dictionary (guaranteed to work)
    omni_params = get_omni_driver_params(robot_name)
    
    omni_driver = Node(
        package='omnidirectional_driver',
        executable='omni_driver',
        name='omnidirectional_driver',
        namespace=robot_name,
        output='screen',
        parameters=[omni_params],
        remappings=[
            # Remap absolute paths (in case binary uses /topic)
            ('/cmd_vel', 'cmd_vel'),
            ('/joint_states', 'joint_states'),
            ('/odom', 'odom'),
            ('/joint_group_velocity_controller/commands', 'joint_group_velocity_controller/commands'),
            ('/twist_with_covariance', 'twist_with_covariance'),
            # Remap relative paths (redundant but safe)
            ('cmd_vel', 'cmd_vel'),
            ('joint_states', 'joint_states'),
            ('odom', 'odom'),
            ('joint_group_velocity_controller/commands', 'joint_group_velocity_controller/commands'),
            ('twist_with_covariance', 'twist_with_covariance'),
            # TF remappings
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )
    
    # Chain the startup sequence with delays
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
    
    return [
        robot_state_publisher,
        delayed_spawn,
        load_jsb_after_spawn,
        load_jgvc_after_jsb,
        load_driver_after_jgvc
    ]


def generate_controller_config(robot_name: str) -> str:
    """Generate a temporary controller config file with correct joint names.
    
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
    
    # Write to temp file
    config_dir = tempfile.mkdtemp(prefix='ausra_controller_')
    config_path = os.path.join(config_dir, f'{robot_name}_controller.yaml')
    with open(config_path, 'w') as f:
        f.write(config_content)
    
    return config_path


def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
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
        
        # Use OpaqueFunction to handle dynamic configuration
        OpaqueFunction(function=generate_robot_description)
    ])
