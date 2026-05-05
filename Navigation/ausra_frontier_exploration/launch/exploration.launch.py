from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Use ausra_spawner config as single source of truth
    ausra_spawner_share = FindPackageShare('ausra_spawner')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file', 
        default=PathJoinSubstitution([ausra_spawner_share, 'config', 'nav2_params.yaml']))
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([ausra_spawner_share, 'config', 'nav2_params.yaml']),
            description='Path to param file'),

        # Exploration Action Server
        Node(
            package='ausra_frontier_exploration',
            executable='exploration_server',
            name='exploration_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),
        
        # Nav2 Bringup (if needed standalone)
        # Note: Usually called separately or included here
    ])
