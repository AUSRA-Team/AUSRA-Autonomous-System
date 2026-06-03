"""
map_merge_hw.launch.py
AUSRA Hardware Map Merge — Multi-Robot Networked Deployment

MULTI-ROBOT NOTES:
  - Each robot's hardware_full_stack.launch.py must be launched inside a
    namespace (e.g., /ausra_1, /ausra_2) so that SLAM publishes to
    /<robot_name>/map instead of the global /map.
  - The map expansion node takes the tape-measured physical spawn offsets
    to expand and shift the map before merging.
  - The SLAM input topic is derived automatically:  /<robot_name>/map
  - The expansion output topic follows the pattern: /<robot_name>/map_fixed

SCALING VIA COMMAND LINE:
  You can now pass your robots and their X/Y offsets directly from the terminal!
  Format: "robot_name:offset_x:offset_y robot_name:offset_x:offset_y"

  Example:
  ros2 launch ausra_map_merge_HW map_merge_hw.launch.py robot_config:="alpha:1.0:0.0 beta:3.45:0.15"
"""

import os
from launch import LaunchDescription
from launch.actions import LogInfo, TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Canvas parameters — must match map_merge_HW_params.yaml
CANVAS_WIDTH      = 1000
CANVAS_HEIGHT     = 1000
CANVAS_RESOLUTION = 0.05
CANVAS_ORIGIN_X   = -25.0
CANVAS_ORIGIN_Y   = -25.0

# Output topic suffix that multirobot_map_merge will subscribe to
MAP_FIXED_SUFFIX = 'map_fixed'

def launch_setup(context, *args, **kwargs):
    actions = []
    pkg_share = get_package_share_directory('ausra_map_merge_HW')
    params_file = os.path.join(pkg_share, 'config', 'map_merge_HW_params.yaml')

    # ── Parse Robot Config String ─────────────────────────────────────────────
    robot_config_str = LaunchConfiguration('robot_config').perform(context)
    robot_hw_config = {}
    
    if robot_config_str.strip():
        # Example format: "ausra_1:1.0:0.0 ausra_2:1.0:0.15"
        robot_entries = robot_config_str.split()
        for entry in robot_entries:
            try:
                name, x_str, y_str = entry.split(':')
                robot_hw_config[name] = {
                    'offset_x': float(x_str),
                    'offset_y': float(y_str)
                }
            except ValueError:
                actions.append(LogInfo(msg=f'[ERROR] Invalid robot config format: {entry}. Expected name:x:y'))

    robot_count = len(robot_hw_config)

    # ── Startup log ────────────────────────────────────────────────────────────
    actions.append(LogInfo(msg=(
        '\n'
        '╔══════════════════════════════════════════════════════════════╗\n'
        '║      AUSRA Hardware Map Merge — Multi-Robot Deployment       ║\n'
        '╠══════════════════════════════════════════════════════════════╣\n'
       f'║ ROBOTS: {robot_count} configured                                        ║\n'
        '║ INPUT:  /<robot_name>/map (namespaced SLAM)                  ║\n'
        '║ OUTPUT: /map_merged                                          ║\n'
        '║ CANVAS: 1000×1000 @ 0.05 m/cell | Origin (-25.0, -25.0)     ║\n'
        '╚══════════════════════════════════════════════════════════════╝\n'
    )))

    # ── Print active robot offsets ─────────────────────────────────────────────
    for robot_name, cfg in robot_hw_config.items():
        slam_topic = f'/{robot_name}/map'
        actions.append(LogInfo(msg=(
            f'[AUSRA HW] {robot_name}: '
            f'SLAM topic={slam_topic} | '
            f'offset=({cfg["offset_x"]:.3f}, {cfg["offset_y"]:.3f})'
        )))

    actions.append(LogInfo(msg=(
        '[AUSRA HW] Confirm all robots are at tape-marked positions with correct yaw.\n'
        '[AUSRA HW] init_pose_* dynamically set to 0.0 for all robots.\n'
    )))

    # ── Map Expansion Nodes (one per real robot) ───────────────────────────────
    for robot_name, cfg in robot_hw_config.items():
        slam_topic   = f'/{robot_name}/map'
        output_topic = f'/{robot_name}/{MAP_FIXED_SUFFIX}'

        expansion_node = Node(
            package='ausra_map_merge_HW',
            executable='map_expansion_node',
            name=f'map_expansion_{robot_name}',
            namespace='',
            parameters=[{
                'input_topic':        slam_topic,
                'output_topic':       output_topic,
                'robot_offset_x':     cfg['offset_x'],
                'robot_offset_y':     cfg['offset_y'],
                'canvas_width':       CANVAS_WIDTH,
                'canvas_height':      CANVAS_HEIGHT,
                'canvas_resolution':  CANVAS_RESOLUTION,
                'canvas_origin_x':    CANVAS_ORIGIN_X,
                'canvas_origin_y':    CANVAS_ORIGIN_Y,
                'publish_rate_hz':    1.0,
            }],
            output='screen',
        )
        actions.append(expansion_node)

    # ── Central Map Merge Node ─────────────────────────────────────────────────
    # Dynamically build init_pose parameters for all parsed robots
    dynamic_init_poses = {}
    for robot_name in robot_hw_config.keys():
        dynamic_init_poses[f'/{robot_name}/map_merge/init_pose_x'] = 0.0
        dynamic_init_poses[f'/{robot_name}/map_merge/init_pose_y'] = 0.0
        dynamic_init_poses[f'/{robot_name}/map_merge/init_pose_z'] = 0.0
        dynamic_init_poses[f'/{robot_name}/map_merge/init_pose_yaw'] = 0.0

    # 2-second delay allows heartbeat canvases to publish at least once before
    # the merger begins its discovery scan.
    map_merge_node = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg='[AUSRA HW] Starting multirobot_map_merge node...'),
            Node(
                package='multirobot_map_merge',
                executable='map_merge',
                name='map_merge',
                namespace='',
                parameters=[params_file, dynamic_init_poses],
                output='screen',
            ),
        ]
    )
    actions.append(map_merge_node)

    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config',
            default_value='ausra_1:1.0:0.0 ausra_2:1.0:0.15',
            description='Space-separated list of robot configs in the format name:offset_x:offset_y'
        ),
        OpaqueFunction(function=launch_setup)
    ])
