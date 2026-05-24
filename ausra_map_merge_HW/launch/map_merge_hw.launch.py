"""
map_merge_hw.launch.py
AUSRA Hardware Map Merge — Multi-Robot Networked Deployment

MULTI-ROBOT NOTES:
  - Each robot's hardware_full_stack.launch.py must be launched inside a
    namespace (e.g., /ausra_1, /ausra_2) so that SLAM publishes to
    /<robot_name>/map instead of the global /map.
  - The ROBOT_HW_CONFIG dictionary below maps each robot to its
    tape-measured physical spawn offset.
  - The SLAM input topic is derived automatically:  /<robot_name>/map
  - The expansion output topic follows the pattern: /<robot_name>/map_fixed
  - No phantom node is needed when 2+ real robots are present.

SCALING:
  To add a new robot:
    1. Add an entry to ROBOT_HW_CONFIG below.
    2. Add a matching init_pose block (all 0.0) in map_merge_HW_params.yaml.
    3. Launch the new robot's hardware stack inside its namespace.
    4. Relaunch this file.
"""

import os
from launch import LaunchDescription
from launch.actions import LogInfo, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# ==============================================================================
# ── ROBOT FLEET CONFIGURATION ─────────────────────────────────────────────────
#
# One entry per physical robot. Offsets come from the tape-measure SOP.
# The SLAM input topic is derived automatically as /<robot_name>/map.
#
# IMPORTANT: Each robot's hardware_full_stack.launch.py must be launched
# inside the corresponding namespace (e.g., --ros-args -r __ns:=/ausra_1).
# ==============================================================================

ROBOT_HW_CONFIG = {
    'ausra_1': {
        'offset_x': 1.0,     # Robot 1 at physical origin
        'offset_y': 0.0,
    },
    'ausra_2': {
        'offset_x': 1.0,    # Measured: 3.45 m along +X from origin
        'offset_y': 0.15,     # Measured: 0.0 m along Y
    },
    # ── Add more robots here ──────────────────────────────────────────────
    # 'ausra_3': {
    #     'offset_x': 1.20,
    #     'offset_y': 2.80,
    # },
}

# Canvas parameters — must match map_merge_HW_params.yaml
CANVAS_WIDTH      = 1000
CANVAS_HEIGHT     = 1000
CANVAS_RESOLUTION = 0.05
CANVAS_ORIGIN_X   = -25.0
CANVAS_ORIGIN_Y   = -25.0

# Output topic suffix that multirobot_map_merge will subscribe to
MAP_FIXED_SUFFIX = 'map_fixed'


def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = get_package_share_directory('ausra_map_merge_HW')
    params_file = os.path.join(pkg_share, 'config', 'map_merge_HW_params.yaml')

    robot_count = len(ROBOT_HW_CONFIG)

    # ── Startup log ────────────────────────────────────────────────────────────
    ld.add_action(LogInfo(msg=(
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
    for robot_name, cfg in ROBOT_HW_CONFIG.items():
        slam_topic = f'/{robot_name}/map'
        ld.add_action(LogInfo(msg=(
            f'[AUSRA HW] {robot_name}: '
            f'SLAM topic={slam_topic} | '
            f'offset=({cfg["offset_x"]:.3f}, {cfg["offset_y"]:.3f})'
        )))

    ld.add_action(LogInfo(msg=(
        '[AUSRA HW] Confirm all robots are at tape-marked positions with correct yaw.\n'
        '[AUSRA HW] init_pose_* in map_merge_HW_params.yaml must be 0.0 for ALL robots.\n'
    )))

    # ── Map Expansion Nodes (one per real robot) ───────────────────────────────
    for robot_name, cfg in ROBOT_HW_CONFIG.items():
        # SLAM topic derived automatically from robot name
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
        ld.add_action(expansion_node)

    # ── Central Map Merge Node ─────────────────────────────────────────────────
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
                parameters=[params_file],
                output='screen',
            ),
        ]
    )
    ld.add_action(map_merge_node)

    return ld
