# ============================================================================
# map_merge.launch.py
# Launches multirobot_map_merge for the AUSRA swarm.
#
# Architecture:
#   1. One map_expansion_node per robot:
#      Subscribes to /ausra_X/map (SLAM Toolbox, dynamic size) and
#      republishes as /ausra_X/map_fixed (fixed canvas, globally pre-aligned).
#
#      Each expansion node receives the robot's spawn offset (robot_offset_x/y)
#      and shifts the SLAM data from the robot's local frame into global pixel
#      positions within the canvas. This means all canvases share the same
#      global coordinate space.
#
#   2. One central map_merge node:
#      Because all canvases are pre-aligned globally, init_pose is set to 0.0
#      for all robots. The node acts as a pure pixel overlay — no shifting,
#      no resizing. Result grid is always 1000x1000 with origin at (-25, -25).
# ============================================================================

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# ── EDIT THIS SECTION ──────────────────────────────────────────────────────
# Spawn offsets MUST match spawn_ausra_full.launch.py x, y arguments
# AND the static TF published by map_offset_node.
ROBOT_SPAWN_POSES = {
    'ausra_1': {'x': 3.0, 'y': 0.0},
    'ausra_2': {'x': 0.0, 'y': 2.0},
    # Add more robots here:
    # 'ausra_3': {'x': 4.0, 'y': 0.0},
}
# ── END EDIT SECTION ──────────────────────────────────────────────────────


def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = get_package_share_directory('ausra_map_merge')
    map_merge_params = os.path.join(pkg_share, 'config', 'map_merge_params.yaml')

    # ── Map Expansion Relay Nodes (one per robot) ──────────────────────────
    # Each node receives the robot's spawn offset to transform SLAM data
    # from the robot's local frame into globally-aligned pixel positions.

    for robot_name, spawn in ROBOT_SPAWN_POSES.items():
        expansion_node = Node(
            package='ausra_map_merge',
            executable='map_expansion_node',
            name=f'map_expansion_{robot_name}',
            namespace='',
            parameters=[{
                'input_topic':  f'/{robot_name}/map',
                'output_topic': f'/{robot_name}/map_fixed',
                # Canvas: 1000x1000 cells at 0.05 m/cell = 50m x 50m
                'canvas_width':      1000,
                'canvas_height':     1000,
                'canvas_resolution': 0.05,
                'canvas_origin_x':   -25.0,
                'canvas_origin_y':   -25.0,
                # Robot's Gazebo spawn position (meters)
                # Shifts SLAM data from local → global pixel positions
                'robot_offset_x':    spawn['x'],
                'robot_offset_y':    spawn['y'],
            }],
            output='screen',
        )
        ld.add_action(expansion_node)

    # ── Single Central Map Merge Node ──────────────────────────────────────
    # All canvases are globally pre-aligned by the expansion nodes.
    # init_pose = 0 for all robots → map_merge is a pure pixel overlay.
    map_merge_node = Node(
        package='multirobot_map_merge',
        executable='map_merge',
        name='map_merge',
        namespace='',
        parameters=[
            map_merge_params,
        ],
        output='screen',
    )
    ld.add_action(map_merge_node)

    return ld
