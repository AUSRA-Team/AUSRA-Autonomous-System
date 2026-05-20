"""
map_merge_hw.launch.py
AUSRA Hardware Map Merge — Incremental Test Launch File

HARDWARE ADAPTATION NOTES (vs simulation version):
  - Input topic:  /map  (not /ausra_1/map)
    Hardware SLAM (async_slam_toolbox_node) runs in the GLOBAL namespace with
    NO robot prefix. It publishes to /map, not a namespaced topic.
  - Frame IDs:    map, ausrabot_odom, ausrabot_robot_footprint
    These are the real hardware TF frames from hardware_full_stack.launch.py.
  - No Gazebo:    spawn coordinates come from tape-measure SOP, not Gazebo args.
  - Phantom robot: A second expansion node with a non-existent input topic is
    always launched. Its subscriber never fires; it just heartbeats all-Unknown.
    This prevents multirobot_map_merge from segfaulting with only one real map
    (the composeGrids pipeline requires at least two valid arrays at init time).

PHASE SWITCHING:
  To switch between Phase 1 and Phase 2, edit only the ROBOT_HW_CONFIG block
  at the top of this file. All other code stays unchanged.

  Phase 1 — Robot at physical origin:
    'ausra_1': {'offset_x': 0.0, 'offset_y': 0.0}

  Phase 2 — Robot translated to (X=2.0, Y=0.0):
    'ausra_1': {'offset_x': 2.0, 'offset_y': 0.0}
"""

import os
from launch import LaunchDescription
from launch.actions import LogInfo, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# ==============================================================================
# ── EDIT THIS BLOCK ONLY ──────────────────────────────────────────────────────
#
# PHASE 1 — Robot placed exactly at the physical origin mark.
# Expected RViz result: merged map appears centred around the canvas origin.
#
#   'ausra_1': {'offset_x': 0.0, 'offset_y': 0.0}
#
# PHASE 2 — Robot physically moved to X=2.0m, Y=0.0m from origin.
# Expected RViz result: the map content shifts 2.0m in the +X direction.
# All walls and features appear 2.0m further right vs Phase 1.
# The canvas origin stays fixed at (-25.0, -25.0) — only content moves.
#
#   'ausra_1': {'offset_x': 2.0, 'offset_y': 0.0}
#
# For hardware deployment (SOP): fill values from tape measurements.
# ==============================================================================

ROBOT_HW_CONFIG = {
    'ausra_1': {
        'offset_x': 1.0,   # ← CHANGE THIS for Phase 2 / real deployment
        'offset_y': 0.15,   # ← CHANGE THIS for real deployment
    },
}

# ==============================================================================
# ── HARDWARE TOPIC MAPPING ────────────────────────────────────────────────────
# The hardware SLAM (async_slam_toolbox_node) runs in the global namespace.
# It publishes to /map — NOT to a namespaced /ausra_1/map topic.
# This dict maps each robot name to its real hardware SLAM topic.
# For a single-robot test, ausra_1 uses /map.
# For future multi-robot: each robot's SLAM must be remapped to its own topic
# (e.g., via namespace in hardware_full_stack.launch.py).
# ==============================================================================

ROBOT_SLAM_TOPICS = {
    'ausra_1': '/map',             # Real hardware SLAM output
    # Phantom robot: topic does not exist on hardware.
    # Subscriber never fires. Heartbeat publishes all-Unknown canvas.
    # This prevents composeGrids from crashing with only one real map.
    'ausra_2_phantom': '/map_phantom_never_published',
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

    # ── Startup log ────────────────────────────────────────────────────────────
    ld.add_action(LogInfo(msg=(
        '\n'
        '╔══════════════════════════════════════════════════════════════╗\n'
        '║         AUSRA Hardware Map Merge — Baseline Test             ║\n'
        '╠══════════════════════════════════════════════════════════════╣\n'
        '║ INPUT:  /map (hardware SLAM — global namespace)              ║\n'
        '║ OUTPUT: /map_merged                                          ║\n'
        '║ CANVAS: 1000×1000 @ 0.05 m/cell | Origin (-25.0, -25.0)     ║\n'
        '║ PHANTOM ROBOT: ausra_2_phantom provides dummy canvas         ║\n'
        '╚══════════════════════════════════════════════════════════════╝\n'
    )))

    # ── Print active robot offsets ─────────────────────────────────────────────
    for robot_name, cfg in ROBOT_HW_CONFIG.items():
        slam_topic = ROBOT_SLAM_TOPICS.get(robot_name, '/map')
        ld.add_action(LogInfo(msg=(
            f'[AUSRA HW] {robot_name}: '
            f'SLAM topic={slam_topic} | '
            f'offset=({cfg["offset_x"]:.3f}, {cfg["offset_y"]:.3f})'
        )))

    ld.add_action(LogInfo(msg=(
        '[AUSRA HW] Confirm robot is at tape-marked position with correct yaw.\n'
        '[AUSRA HW] init_pose_* in map_merge_HW_params.yaml must be 0.0.\n'
    )))

    # ── Map Expansion Nodes (one per real robot) ───────────────────────────────
    for robot_name, cfg in ROBOT_HW_CONFIG.items():
        slam_topic = ROBOT_SLAM_TOPICS.get(robot_name, '/map')
        output_topic = f'/{robot_name}/{MAP_FIXED_SUFFIX}'

        expansion_node = Node(
            package='ausra_map_merge_HW',
            executable='map_expansion_node',
            name=f'map_expansion_{robot_name}',
            namespace='',
            parameters=[{
                # ── Hardware-specific: input is global /map, not namespaced ──
                'input_topic':        slam_topic,
                'output_topic':       output_topic,

                # ── Spatial alignment: from tape measurement (SOP Phase 2) ──
                # Phase 1: both 0.0 (robot at physical origin)
                # Phase 2: set offset_x/offset_y to measured values
                'robot_offset_x':     cfg['offset_x'],
                'robot_offset_y':     cfg['offset_y'],

                # ── Canvas parameters (must match map_merge_HW_params.yaml) ─
                'canvas_width':       CANVAS_WIDTH,
                'canvas_height':      CANVAS_HEIGHT,
                'canvas_resolution':  CANVAS_RESOLUTION,
                'canvas_origin_x':    CANVAS_ORIGIN_X,
                'canvas_origin_y':    CANVAS_ORIGIN_Y,

                # ── Heartbeat rate ─────────────────────────────────────────
                'publish_rate_hz':    1.0,
            }],
            output='screen',
        )
        ld.add_action(expansion_node)

    # ── Phantom Expansion Node ─────────────────────────────────────────────────
    # This node subscribes to a topic that does not exist on hardware.
    # Its mapCallback never fires. Its heartbeat timer publishes a valid
    # 1000×1000 all-Unknown canvas immediately on startup.
    #
    # PURPOSE: multirobot_map_merge's composeGrids pipeline requires at least
    # two valid grids at initialisation time. Without this phantom node, the
    # merger discovers only one namespace (ausra_1) and crashes attempting an
    # affine transform against a null second matrix (Trigger B segfault).
    # With this node, the merger always has two valid arrays and initialises
    # safely. The phantom canvas is all-Unknown, so it does not pollute the
    # merged output — it only prevents the crash.
    #
    # Remove this node when a second physical robot is added to the fleet.
    phantom_expansion_node = Node(
        package='ausra_map_merge_HW',
        executable='map_expansion_node',
        name='map_expansion_ausra_2_phantom',
        namespace='',
        parameters=[{
            # This topic does not exist on hardware — subscriber never fires
            'input_topic':        ROBOT_SLAM_TOPICS['ausra_2_phantom'],
            'output_topic':       f'/ausra_2/{MAP_FIXED_SUFFIX}',
            'robot_offset_x':     0.0,
            'robot_offset_y':     0.0,
            'canvas_width':       CANVAS_WIDTH,
            'canvas_height':      CANVAS_HEIGHT,
            'canvas_resolution':  CANVAS_RESOLUTION,
            'canvas_origin_x':    CANVAS_ORIGIN_X,
            'canvas_origin_y':    CANVAS_ORIGIN_Y,
            'publish_rate_hz':    1.0,
        }],
        output='screen',
    )
    ld.add_action(phantom_expansion_node)

    # ── Central Map Merge Node ─────────────────────────────────────────────────
    # Small delay allows the heartbeat canvases to be published at least once
    # before the merger begins its discovery scan. At 1 Hz heartbeat, 2 seconds
    # guarantees at least one valid publish from both expansion nodes.
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
