# Multi-Robot Hardware Upgrade Guide

## Transitioning `ausra_map_merge_HW` from Single-Robot to Networked Multi-Robot

**Prerequisite:** Successful completion of the single-robot Phase 1 and Phase 2 baseline tests as documented in `Hardware_Baseline_Testing_Plan.md`.

---

## 1. What Changes and What Doesn't

Before making any edits, understand the scope of this upgrade:

| Component | Changes Required? | Explanation |
|---|---|---|
| `map_expansion_node.cpp` | **No** | The C++ node is fully parameterised. It reads `input_topic` and `output_topic` from its launch parameters. It does not care whether the input comes from `/map` or `/ausra_1/map`. No recompilation needed. |
| `map_merge_hw.launch.py` | **Yes** | The `ROBOT_HW_CONFIG` and `ROBOT_SLAM_TOPICS` dictionaries must be updated. The Phantom Node block must be removed. The banner log should be updated. |
| `map_merge_HW_params.yaml` | **Yes** | A new `init_pose` block must be added for each additional robot (all values `0.0`). |
| `CMakeLists.txt` | **No** | No new executables or install targets. |
| `package.xml` | **No** | No new dependencies. |

---

## 2. The Core Architectural Shift

### Single-Robot Architecture (Current)

In the single-robot setup, hardware SLAM runs in the **global namespace** and publishes to `/map`. A phantom node fakes the existence of a second robot to prevent the merger from segfaulting.

```
                              ┌─────────────────────┐
  slam_toolbox ──► /map ────► │ expansion (ausra_1) │ ──► /ausra_1/map_fixed ──┐
                              └─────────────────────┘                          │
                                                                               ├──► map_merge ──► /map_merged
                              ┌─────────────────────┐                          │
  (nothing) ──► /map_phantom  │ expansion (phantom)  │ ──► /ausra_2/map_fixed ──┘
                never_published└─────────────────────┘
                                     (all Unknown)
```

### Multi-Robot Architecture (Target)

Each robot's hardware stack is launched inside a **namespace**. SLAM publishes to `/<robot_name>/map`. Each robot gets a real expansion node. No phantom is needed because there are genuinely two or more grids for the merger to consume.

```
  Robot 1 SLAM ──► /ausra_1/map ──► expansion (ausra_1) ──► /ausra_1/map_fixed ──┐
                                                                                   ├──► map_merge ──► /map_merged
  Robot 2 SLAM ──► /ausra_2/map ──► expansion (ausra_2) ──► /ausra_2/map_fixed ──┘
```

### Prerequisite: Namespace Your Hardware Stacks

This upgrade guide assumes that each robot's `hardware_full_stack.launch.py` is now launched **inside a namespace**. This is the single upstream change that makes everything else work.

On **Robot 1's** onboard computer:
```bash
ros2 launch lidar_slam_pkg hardware_full_stack.launch.py \
    --ros-args -r __ns:=/ausra_1
```

On **Robot 2's** onboard computer:
```bash
ros2 launch lidar_slam_pkg hardware_full_stack.launch.py \
    --ros-args -r __ns:=/ausra_2
```

This causes each robot's `slam_toolbox` to publish to `/<namespace>/map` instead of the global `/map`.

> **⚠️ Important:** If your hardware stack does not support namespace injection via `--ros-args`, you will need to wrap it in a `GroupAction` with `PushRosNamespace` in a parent launch file. The end result must be that each robot's SLAM publishes to `/<robot_name>/map`.

---

## 3. Topic Routing — Eliminating the Hardcoded `/map`

### The Problem

The current `ROBOT_SLAM_TOPICS` dictionary hardcodes `/map` as the input for `ausra_1`:

```python
# CURRENT (single-robot)
ROBOT_SLAM_TOPICS = {
    'ausra_1': '/map',
    'ausra_2_phantom': '/map_phantom_never_published',
}
```

This worked when SLAM ran in the global namespace. With namespaced hardware stacks, SLAM now publishes to `/<robot_name>/map`.

### The Solution

Eliminate the `ROBOT_SLAM_TOPICS` dictionary entirely. Instead, derive the SLAM topic automatically from the robot name inside the launch loop:

```python
# NEW (multi-robot) — no ROBOT_SLAM_TOPICS dict needed
slam_topic = f'/{robot_name}/map'
```

This is both correct and scalable: adding a third robot to `ROBOT_HW_CONFIG` automatically generates the correct `input_topic` of `/ausra_3/map` without any additional mapping.

---

## 4. Phantom Node Removal

### Why It Existed

`multirobot_map_merge`'s `composeGrids` function segfaults when it discovers only one robot namespace and attempts an affine transform against a null second matrix. The phantom node published a valid all-Unknown canvas to `/ausra_2/map_fixed`, satisfying the merger's requirement for ≥ 2 grids at initialisation.

### Why It's No Longer Needed

With two (or more) real robots, each running an expansion node with a heartbeat timer, the merger discovers multiple namespaces and receives multiple valid grids immediately at startup. The phantom's job is done.

### How to Remove It

Delete the entire `phantom_expansion_node` block (lines 157–191 in the current launch file). Also remove the `'ausra_2_phantom'` entry from `ROBOT_SLAM_TOPICS` (which is itself being removed as described in Section 3).

> **⚠️ Re-introduce the Phantom if you ever drop back to a single-robot test.** The segfault protection is only unnecessary when ≥ 2 real expansion nodes are actively heartbeating. If Robot 2's expansion node is present in the launch file but Robot 2's SLAM never starts, the heartbeat still publishes all-Unknown — the merger won't crash. The phantom is only needed when no second expansion node exists at all.

---

## 5. Configuration Updates

### 5.1 — `ROBOT_HW_CONFIG` (in `map_merge_hw.launch.py`)

Populate one entry per physical robot with tape-measured offsets:

```python
ROBOT_HW_CONFIG = {
    'ausra_1': {
        'offset_x': 0.0,     # Robot 1 at physical origin
        'offset_y': 0.0,
    },
    'ausra_2': {
        'offset_x': 3.45,    # Measured: 3.45 m along +X from origin
        'offset_y': 0.0,     # Measured: 0.0 m along Y
    },
    # Add more robots as needed:
    # 'ausra_3': {
    #     'offset_x': 1.20,
    #     'offset_y': 2.80,
    # },
}
```

### 5.2 — `map_merge_HW_params.yaml`

Add a new `init_pose` block for each robot. **All values must remain `0.0`.**

```yaml
    # Robot 1
    /ausra_1/map_merge/init_pose_x:   0.0   # DO NOT CHANGE
    /ausra_1/map_merge/init_pose_y:   0.0   # DO NOT CHANGE
    /ausra_1/map_merge/init_pose_z:   0.0
    /ausra_1/map_merge/init_pose_yaw: 0.0

    # Robot 2
    /ausra_2/map_merge/init_pose_x:   0.0   # DO NOT CHANGE
    /ausra_2/map_merge/init_pose_y:   0.0   # DO NOT CHANGE
    /ausra_2/map_merge/init_pose_z:   0.0
    /ausra_2/map_merge/init_pose_yaw: 0.0

    # Robot 3 (add when deploying a third robot)
    # /ausra_3/map_merge/init_pose_x:   0.0
    # /ausra_3/map_merge/init_pose_y:   0.0
    # /ausra_3/map_merge/init_pose_z:   0.0
    # /ausra_3/map_merge/init_pose_yaw: 0.0
```

The `robot_namespace: ausra_` and `robot_map_topic: map_fixed` settings in the YAML **do not change**. The merger's namespace discovery pattern (`/ausra_N/map_fixed`) already matches the expansion node output topics.

---

## 6. C++ Verification — No Changes Required

The `map_expansion_node.cpp` is fully parameterised. Here is the relevant constructor code:

```cpp
this->declare_parameter<std::string>("input_topic",  "/map");
this->declare_parameter<std::string>("output_topic", "/ausra_1/map_fixed");
```

These are **defaults** that are overridden at launch time by the `parameters` block in the launch file. The node reads:

```cpp
input_topic_  = this->get_parameter("input_topic").as_string();
output_topic_ = this->get_parameter("output_topic").as_string();
```

When the launch file sets `input_topic` to `/ausra_2/map`, the node subscribes to that topic without any code changes. The spatial math, heartbeat timer, partial reset, fault tolerance, and QoS settings are all topic-agnostic.

**Verdict: No C++ changes. No recompilation needed.**

---

## 7. Complete Updated Launch File

The following is the complete, copy-pasteable replacement for `map_merge_hw.launch.py`:

```python
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
        'offset_x': 0.0,     # Robot 1 at physical origin
        'offset_y': 0.0,
    },
    'ausra_2': {
        'offset_x': 3.45,    # Measured: 3.45 m along +X from origin
        'offset_y': 0.0,     # Measured: 0.0 m along Y
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
```

---

## 8. Complete Updated YAML Configuration

The following is the complete replacement for `config/map_merge_HW_params.yaml` for a 2-robot deployment:

```yaml
# ==============================================================================
# map_merge_HW_params.yaml
# AUSRA Hardware Map Merge — Multi-Robot Merger Configuration
#
# CRITICAL — DO NOT CHANGE init_pose_* VALUES:
#   Every init_pose_* must remain 0.0 for ALL robots.
#   The map_expansion_node bakes the physical spawn offset into the canvas.
#   Setting init_pose_* to spawn coordinates doubles the shift.
#
#   WHERE spawn coordinates live → map_merge_hw.launch.py (ROBOT_HW_CONFIG)
#   WHERE init_pose_* lives      → here, always 0.0
# ==============================================================================

map_merge:
  ros__parameters:

    # ── Topic and namespace discovery ─────────────────────────────────────────
    robot_map_topic:   map_fixed
    robot_namespace:   ausra_

    # ── Output ────────────────────────────────────────────────────────────────
    merged_map_topic:  /map_merged

    # ── Frame ID ──────────────────────────────────────────────────────────────
    world_frame:       map

    # ── Merge mode ────────────────────────────────────────────────────────────
    known_init_poses:  true

    # ── Timing ────────────────────────────────────────────────────────────────
    merging_rate:      1.0
    discovery_rate:    0.05

    # ── Unused (known_init_poses=true bypasses feature matching) ──────────────
    estimation_rate:        0.5
    estimation_confidence:  1.0

    # ==========================================================================
    # !! INIT POSE VALUES — MUST ALL REMAIN 0.0 !!
    # ==========================================================================

    # Robot 1
    /ausra_1/map_merge/init_pose_x:   0.0   # DO NOT CHANGE
    /ausra_1/map_merge/init_pose_y:   0.0   # DO NOT CHANGE
    /ausra_1/map_merge/init_pose_z:   0.0
    /ausra_1/map_merge/init_pose_yaw: 0.0

    # Robot 2
    /ausra_2/map_merge/init_pose_x:   0.0   # DO NOT CHANGE
    /ausra_2/map_merge/init_pose_y:   0.0   # DO NOT CHANGE
    /ausra_2/map_merge/init_pose_z:   0.0
    /ausra_2/map_merge/init_pose_yaw: 0.0
```

---

## 9. Diff Summary — What Changed

### `map_merge_hw.launch.py`

```diff
-ROBOT_HW_CONFIG = {
-    'ausra_1': {
-        'offset_x': 1.0,
-        'offset_y': 0.15,
-    },
-}
+ROBOT_HW_CONFIG = {
+    'ausra_1': {
+        'offset_x': 0.0,
+        'offset_y': 0.0,
+    },
+    'ausra_2': {
+        'offset_x': 3.45,
+        'offset_y': 0.0,
+    },
+}

-ROBOT_SLAM_TOPICS = {
-    'ausra_1': '/map',
-    'ausra_2_phantom': '/map_phantom_never_published',
-}
+(removed — SLAM topic now derived as f'/{robot_name}/map')

-(entire phantom_expansion_node block removed — lines 157-191)

-        slam_topic = ROBOT_SLAM_TOPICS.get(robot_name, '/map')
+        slam_topic = f'/{robot_name}/map'
```

### `map_merge_HW_params.yaml`

```diff
-    # Robot 2 (phantom — all-Unknown canvas, prevents composeGrids segfault)
+    # Robot 2 (real robot at measured position)
     /ausra_2/map_merge/init_pose_x:   0.0
     /ausra_2/map_merge/init_pose_y:   0.0
     /ausra_2/map_merge/init_pose_z:   0.0
     /ausra_2/map_merge/init_pose_yaw: 0.0
```

### `map_expansion_node.cpp`

```diff
 (no changes)
```

---

## 10. Multi-Robot Topic Map (Post-Upgrade)

| # | Topic | Publisher | Subscriber | Notes |
|---|---|---|---|---|
| 1 | `/ausra_1/map` | Robot 1 `slam_toolbox` | `map_expansion_ausra_1` | Namespaced SLAM output |
| 2 | `/ausra_2/map` | Robot 2 `slam_toolbox` | `map_expansion_ausra_2` | Namespaced SLAM output |
| 3 | `/ausra_1/map_fixed` | `map_expansion_ausra_1` | `map_merge` | Fixed 1000×1000 canvas |
| 4 | `/ausra_2/map_fixed` | `map_expansion_ausra_2` | `map_merge` | Fixed 1000×1000 canvas |
| 5 | `/map_merged` | `map_merge` | RViz | Final merged output |

---

## 11. Validation Commands

Run these on the ground station to confirm the full pipeline:

```bash
# Confirm both SLAM stacks are publishing (run from ground station)
ros2 topic hz /ausra_1/map
ros2 topic hz /ausra_2/map

# Confirm both expansion nodes are outputting
ros2 topic hz /ausra_1/map_fixed
ros2 topic hz /ausra_2/map_fixed

# Confirm merged output
ros2 topic hz /map_merged

# Verify canvas metadata
ros2 topic echo /ausra_1/map_fixed --no-arr --once
ros2 topic echo /ausra_2/map_fixed --no-arr --once

# List all merge-related nodes
ros2 node list | grep -E 'expansion|merge'
# Expected:
#   /map_expansion_ausra_1
#   /map_expansion_ausra_2
#   /map_merge
```

---

## 12. Scaling Checklist

When adding Robot N to the fleet:

- [ ] Tape-measure Robot N's position from the physical origin (SOP Phase 2)
- [ ] Add `'ausra_N': {'offset_x': X, 'offset_y': Y}` to `ROBOT_HW_CONFIG`
- [ ] Add `init_pose` block (all `0.0`) for `/ausra_N/` in `map_merge_HW_params.yaml`
- [ ] Launch Robot N's hardware stack inside namespace `/ausra_N`
- [ ] Relaunch `map_merge_hw.launch.py`
- [ ] Verify `/ausra_N/map_fixed` publishes at 1 Hz
- [ ] Verify `/map_merged` contains Robot N's contribution
