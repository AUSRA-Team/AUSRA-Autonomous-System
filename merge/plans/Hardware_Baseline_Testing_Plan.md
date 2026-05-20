# Hardware Baseline Testing Plan
## `ausra_map_merge_HW` — Single Robot Incremental Verification

**Package:** `ausra_map_merge_HW`
**Target hardware:** Physical AUSRA robot (RPLidar A1, omnidirectional base)
**SLAM stack:** `hardware_full_stack.launch.py` → `async_slam_toolbox_node`
**Document purpose:** Step-by-step verification of the map expansion pipeline
on real hardware, starting with the simplest possible case and progressively
confirming each layer of the spatial math.

---

## Table of Contents

1. [Hardware vs Simulation — Key Differences](#1-hardware-vs-simulation--key-differences)
2. [Package Structure](#2-package-structure)
3. [Build Instructions](#3-build-instructions)
4. [The Phantom Robot — Why It Exists](#4-the-phantom-robot--why-it-exists)
5. [Phase 1 — Single Robot at Physical Origin](#5-phase-1--single-robot-at-physical-origin)
6. [Phase 2 — Single Robot at Measured Offset](#6-phase-2--single-robot-at-measured-offset)
7. [⚠️ Critical Warning — init_pose Must Be Zero](#7-️-critical-warning--init_pose-must-be-zero)
8. [RViz Setup Reference](#8-rviz-setup-reference)
9. [Troubleshooting](#9-troubleshooting)

---

## 1. Hardware vs Simulation — Key Differences

Before running anything, understand why the hardware package differs from the
simulation package. Failure to understand this will produce confusing results.

| Aspect | Simulation (`ausra_map_merge`) | Hardware (`ausra_map_merge_HW`) |
|---|---|---|
| SLAM map topic | `/ausra_1/map` | `/map` |
| Robot namespace | `ausra_1/`, `ausra_2/` applied at spawn | **None** — global namespace |
| TF global frame | `map` (Gazebo world) | `map` (slam_toolbox) |
| TF odom frame | `ausra_1_odom` | `ausrabot_odom` |
| TF base frame | `ausra_1_base_link` | `ausrabot_robot_footprint` |
| Spawn coordinates | Known from Gazebo args | Tape-measured from physical origin |
| Second robot during tests | Real robot in Gazebo | **Phantom expansion node** (no robot) |

### The Critical Topic Difference

The hardware SLAM stack (`async_slam_toolbox_node`) is launched with **no
namespace** in `hardware_full_stack.launch.py`:

```
"There are no explicit namespaces applied at the top level of this launch
file, meaning nodes operate in the global namespace by default."
```

This means SLAM publishes to `/map`, not `/ausra_1/map`. The expansion node
`input_topic` parameter is therefore set to `/map` for hardware. This is the
single most important adaptation from simulation to hardware.

---

## 2. Package Structure

```
ausra_map_merge_HW/
├── CMakeLists.txt
├── package.xml
├── src/
│   └── map_expansion_node.cpp        ← heartbeat timer architecture (validated)
├── launch/
│   └── map_merge_hw.launch.py        ← hardware launch (edit ROBOT_HW_CONFIG only)
├── config/
│   └── map_merge_HW_params.yaml      ← merger config (init_pose_* stays 0.0)
└── docs/
    ├── AUSRA_Hardware_Map_Merge_SOP.md
    └── Alternative_Hardware_Strategies.md
```

---

## 3. Build Instructions

Run these commands on the machine that will be connecting to the robot
(onboard computer or ground station with ROS 2 Humble installed).

### Step 3.1 — Copy the Package into Your Workspace

```bash
# Assuming your ROS 2 workspace is ~/ros2_ws
cp -r ausra_map_merge_HW ~/ros2_ws/src/
```

### Step 3.2 — Install Dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Step 3.3 — Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select ausra_map_merge_HW --symlink-install
```

Expected output (no errors):

```
Starting >>> ausra_map_merge_HW
Finished <<< ausra_map_merge_HW [~5s]

Summary: 1 package finished
```

### Step 3.4 — Source the Workspace

```bash
source ~/ros2_ws/install/setup.bash
```

Add to `~/.bashrc` to avoid sourcing every terminal:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## 4. The Phantom Robot — Why It Exists

Before running any test, understand why the launch file starts **two** expansion
nodes when only one physical robot is present.

### The Segfault Problem

`multirobot_map_merge`'s internal `composeGrids` function is hardcoded to
stitch **multiple** grids together. When it discovers only one robot namespace
and receives only one map, it attempts to apply an affine transform against a
`NULL` second matrix and crashes with exit code -11 (Segmentation Fault).

This is **Trigger B** from the `problems.md` diagnostic report:
> *"Launching the map_merge node when only Robot 1 has spawned and started
> SLAM, but Robot 2 has not yet initialized."*

### The Phantom Solution

The launch file starts a second `map_expansion_node` (`ausra_2_phantom`)
whose `input_topic` is set to `/map_phantom_never_published` — a topic that
does not exist on the hardware. Its subscriber never fires. However, its
**heartbeat timer** fires immediately at 1 Hz, publishing a valid 1000×1000
canvas filled entirely with `-1` (Unknown space).

The merger now discovers **two** namespaces (`ausra_1` and `ausra_2`) and
receives **two** valid grids. It initialises correctly. The phantom canvas
is all-Unknown, so it contributes nothing to the merged output — it only
prevents the crash.

```
ausra_1 expansion node ──► /ausra_1/map_fixed  (real SLAM data)   ┐
                                                                     ├─► /map_merged
ausra_2 phantom node   ──► /ausra_2/map_fixed  (all Unknown, -1)  ┘
```

**When to remove the phantom node:** When a second physical robot is added to
the fleet and its expansion node is active. At that point, remove the
`phantom_expansion_node` block from `map_merge_hw.launch.py`.

---

## 5. Phase 1 — Single Robot at Physical Origin

### Goal

Confirm the complete hardware pipeline runs without crashing and that the
expansion node correctly receives, processes, and publishes the hardware
SLAM map. No offset is applied — the robot is at the physical origin.

### 5.1 Physical Setup

Follow `docs/AUSRA_Hardware_Map_Merge_SOP.md` Phases 1 and 2:

1. Select and mark the physical origin point on the floor.
2. Run the X axis tape line from the origin.
3. Place the robot **at** the origin mark with its forward direction
   parallel to the X axis tape.
4. Record yaw alignment quality (must be at least Acceptable).

```
Origin mark (0,0) ──► +X axis tape
[Robot 1 placed HERE, facing +X]
```

### 5.2 Verify the Launch File Configuration

Open `launch/map_merge_hw.launch.py`. Confirm the `ROBOT_HW_CONFIG` block
is set to Phase 1 values:

```python
ROBOT_HW_CONFIG = {
    'ausra_1': {
        'offset_x': 0.0,   # ← Phase 1: robot at origin
        'offset_y': 0.0,   # ← Phase 1: robot at origin
    },
}
```

No other changes are needed.

### 5.3 Terminal Layout

Open **three terminals** on the machine running the map merge stack.

### 5.4 Terminal 1 — Launch the Robot Hardware Stack

```bash
# On the robot's onboard computer (or via SSH)
source ~/ros2_ws/install/setup.bash
ros2 launch your_robot_pkg hardware_full_stack.launch.py
```

Wait for the staged boot to complete (approximately 35 seconds):
- T+5s: EKF and SLAM start
- T+15s: Nav2 starts
- T+30s: Explore starts

### 5.5 Terminal 2 — Confirm SLAM is Publishing

```bash
# Verify the hardware SLAM map topic exists and is active
ros2 topic hz /map
```

Expected output:
```
average rate: 1.000
  min: 0.980s  max: 1.020s  std dev: 0.010s  window: 10
```

If this shows no data after 10 seconds of waiting, SLAM has not started.
Do not proceed to step 5.6 until this shows a positive rate.

Also verify the frame ID matches what the hardware publishes:
```bash
ros2 topic echo /map --once | grep frame_id
```

Expected:
```
  frame_id: map
```

If `frame_id` shows `ausrabot_map` or any other value, update the
`world_frame` parameter in `config/map_merge_HW_params.yaml` to match.

### 5.6 Terminal 3 — Launch the Map Merge Stack

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ausra_map_merge_HW map_merge_hw.launch.py
```

### 5.7 Expected Terminal Output

Within the first 3 seconds:

```
[map_expansion_ausra_1]: MapExpansionNode initialised: /map → /ausra_1/map_fixed
[map_expansion_ausra_1]: Canvas 1000×1000 @ 0.050 m/cell | Origin (-25.0, -25.0)
[map_expansion_ausra_1]: Robot spawn offset (0.00, 0.00)
[map_expansion_ausra_1]: Heartbeat timer armed at 1.0 Hz. Publishing all-Unknown canvas until SLAM arrives.

[map_expansion_ausra_2_phantom]: MapExpansionNode initialised: /map_phantom_never_published → /ausra_2/map_fixed
[map_expansion_ausra_2_phantom]: Heartbeat timer armed at 1.0 Hz. Publishing all-Unknown canvas until SLAM arrives.
```

After SLAM map is received by `ausra_1` node (~1 second after SLAM publishes):

```
[map_expansion_ausra_1]: First SLAM map received from frame 'map' at offset (500, 500) px.
[map_expansion_ausra_1]: Canvas now carries real map data.
```

After merger starts (~2 second delay from launch):

```
[map_merge]: Merging maps from 2 robots.
```

### 5.8 Verify Topic Output

```bash
# Confirm the fixed canvas is publishing
ros2 topic hz /ausra_1/map_fixed

# Confirm the merged map is publishing
ros2 topic hz /map_merged

# Inspect the canvas metadata
ros2 topic echo /ausra_1/map_fixed --no-arr --once
```

Expected metadata from `map_fixed`:

```
header:
  frame_id: map
info:
  resolution: 0.05
  width: 1000
  height: 1000
  origin:
    position:
      x: -25.0
      y: -25.0
      z: 0.0
    orientation:
      w: 1.0
```

**Phase 1 PASS criteria:**
- [ ] `map_expansion_node` starts without error
- [ ] `/ausra_1/map_fixed` publishes at 1 Hz
- [ ] `/ausra_2/map_fixed` (phantom) publishes at 1 Hz
- [ ] `/map_merged` publishes at 1 Hz
- [ ] `map_merge` node does not crash (no exit code -11)
- [ ] Canvas metadata shows `width=1000, height=1000, origin=(-25, -25)`

### 5.9 RViz Verification for Phase 1

Open RViz:

```bash
rviz2
```

Add the following displays:

| Display | Topic | Frame |
|---|---|---|
| Map | `/map` | `map` |
| Map | `/ausra_1/map_fixed` | `map` |
| Map | `/map_merged` | `map` |
| TF | — | — |

**Set Fixed Frame to `map`** in the Global Options panel.

**What you should see:**

1. `/map` — The raw SLAM map. This is slam_toolbox's dynamic, shifting map.
   Its size changes as the robot explores. Its origin moves. This is the
   unfixed, pre-expansion version.

2. `/ausra_1/map_fixed` — The fixed 1000×1000 canvas. It should show the
   same walls and features as `/map`, but:
   - It is always exactly 1000×1000 cells
   - Its origin is always `(-25.0, -25.0)` — never moves
   - The map content does not slide when slam_toolbox shifts its internal origin

3. `/map_merged` — Should look identical to `/ausra_1/map_fixed` for a
   single-robot test (the phantom robot contributes nothing visible).

**Specific Phase 1 validation check:**

Wait 60 seconds for the robot to build some map, then visually confirm:
- The walls visible in `/map` are in the same positions in `/ausra_1/map_fixed`
- The `/ausra_1/map_fixed` canvas does NOT slide or shift as the robot moves
- If `/map` appears to expand and its displayed origin shifts in RViz,
  `/ausra_1/map_fixed` must remain visually stable

---

## 6. Phase 2 — Single Robot at Measured Offset

### Goal

Confirm the `robot_offset_x/y` math in `map_expansion_node` correctly
translates the canvas by the physical spawn distance. After passing Phase 1,
this test proves the spatial math works before adding a second real robot.

### 6.1 Physical Setup

1. **Power down the robot.** Do not move it during a live SLAM session.
2. **Move the robot** to a new position that is exactly **X = 2.0 m, Y = 0.0 m**
   from the physical origin mark.
3. Align the robot's forward direction to the +X axis tape (yaw alignment).
4. Re-measure and record the actual position with cross-check (see SOP).

```
Origin (0,0) ────────── 2.0 m ──────────► [Robot 1 NEW POSITION]
                                            (facing +X, yaw aligned)
```

### 6.2 The Only Parameter Change Required

Open `launch/map_merge_hw.launch.py`. Edit **only** the `ROBOT_HW_CONFIG`
block. Change `offset_x` from `0.0` to `2.0`:

```python
# BEFORE (Phase 1):
ROBOT_HW_CONFIG = {
    'ausra_1': {
        'offset_x': 0.0,
        'offset_y': 0.0,
    },
}

# AFTER (Phase 2):
ROBOT_HW_CONFIG = {
    'ausra_1': {
        'offset_x': 2.0,   # ← CHANGED: robot is 2.0 m from origin on X axis
        'offset_y': 0.0,   # ← unchanged
    },
}
```

**Nothing else changes.** `map_merge_HW_params.yaml` is not touched.
`init_pose_*` values in the YAML stay at `0.0`.

### 6.3 Launch Sequence (Same as Phase 1)

```bash
# Terminal 1 — Hardware stack
ros2 launch your_robot_pkg hardware_full_stack.launch.py

# Terminal 2 — Verify SLAM is publishing
ros2 topic hz /map

# Terminal 3 — Map merge stack
ros2 launch ausra_map_merge_HW map_merge_hw.launch.py
```

### 6.4 Expected Terminal Output Change

The only visible difference from Phase 1 in the logs:

```
[map_expansion_ausra_1]: Robot spawn offset (2.00, 0.00)    ← was (0.00, 0.00)
[map_expansion_ausra_1]: First SLAM map received from frame 'map' at offset (540, 500) px.
                                                                              ^^^
                                                                    was 500, now 540
                                                      (2.0m / 0.05 m/cell = 40 cells shift)
```

The canvas pixel offset for the origin is now 540 in X instead of 500.
That 40-cell shift (40 × 0.05 m = 2.0 m) is the physical offset being applied.

### 6.5 RViz Verification for Phase 2

**What you should see:**

Compare `/ausra_1/map_fixed` between Phase 1 and Phase 2 in RViz:

- The **canvas boundary** stays fixed (always -25.0 to +25.0 in both X and Y)
- The **map content** (walls, free space) shifts **2.0 m in the +X direction**
  relative to Phase 1

Concretely: if in Phase 1 a wall appeared at canvas coordinates `(0.5, 1.0)`,
in Phase 2 the same physical wall should appear at canvas coordinates `(2.5, 1.0)`.

**Measurement test in RViz:**

Use the Measure tool to measure the distance between the same known wall feature
in Phase 1 (screenshot or memory) and Phase 2 (live view).

Expected: The wall has moved **+2.0 m in the X direction** (right on screen,
assuming default RViz orientation).

Tolerance: ±5 cm (±1 canvas cell). If the shift is larger than 10 cm from the
expected 2.0 m, re-measure the physical robot position.

**Phase 2 PASS criteria:**
- [ ] `/ausra_1/map_fixed` terminal log shows offset `(540, 500)` pixels
- [ ] In RViz, the map content has shifted ~2.0 m in +X vs Phase 1
- [ ] The canvas boundary remains at (-25.0, -25.0) → (25.0, 25.0)
- [ ] `/map_merged` reflects the shifted content
- [ ] No crashes, no `Canvas overflow` warnings in terminal

---

## 7. ⚠️ Critical Warning — `init_pose` Must Be Zero

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          CRITICAL WARNING                               │
│                                                                         │
│  The init_pose_* values in map_merge_HW_params.yaml must ALWAYS be     │
│  0.0 for all robots.                                                    │
│                                                                         │
│  DO NOT set them to the robot's physical position.                      │
│  DO NOT set them to robot_offset_x / robot_offset_y values.            │
│  DO NOT set them to any non-zero value.                                 │
│                                                                         │
│  WHY: The map_expansion_node has already applied the physical spawn     │
│  offset by converting local SLAM coordinates to global canvas pixels.  │
│  The merger receives grids that are already globally aligned.           │
│  If init_pose_* is non-zero, the merger applies the spawn offset a     │
│  SECOND TIME, shifting every pixel by twice the correct distance.      │
│  The merged map will appear wrong with no error message.               │
│                                                                         │
│  The spawn coordinates live in map_merge_hw.launch.py (ROBOT_HW_CONFIG)│
│  The merger config lives in map_merge_HW_params.yaml (stays at 0.0)   │
│                                                                         │
│  One parameter, one place. Never both.                                  │
└─────────────────────────────────────────────────────────────────────────┘
```

### Correct Configuration (copy this exactly)

`config/map_merge_HW_params.yaml`:
```yaml
/ausra_1/map_merge/init_pose_x:   0.0   # DO NOT CHANGE
/ausra_1/map_merge/init_pose_y:   0.0   # DO NOT CHANGE
/ausra_1/map_merge/init_pose_z:   0.0
/ausra_1/map_merge/init_pose_yaw: 0.0

/ausra_2/map_merge/init_pose_x:   0.0   # DO NOT CHANGE
/ausra_2/map_merge/init_pose_y:   0.0   # DO NOT CHANGE
/ausra_2/map_merge/init_pose_z:   0.0
/ausra_2/map_merge/init_pose_yaw: 0.0
```

`launch/map_merge_hw.launch.py` `ROBOT_HW_CONFIG`:
```python
# Phase 1:
'ausra_1': {'offset_x': 0.0, 'offset_y': 0.0}    # ← spawn coords go HERE

# Phase 2:
'ausra_1': {'offset_x': 2.0, 'offset_y': 0.0}    # ← spawn coords go HERE
```

---

## 8. RViz Setup Reference

### Recommended Display Configuration

```
Global Options:
  Fixed Frame: map

Displays:
  ┌ Map
  │   Topic: /map
  │   Color Scheme: costmap
  │   Update Topic: /map
  └ (Raw SLAM output — unfixed, may slide)

  ┌ Map
  │   Topic: /ausra_1/map_fixed
  │   Color Scheme: map
  └ (Fixed canvas — Phase 1/2 validation target)

  ┌ Map
  │   Topic: /map_merged
  │   Color Scheme: map
  └ (Final merged output)

  ┌ TF
  └ (Verify map → ausrabot_odom → ausrabot_robot_footprint frames)

  ┌ RobotModel
  └ (Confirm robot position matches physical placement)
```

### Saving the RViz Config

```bash
# Save to package for reuse
cp ~/.rviz2/default.rviz ~/ros2_ws/src/ausra_map_merge_HW/config/hardware_test.rviz
```

---

## 9. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `map_merge` crashes with exit code -11 immediately | Phantom node not starting before merger | Check phantom node is in launch file; ensure 2-second delay before merger starts |
| `/ausra_1/map_fixed` not publishing | `input_topic` wrong — SLAM not publishing to `/map` | Run `ros2 topic list \| grep map`; check actual SLAM output topic |
| Canvas metadata shows wrong frame_id | Hardware SLAM publishes with unexpected frame_id | Check `ros2 topic echo /map --once \| grep frame_id`; update `world_frame` in YAML |
| Map content in Phase 2 shifts by 4.0 m instead of 2.0 m | `init_pose_*` set to `2.0` in YAML (double-shift) | Set all `init_pose_*` back to `0.0` in `map_merge_HW_params.yaml` |
| Map content does not shift at all between Phase 1 and 2 | `offset_x` not updated in `ROBOT_HW_CONFIG` | Edit `map_merge_hw.launch.py`, set `offset_x: 2.0` for `ausra_1` |
| Map slides / moves during exploration | Moving floor bug — canvas math incorrect | Verify `map_expansion_node.cpp` is the heartbeat version; check `robot_offset_x/y` signs |
| `Canvas overflow` warnings in terminal | Robot exploring beyond 25 m canvas boundary | Increase `canvas_width/height` to `2000` or reposition origin |
| SLAM frame_id is `ausrabot_map` not `map` | slam_toolbox published with different frame | Update `world_frame: ausrabot_map` in `map_merge_HW_params.yaml` |
| `/map` topic not found | SLAM not started yet | Wait for hardware_full_stack T+5s stage; run `ros2 topic hz /map` to verify |

---

## Appendix: Phase Progression Checklist

```
Phase 1 — Robot at Origin
  [ ] Physical origin marked on floor
  [ ] X axis tape line installed
  [ ] Robot placed at origin with yaw aligned to +X tape
  [ ] ROBOT_HW_CONFIG: offset_x=0.0, offset_y=0.0
  [ ] map_merge_HW_params.yaml: all init_pose_* = 0.0
  [ ] /map publishes at 1 Hz
  [ ] /ausra_1/map_fixed publishes at 1 Hz, origin=(-25,-25), size=1000x1000
  [ ] /map_merged publishes at 1 Hz, no crash
  [ ] RViz: map_fixed content stable (no sliding)
  → PHASE 1 PASSED

Phase 2 — Robot at Offset
  [ ] Robot physically moved to (2.0, 0.0) from origin
  [ ] Yaw re-aligned to +X tape
  [ ] ROBOT_HW_CONFIG: offset_x=2.0, offset_y=0.0 (ONLY change)
  [ ] map_merge_HW_params.yaml: unchanged (all 0.0)
  [ ] Terminal log shows offset (540, 500) px
  [ ] RViz: map content shifted +2.0 m in X vs Phase 1 result
  [ ] Measure tool confirms ~2.0 m shift ±5 cm
  → PHASE 2 PASSED → Ready for dual-robot SOP testing
```
