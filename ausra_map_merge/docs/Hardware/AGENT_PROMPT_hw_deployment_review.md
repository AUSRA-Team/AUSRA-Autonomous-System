# Agent Task: Hardware Deployment Review and Package Creation
## `ausra_map_merge_HW`

---

## Your Role

You are a Senior ROS 2 Systems Architect and Hardware Deployment Engineer.
Your task is to review a set of design and implementation documents for the
AUSRA multi-robot map merging system, make a structured go/no-go deployment
decision, and if the decision is GO, create a complete, deployment-ready
ROS 2 package called `ausra_map_merge_HW`.

---

## Files to Review (Read ALL of these before making any decision)

Read each file in full before proceeding to the decision phase.

### Architecture and Implementation
1. `AUSRA_Map_Merge_Code_Explanation.md`
   — The current working simulation architecture. Understand the Smart Canvas
     pattern, `robot_offset_x/y` math, and the zero `init_pose_*` contract.

2. `map_expansion_node.cpp`
   — The corrected, production-ready C++ node with heartbeat timer architecture.
     This is the core of the entire pipeline.

3. `map_merge.launch.py`
   — The corrected launch file with YAML-based init pose loading.

4. `init_poses.yaml`
   — The parameter file. Understand WHY all `init_pose_*` values are 0.0.

### Bug History and Fixes
5. `moving_floor_bug_and_canvas_fix.md`
   — The root cause analysis of the sliding map bug and the mathematical
     proof of the canvas fix. Critical for understanding what was wrong.

6. `AUSRA_canvas_fix_audit.md`
   — Pre-deployment audit of the canvas node: all six known issues and their
     fixes. Confirm these are all addressed in `map_expansion_node.cpp`.

### Hardware Deployment Plan
7. `AUSRA_Hardware_Map_Merge_SOP.md`
   — The winning hardware deployment strategy (Physical World Reference Frame).
     Review the procedure, configuration rules, and validation steps.

8. `Alternative_Hardware_Strategies.md`
   — The four alternative strategies (Fixed Stations, ArUco, UWB, LiDAR
     Scan Match). Understand the upgrade path.

---

## Decision Phase: Go / No-Go Criteria

After reading all files, evaluate the following checklist. Each item must
pass for a GO decision.

### Technical Readiness
- [ ] The `map_expansion_node.cpp` implements the heartbeat timer pattern
      (publisher decoupled from subscriber).
- [ ] The six issues from `AUSRA_canvas_fix_audit.md` are all addressed
      in the current `map_expansion_node.cpp` (resolution guard, data size
      guard, overflow warning, correct comment, partial reset optimisation,
      and the wrong-comment fix on frame conventions).
- [ ] The `robot_offset_x/y` math is present and correctly converts
      local SLAM origin → global canvas pixel offset.
- [ ] The `init_pose_*` values are confirmed to be 0.0 in the YAML, with
      clear documentation explaining why.
- [ ] The moving floor bug mathematical invariant is preserved:
      `canvas_col = (world_x_of_cell - canvas_origin_x) / resolution`
      is independent of slam_toolbox origin drift.

### Hardware Deployment Readiness
- [ ] `AUSRA_Hardware_Map_Merge_SOP.md` contains a complete physical
      setup procedure including origin selection, X axis tape marking,
      per-robot measurement, yaw alignment, config update, launch sequence,
      and post-launch validation.
- [ ] The SOP explicitly warns against the double-shift misconfiguration
      (setting `init_pose_*` to spawn coordinates in the YAML).
- [ ] The SOP includes a yaw alignment step with a pass/fail quality check.
- [ ] The SOP includes a validation procedure to detect translational
      misalignment and yaw misalignment separately.
- [ ] `Alternative_Hardware_Strategies.md` documents at least three
      future upgrade strategies with integration notes.

### Decision Output

State your decision clearly:

```
DECISION: GO / NO-GO
REASON:   <one paragraph — cite specific checklist items if NO-GO>
BLOCKING ISSUES (if NO-GO): <numbered list of exact things that must be
                              fixed before reconsidering>
```

If NO-GO: stop here. List exactly what must be fixed and do not create
any package files.

If GO: proceed to the Package Creation phase below.

---

## Package Creation Phase (Execute only if decision is GO)

Create a complete ROS 2 package named `ausra_map_merge_HW` under the path:
```
/mnt/user-data/outputs/ausra_map_merge_HW/
```

This package is the hardware-only variant of the simulation package. It must
be immediately deployable on physical AUSRA robots using the tape-measure SOP.

### Required Package Structure

```
ausra_map_merge_HW/
├── CMakeLists.txt
├── package.xml
├── src/
│   └── map_expansion_node.cpp          ← from reviewed file, hardware-ready
├── launch/
│   └── map_merge_HW.launch.py          ← hardware launch file (see spec below)
├── config/
│   ├── map_merge_HW_params.yaml        ← merger config (see spec below)
│   └── robot_HW_poses.yaml             ← new file (see spec below)
├── docs/
│   ├── AUSRA_Hardware_Map_Merge_SOP.md ← copy from reviewed file
│   └── Alternative_Hardware_Strategies.md ← copy from reviewed file
└── README.md                           ← generate (see spec below)
```

---

### File Specifications

#### `CMakeLists.txt`
Standard ROS 2 CMake file for a C++ executable node. Must:
- Set `cmake_minimum_required(VERSION 3.8)`
- Find packages: `rclcpp`, `nav_msgs`
- Build the executable `map_expansion_node` from `src/map_expansion_node.cpp`
- Install the executable, launch files, and config directory

#### `package.xml`
Standard ROS 2 package manifest. Must:
- Set package name: `ausra_map_merge_HW`
- Set version: `1.0.0`
- Set description: `Hardware deployment variant of ausra_map_merge for physical AUSRA robots`
- Declare `exec_depend` on: `rclcpp`, `nav_msgs`, `multirobot_map_merge`
- Use `ament_cmake` build type

#### `src/map_expansion_node.cpp`
Use the reviewed `map_expansion_node.cpp` exactly as-is. Do not modify the
code. Add only one change at the top of the file comment block:

```cpp
// HARDWARE DEPLOYMENT VARIANT
// Package: ausra_map_merge_HW
// Target:  Physical AUSRA robots (not Gazebo simulation)
// Config:  Set robot_offset_x/y from tape-measured spawn positions.
//          See config/robot_HW_poses.yaml and docs/AUSRA_Hardware_Map_Merge_SOP.md
```

#### `config/robot_HW_poses.yaml`
A new YAML file that stores the physical robot spawn positions measured during
the SOP procedure. This is the hardware equivalent of `ROBOT_SPAWN_POSES` in
the simulation launch file.

```yaml
# config/robot_HW_poses.yaml
#
# HARDWARE DEPLOYMENT CONFIGURATION
# ──────────────────────────────────────────────────────────────────────────
# Record tape-measured robot positions here before each deployment session.
# Values are in metres, relative to the physical origin mark on the floor.
#
# HOW TO FILL THIS FILE:
#   Follow AUSRA_Hardware_Map_Merge_SOP.md Phase 2 (Robot Placement).
#   Measure robot_offset_x and robot_offset_y for each robot.
#   Robot 1 should always be placed AT the physical origin (0.0, 0.0).
#
# IMPORTANT: These values feed into robot_offset_x/y in map_expansion_node.
#   Do NOT put these values into map_merge_HW_params.yaml.
#   The init_pose_* values in map_merge_HW_params.yaml must always be 0.0.
#   Setting init_pose_* to these values causes a double-shift (wrong map).
#
# ──────────────────────────────────────────────────────────────────────────

robot_hw_poses:
  ausra_1:
    offset_x: 0.0        # Robot 1 placed AT the physical origin
    offset_y: 0.0        # Update if Robot 1 is not at origin
    # yaw_alignment_quality: Good  ← record from SOP Phase 2 Step 4.5

  ausra_2:
    offset_x: 0.0        # ← REPLACE with tape-measured value before launch
    offset_y: 0.0        # ← REPLACE with tape-measured value before launch
    # yaw_alignment_quality:       ← record from SOP Phase 2 Step 4.5

  # Template for additional robots:
  # ausra_3:
  #   offset_x: 0.0
  #   offset_y: 0.0
```

#### `config/map_merge_HW_params.yaml`
Merge node configuration. Identical in function to the simulation YAML but
with hardware-specific comments. Must include:
- `robot_map_topic: map_fixed`
- `known_init_poses: true`
- `world_frame: map`
- `robot_namespace: ausra_`
- All `init_pose_*` values set to `0.0` for all robots
- A prominent comment block explaining why they are 0.0 and what happens
  if someone incorrectly sets them to spawn coordinates

#### `launch/map_merge_HW.launch.py`
Hardware-specific launch file. It must:

1. Read `robot_hw_poses.yaml` at launch time to get offset values
   (do not hardcode offsets in the launch file itself — read from YAML).

2. For each robot in the YAML, spawn a `map_expansion_node` with the
   correct `robot_offset_x` and `robot_offset_y` from the YAML.

3. Launch the `multirobot_map_merge` node with `map_merge_HW_params.yaml`.

4. Include a startup log message (via `LogInfo` action) that prints:
   ```
   [AUSRA HW] Launching map_merge_HW with N robots.
   [AUSRA HW] Robot offsets loaded from robot_HW_poses.yaml.
   [AUSRA HW] Confirm all robots are at tape-marked positions with correct yaw.
   ```

5. Include a `DeclareLaunchArgument` for `poses_file` so the operator can
   override the default YAML path:
   ```
   ros2 launch ausra_map_merge_HW map_merge_HW.launch.py \
     poses_file:=/path/to/custom_poses.yaml
   ```

#### `README.md`
Generate a complete README that covers:

1. **Package Purpose** — One paragraph. What problem this solves, difference
   from simulation package.

2. **Prerequisites** — ROS 2 Humble, `multirobot_map_merge`, `slam_toolbox`.

3. **Quick Start** — Three steps:
   - Follow `docs/AUSRA_Hardware_Map_Merge_SOP.md` to measure and fill
     `config/robot_HW_poses.yaml`
   - Build: `colcon build --packages-select ausra_map_merge_HW`
   - Launch: `ros2 launch ausra_map_merge_HW map_merge_HW.launch.py`

4. **Architecture Overview** — The Smart Canvas + Dumb Overlay pattern.
   The `robot_offset_x/y` math. Why `init_pose_*` is always 0.0.

5. **Configuration Reference** — Table of all parameters in
   `robot_HW_poses.yaml` and `map_merge_HW_params.yaml` with types,
   defaults, and descriptions.

6. **Fault Tolerance** — Explain the heartbeat timer behaviour across all
   four lifecycle states (before SLAM, SLAM online, robot dies, new robot joins).

7. **Common Mistakes** — The double-shift table from the SOP, reproduced here.

8. **Upgrade Path** — One-line summary of each alternative strategy from
   `Alternative_Hardware_Strategies.md` with a reference to that doc.

---

## Output Requirements

1. All files must be fully written — no placeholders, no `TODO` comments,
   no `...` abbreviations. Every file must be immediately usable.

2. After creating all files, run a final consistency check:
   - Confirm `robot_offset_x/y` flows correctly from `robot_HW_poses.yaml`
     → `map_merge_HW.launch.py` → `map_expansion_node` parameters.
   - Confirm no file sets `init_pose_*` to non-zero values.
   - Confirm `map_expansion_node.cpp` is unmodified except for the header comment.

3. Present all created files when complete.

4. End with a one-paragraph deployment readiness statement summarising
   what the hardware team must do before the first physical run.

---

## Constraints

- Do not modify the logic inside `map_expansion_node.cpp`.
- Do not modify `multirobot_map_merge` source code.
- Do not create any new Python or C++ nodes beyond what is specified above.
- The package must build with standard `colcon build` on ROS 2 Humble.
- All YAML files must use valid ROS 2 parameter file syntax.
- All Python launch files must use the `launch` and `launch_ros` APIs only.
