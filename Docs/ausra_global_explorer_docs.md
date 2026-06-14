# AUSRA Global Frontier Explorer — System Documentation

> **Status:** Implemented, ready for simulation testing before hardware deployment.
> Communication issues on hardware → test in simulation first (see Section 4).

---

## 1. What Was Built and Why

### The Problem With Per-Robot explore_lite

If each robot runs its own `explore_lite` subscribing to `/map_merged`, every instance independently scores ALL frontiers and picks the **same** highest-scoring one. Both robots race to the same point → inefficient, potentially colliding.

### The Solution

A single **`frontier_coordinator`** node runs on the **base station**. It is the only entity that assigns navigation goals. Each frontier is given to at most one robot at a time. All robots move in parallel to different frontiers.

---

## 2. Package Structure — `ausra_global_explorer`

```
AUSRA-Autonomous-System/
└── Navigation/
    └── ausra_global_explorer/           ← NEW ROS 2 Python package
        ├── package.xml                  ← ROS 2 package manifest
        ├── setup.py                     ← Python entry points
        ├── setup.cfg                    ← ament_python build config
        ├── resource/
        │   └── ausra_global_explorer    ← ament resource index marker
        ├── config/
        │   └── coordinator_params.yaml  ← all tunable parameters
        └── ausra_global_explorer/
            ├── __init__.py
            └── frontier_coordinator.py  ← THE MAIN NODE
```

### `frontier_coordinator.py` — What It Does

| Component | Role |
|---|---|
| `Frontier` class | Stores a frontier centroid `(x, y)` in map frame and its cell count |
| `RobotState` class | Tracks each robot: idle/busy, current goal, goal handle, start time |
| `_on_map()` | Stores latest `/map_merged` whenever it arrives |
| `_plan()` | Runs every 5 s: cancels stuck robots, finds frontiers, assigns goals |
| `_find_frontiers()` | NumPy-based detection: free cells adjacent to unknown → BFS clusters |
| `_get_robot_pose()` | TF lookup `map → <robot>_robot_footprint` → live global position |
| `_send_goal()` | Sends `NavigateToPose` action to `/<robot>/navigate_to_pose` |
| `_on_goal_result()` | On finish: marks robot idle, blacklists on abort |

### `coordinator_params.yaml` — Tunable Parameters

```yaml
robot_names:          "ausra_1,ausra_2"   # comma-separated, must match namespaces
map_topic:            "/map_merged"        # merged map source
planning_rate_hz:     0.2                 # recheck every 5 seconds
min_frontier_cells:   10                  # ignore tiny frontier clusters
blacklist_radius_m:   0.75               # blacklist radius around failed goals (m)
progress_timeout_s:   90.0               # cancel stuck robot after 90 s
free_threshold:       50                  # OccupancyGrid: 0–49 = free, 50+ = obstacle
visualize:            true               # publish /frontier_coordinator/frontiers markers
```

---

## 3. Changes to Existing Files

### 3.1 `hardware_full_stack.launch.py` — Two Changes

#### Change A — New `use_local_explorer` launch argument (line 74 + 582–588)

```python
# Added to LaunchConfiguration block:
use_local_explorer = LaunchConfiguration('use_local_explorer', default='false')

# Added to generate_launch_description():
DeclareLaunchArgument(
    'use_local_explorer',
    default_value='false',
    description=(
        'true  = run explore_lite on this robot (single-robot or testing). '
        'false = skip Stage 3; base-station frontier_coordinator sends goals.')
),
```

#### Change B — Stage 3 is now conditional (lines 508–529)

```python
# BEFORE — explore_lite always launched:
TimerAction(period=30.0, actions=[GroupAction([..., exploration_server])])

# AFTER — conditional on use_local_explorer:
*(
    [TimerAction(period=30.0, actions=[GroupAction([..., exploration_server])])]
    if use_local_explorer.perform(context) == 'true'
    else [LogInfo(msg='Stage 3: Skipped — coordinator will assign goals.')]
),
```

#### Change C — Stage 3 explore_lite costmap source (lines 439–453)

```python
# BEFORE — robot's own local costmap:
'costmap_topic': 'global_costmap/costmap',
'costmap_updates_topic': 'global_costmap/costmap_updates',

# AFTER — global merged map:
'costmap_topic': '/map_merged',
'costmap_updates_topic': '/map_merged_updates_unused',  # disabled, no partial updates
```

> This only applies when `use_local_explorer:=true`. In normal multi-robot mode
> Stage 3 is skipped entirely.

---

### 3.2 `start_global_frontier.sh` — New Base Station Script

Located at `AUSRA-Autonomous-System/start_global_frontier.sh`

**What it does:**
1. Parses robot configs from CLI args (`name:x:y`)
2. Builds `robot_config` string for `map_merge_hw.launch.py`
3. Launches map merge pipeline (expansion nodes + multirobot_map_merge)
4. Waits 5 seconds for heartbeat canvases
5. Launches `frontier_coordinator` with the robot names CSV
6. Prints matching robot launch commands as a reminder

**Usage:**
```bash
./start_global_frontier.sh ausra_1:0.0:0.0 ausra_2:1.5:0.0
./start_global_frontier.sh ausra_1:0.0:0.0 ausra_2:1.5:0.0 ausra_3:3.0:0.0
./start_global_frontier.sh --help
```

---

## 4. Operating Procedure

### 4.1 Order of Operations (Hardware)

```
Step 1 — On each Jetson (FIRST):
  ros2 launch lidar_slam_pkg hardware_full_stack.launch.py \
    robot_name:=ausra_1  x:=<tape_x>  y:=<tape_y>  yaw:=0.0

  ros2 launch lidar_slam_pkg hardware_full_stack.launch.py \
    robot_name:=ausra_2  x:=<tape_x>  y:=<tape_y>  yaw:=0.0

Step 2 — On the Base Station laptop (AFTER robots are up and SLAM is running):
  ./start_global_frontier.sh  ausra_1:<tape_x>:<tape_y>  ausra_2:<tape_x>:<tape_y>
```

> **Critical:** The `x:=` and `y:=` values on each Jetson must be **identical** to
> the `x:y` in the script. Both use the same tape-measured spawn offsets.
> The script prints a reminder of the exact robot commands.

### 4.2 Build First

```bash
cd ~/Swarm-HW           # workspace root (one level above src/)
colcon build --packages-select ausra_global_explorer
source install/setup.bash
```

### 4.3 Verification Commands

```bash
# Check /map_merged is live and arriving from robots:
ros2 topic hz /map_merged

# Watch coordinator assignments in real time:
ros2 node info /frontier_coordinator

# See frontier markers in RViz2:
# Add → MarkerArray → /frontier_coordinator/frontiers
# Blue = available frontier, Grey = already assigned to a robot

# Check a robot is receiving goals:
ros2 action list | grep navigate_to_pose
ros2 action status /ausra_1/navigate_to_pose
```

---

## 5. Simulation Migration Guide

Since hardware communication has issues, test in simulation first.

### 5.1 What Simulation Replaces

| Hardware component | Simulation equivalent |
|---|---|
| Jetson Nano + real robot | Gazebo spawned robot with TurtleBot3 or custom URDF |
| Micro-ROS / hardware drivers | Gazebo differential drive plugin |
| LiDAR hardware | Gazebo ray sensor plugin |
| Physical tape measurement | Gazebo spawn `x:=` `y:=` arguments |
| ROS 2 DDS over WiFi | localhost (same machine, no network issues) |

### 5.2 Key Parameter Changes for Simulation

When launching in simulation, add `use_sim_time:=true` everywhere:

```bash
# Robot launch (simulation):
ros2 launch lidar_slam_pkg hardware_full_stack.launch.py \
  robot_name:=ausra_1  x:=0.0  y:=0.0  use_sim_time:=true

# Frontier coordinator (simulation):
ros2 run ausra_global_explorer frontier_coordinator \
  --ros-args \
  -p robot_names:="ausra_1,ausra_2" \
  -p use_sim_time:=true
```

### 5.3 Minimal Simulation Test — Single Machine

The simplest test: run everything on ONE laptop, no network needed.

```
Terminal 1 — Robot 1:
  ros2 launch <your_sim_pkg> spawn_robot.launch.py \
    namespace:=ausra_1  x:=0.0  y:=0.0

Terminal 2 — Robot 2:
  ros2 launch <your_sim_pkg> spawn_robot.launch.py \
    namespace:=ausra_2  x:=1.5  y:=0.0

Terminal 3 — Map merge + coordinator:
  ./start_global_frontier.sh  ausra_1:0.0:0.0  ausra_2:1.5:0.0

Terminal 4 — RViz2:
  ros2 run rviz2 rviz2
  # Add: /map_merged, /frontier_coordinator/frontiers, TF
```

### 5.4 Things to Validate in Simulation Before Hardware

- [ ] `/map_merged` publishes and both robot maps appear correctly aligned
- [ ] `map → ausra_1_robot_footprint` TF chain resolves (check with `ros2 run tf2_tools view_frames`)
- [ ] Coordinator logs show frontier detection: `found X frontiers`
- [ ] Coordinator logs show assignment: `[ASSIGN] ausra_1 → frontier (x, y)`
- [ ] Both robots receive and execute their Nav2 goals simultaneously
- [ ] When `ausra_1` finishes, it gets a new frontier without waiting for `ausra_2`
- [ ] Blue markers appear in RViz2 at frontier locations

### 5.5 Adjusting `planning_rate_hz` for Simulation

In simulation the map updates faster. You can increase the rate for quicker testing:

```bash
ros2 run ausra_global_explorer frontier_coordinator \
  --ros-args \
  -p robot_names:="ausra_1,ausra_2" \
  -p planning_rate_hz:=0.5 \
  -p progress_timeout_s:=30.0 \
  -p min_frontier_cells:=5
```

---

## 6. The Offset Flow (Quick Reference)

```
You type: ausra_2:1.5:0.0
              │
    ┌─────────┴──────────────┐
    ▼                        ▼
map_expansion_node       static_transform_publisher
robot_offset_x = 1.5     "map" → "ausra_2_map"
                          at position (1.5, 0.0)
    │                        │
    ▼                        ▼
SLAM canvas shifted      TF chain anchored
1.5m right on            1.5m right in
shared canvas            global frame
    │                        │
    └─────────┬──────────────┘
              ▼
    /map_merged: ausra_2's walls
    appear at correct global position
    AND
    coordinator TF lookup returns
    ausra_2's live position in global frame
```

> The offset is used **twice with the same value** — once for canvas placement,
> once for TF anchoring. They must always match.
