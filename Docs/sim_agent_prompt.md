# Agent Prompt — Adapt Global Frontier Coordinator to Simulation

## Context You Must Know First

I have a **working multi-robot simulation** with the following setup:

- Multiple robots are launched with namespaces (e.g., `ausra_1`, `ausra_2`).
  Each robot is given its spawn position when launched.
- A **map merge** node was running and publishing a merged map (the topic name
  may be `/map_merged` or something else — check the simulation launch files).
- **Local explore_lite** (`explore_lite` / `explore` executable from `m-explore-ros2`)
  was running inside each robot's namespace and working correctly.
- The map merge launch file is **hardcoded** (not dynamic like the hardware version).
  It already knows the robot namespaces and their offsets — do NOT change it.

I do NOT want to touch the map merge. I only want to:
1. **Disable** the per-robot explore_lite (or make it skippable)
2. **Add** a central `frontier_coordinator` node that assigns unique frontiers
   to each robot from the merged map so they never go to the same frontier

---

## What You Need to Find in This Workspace

Before making any changes, inspect the simulation code to find:

### A. The merged map topic name
- Run `ros2 topic list | grep map` while the simulation is running, OR
- Open the map merge launch file and look for `merged_map_topic` or the topic
  the `multirobot_map_merge` node publishes to
- It may be `/map_merged`, `/merged_map`, or something else

### B. The robot base frame names
- What TF frame represents each robot's footprint?
- Open the URDF or the robot launch file and look for `base_frame` or `robot_base_frame`
- It may be `<robot_name>_robot_footprint`, `<robot_name>/base_footprint`,
  `<robot_name>_base_link`, or something else
- This is what the coordinator uses to look up the robot's live position:
  `tf_buffer.lookup_transform("map", "<base_frame>")`

### C. The Nav2 action server topic per robot
- Check if each robot's Nav2 navigate_to_pose action is at:
  `/<robot_name>/navigate_to_pose`
- Confirm by running: `ros2 action list` while simulation is running

### D. What explore_lite was subscribing to
- Open the explore_lite launch/config for the simulation
- What was `costmap_topic` set to? (e.g., `global_costmap/costmap` or `/map_merged`)
- Find where explore_lite is launched so you can disable it

---

## The `frontier_coordinator` Node — Copy This Exactly

Create the following ROS 2 Python package in this simulation workspace.
The package is called `ausra_global_explorer`.

### Package Structure to Create

```
<your_sim_workspace>/src/ausra_global_explorer/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── ausra_global_explorer
├── config/
│   └── coordinator_params.yaml
└── ausra_global_explorer/
    ├── __init__.py
    └── frontier_coordinator.py
```

### `package.xml`

```xml
<?xml version="1.0"?>
<package format="3">
  <name>ausra_global_explorer</name>
  <version>0.1.0</version>
  <description>Centralized multi-robot frontier coordinator</description>
  <maintainer email="ausra@team.local">AUSRA Team</maintainer>
  <license>BSD</license>
  <depend>rclpy</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav2_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>action_msgs</depend>
  <depend>visualization_msgs</depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### `setup.cfg`

```ini
[develop]
script_dir=$base/lib/ausra_global_explorer

[install]
install_scripts=$base/lib/ausra_global_explorer
```

### `setup.py`

```python
from setuptools import setup
import os
from glob import glob

package_name = 'ausra_global_explorer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'frontier_coordinator = ausra_global_explorer.frontier_coordinator:main',
        ],
    },
)
```

### `resource/ausra_global_explorer`
Empty file — just create it.

### `ausra_global_explorer/__init__.py`
Empty file.

### `coordinator_params.yaml`

```yaml
/**:
  ros__parameters:
    robot_names: "ausra_1,ausra_2"     # CHANGE: comma-separated, match your sim namespaces
    map_topic: "/map_merged"           # CHANGE: use the actual merged map topic you found
    planning_rate_hz: 0.5             # 0.5 Hz for simulation (faster than hardware)
    min_frontier_cells: 5             # Lower for simulation (smaller maps)
    blacklist_radius_m: 0.5
    progress_timeout_s: 30.0          # Lower for simulation
    free_threshold: 50
    visualize: true
```

### `frontier_coordinator.py`

Copy this file exactly. It contains:
- `Frontier` class: stores centroid `(x, y)` and cell count
- `RobotState` class: tracks each robot's idle state, current goal, goal handle
- `_on_map()`: stores latest merged map
- `_plan()`: runs on timer — cancels stuck robots, finds frontiers, assigns goals
- `_find_frontiers()`: NumPy BFS-based frontier detection (free cells adjacent to unknown)
- `_get_robot_pose()`: TF lookup `map → <robot_base_frame>`
- `_send_goal()`: sends `NavigateToPose` action to `/<robot>/navigate_to_pose`

**CRITICAL ADAPTATION NEEDED in `frontier_coordinator.py`:**

In `RobotState.__init__()`, the base frame is currently hardcoded as:
```python
self.base_frame = f'{name}_robot_footprint'
```

You MUST change this to match the actual TF frame name used in simulation.
For example, if your robots use `ausra_1/base_footprint`, change to:
```python
self.base_frame = f'{name}/base_footprint'
```

Or if it uses `ausra_1_base_link`:
```python
self.base_frame = f'{name}_base_link'
```

Find the correct frame name FIRST (Step B above), then edit this one line.

---

## Disabling Per-Robot explore_lite

Find where `explore_lite` (the `explore` node) is launched for each robot in simulation.

**Option A — Comment it out** in the launch file:
```python
# exploration_server = Node(package='explore_lite', ...)  # disabled: coordinator takes over
```

**Option B — Add a condition** (preferred, keeps option to re-enable):
```python
use_global_coordinator = LaunchConfiguration('use_global_coordinator', default='true')

# Only launch explore_lite if NOT using global coordinator
*(
    [Node(package='explore_lite', ...)]
    if use_global_coordinator.perform(context) != 'true'
    else [LogInfo(msg='explore_lite disabled — global coordinator active')]
),
```

Then launch robots with `use_global_coordinator:=true` (the default).

---

## Full Command Lines for Simulation Testing

After implementing the above, use these commands in order:

### Step 1 — Build the new package

```bash
cd <your_sim_workspace>
colcon build --packages-select ausra_global_explorer
source install/setup.bash
```

### Step 2 — Launch Gazebo world (in its own terminal)

```bash
ros2 launch <your_gazebo_pkg> <world_launch_file>.launch.py
```

### Step 3 — Launch Robot 1 (in its own terminal)

```bash
ros2 launch <your_robot_pkg> <robot_launch>.launch.py \
  robot_name:=ausra_1  x:=0.0  y:=0.0  use_sim_time:=true
```

### Step 4 — Launch Robot 2 (in its own terminal)

```bash
ros2 launch <your_robot_pkg> <robot_launch>.launch.py \
  robot_name:=ausra_2  x:=1.5  y:=0.0  use_sim_time:=true
```

### Step 5 — Launch Map Merge (in its own terminal, unchanged from before)

```bash
ros2 launch <your_map_merge_pkg> <map_merge_launch>.launch.py
```

### Step 6 — Launch Frontier Coordinator (in its own terminal)

```bash
ros2 run ausra_global_explorer frontier_coordinator \
  --ros-args \
  -p robot_names:="ausra_1,ausra_2" \
  -p map_topic:="<ACTUAL_MERGED_MAP_TOPIC>" \
  -p planning_rate_hz:=0.5 \
  -p min_frontier_cells:=5 \
  -p progress_timeout_s:=30.0 \
  -p use_sim_time:=true
```

Replace `<ACTUAL_MERGED_MAP_TOPIC>` with whatever topic you found in Step A.

### Step 7 — RViz2 for monitoring

```bash
ros2 run rviz2 rviz2
```

Add these displays:
- **Map** → topic: `<merged_map_topic>` — shows the fused map
- **MarkerArray** → topic: `/frontier_coordinator/frontiers` — blue = available frontier
- **TF** — shows robot positions in global frame

---

## Verification Checklist

Run these to confirm each piece is working before starting exploration:

```bash
# 1. Merged map is publishing:
ros2 topic hz <merged_map_topic>
# Expected: ~1 Hz

# 2. TF chain resolves for each robot:
ros2 run tf2_ros tf2_echo map ausra_1_robot_footprint
ros2 run tf2_ros tf2_echo map ausra_2_robot_footprint
# Expected: position near the spawn point (0,0) and (1.5,0)

# 3. Nav2 action servers are running:
ros2 action list | grep navigate_to_pose
# Expected: /ausra_1/navigate_to_pose and /ausra_2/navigate_to_pose

# 4. Coordinator is detecting frontiers (read its logs):
ros2 node info /frontier_coordinator
# Check for log output about frontiers found and assignments

# 5. Robots are receiving goals (watch assignments):
# Look for coordinator log lines like:
# [ASSIGN] ausra_1 → frontier (2.3, 1.1) | size=45 cells | dist=2.5m
# [ausra_1] Goal ACCEPTED → (2.3, 1.1)
```

---

## What to Report Back

After inspecting the simulation workspace, report:

1. The **actual merged map topic name** (e.g., `/map_merged`, `/merged_map`)
2. The **actual robot base frame** format (e.g., `ausra_1_robot_footprint`, `ausra_1/base_footprint`)
3. The **Nav2 action server path** per robot
4. Where **explore_lite** is launched (file path + line) so it can be disabled
5. The **exact robot and map_merge launch commands** currently working in simulation

With those 5 facts, the adaptation is a single line change in `frontier_coordinator.py`
(the `base_frame` line) and disabling explore_lite in the robot launch file.
