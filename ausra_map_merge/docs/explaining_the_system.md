# AUSRA Multi-Robot Map Merge System - Detailed Code Explanation

This document provides a highly detailed, code-level explanation of the `ausra_map_merge` package. This package is responsible for safely and reliably merging multiple dynamic SLAM occupancy grids into a single, globally aligned map without causing segfaults or moving-floor misalignment issues.

## System Architecture Overview

The system consists of three major components:
1. **`map_expansion_node.cpp`**: A dedicated C++ node running per-robot that receives dynamic SLAM grids and embeds them into a statically sized, globally aligned "canvas".
2. **`map_merge.launch.py`**: The orchestration file that instantiates an expansion node for every robot and one central `multirobot_map_merge` node.
3. **`map_merge_params.yaml`**: The configuration for the map merger, ensuring it acts as a pure pixel-overlay tool.

---

## 1. Map Expansion Node (`map_expansion_node.cpp`)

The Map Expansion Node is built on a **Decoupled Publisher/Subscriber** architecture utilizing a heartbeat timer. This directly resolves the `multirobot_map_merge` segmentation faults that occur when merging nodes initialize without map data.

### Core Architecture: The Heartbeat Timer
The node decouples receiving SLAM data from publishing the canvas.
```cpp
// Constructor initialization
const auto period_us = std::chrono::microseconds(
  static_cast<int64_t>(1'000'000.0 / publish_rate_hz));

heartbeat_timer_ = this->create_wall_timer(
  period_us,
  std::bind(&MapExpansionNode::publishCanvas, this));
```
- **`publishCanvas()`**: Fires unconditionally at 1 Hz. It publishes the current state of `canvas_data_`.
- **`mapCallback()`**: Fires whenever `slam_toolbox` publishes. It updates `canvas_data_` in-place but **never publishes**.

### Pre-allocation & Segfault Prevention
In the constructor, before SLAM data even arrives, the canvas is pre-allocated with Unknown cells (-1):
```cpp
const size_t canvas_size = static_cast<size_t>(canvas_width_) * static_cast<size_t>(canvas_height_);
canvas_data_.assign(canvas_size, static_cast<int8_t>(-1));
```
Because the heartbeat timer publishes immediately on launch, the central `map_merge` node always receives a valid, perfectly sized matrix, eliminating the race condition that causes segfaults.

### Fault Tolerance & "Ghost Maps"
If a robot crashes or its SLAM fails, `mapCallback()` stops firing. Because of the decoupled architecture, `publishCanvas()` continues to broadcast the last known state of `canvas_data_`. This ensures the central merged map retains the lost robot's "ghost map" rather than deleting its contribution.

### Spatial Math and Pixel Shifting
To prevent the "moving floor" problem caused by `slam_toolbox` drifting its local origin, the node converts the local SLAM frame to global canvas pixels:

```cpp
// 1. Calculate global coordinates using the robot's physical spawn offset
const double global_origin_x = local_origin_x + robot_offset_x_;
const double global_origin_y = local_origin_y + robot_offset_y_;

// 2. Determine the pixel offset within the fixed canvas
const int offset_x = static_cast<int>(
  std::round((global_origin_x - canvas_origin_x_) / canvas_resolution_));
const int offset_y = static_cast<int>(
  std::round((global_origin_y - canvas_origin_y_) / canvas_resolution_));
```
Since `canvas_origin_x_` is fixed at `-25.0` and `canvas_resolution_` is `0.05`, the resulting `canvas_col` and `canvas_row` indices remain constant for fixed physical walls, fully eliminating map drift.

### Performance Optimization: Partial Reset
Instead of clearing the entire 1,000,000-cell canvas (O(N) operation) on every SLAM update, the node tracks which indices were modified in the *previous* callback and only resets those:
```cpp
for (const int idx : last_written_indices_) {
  canvas_data_[idx] = static_cast<int8_t>(-1);
}
last_written_indices_.clear();
```
This reduces the operation cost from ~10,000–100,000 ops per tick compared to a full array wipe. The node then copies the incoming SLAM data over the freshly cleared bounds and tracks the newly modified indices.

---

## 2. Launch Orchestration (`map_merge.launch.py`)

The launch file constructs the map merging graph by passing the exact Gazebo spawn coordinates to each robot's expansion node.

### Spawn Coordinate Injection
The spawn locations are defined in a dictionary and injected into the expansion nodes as `robot_offset_x` and `robot_offset_y`:
```python
ROBOT_SPAWN_POSES = {
    'ausra_1': {'x': 3.0, 'y': 0.0},
    'ausra_2': {'x': 0.0, 'y': 2.0},
}
```
For each robot, a `map_expansion_node` is launched that subscribes to `/{robot_name}/map` and publishes to `/{robot_name}/map_fixed`. All canvases are identically sized (1000x1000 at 0.05 resolution, origin at -25.0, -25.0).

---

## 3. Configuration (`map_merge_params.yaml`)

Because the `map_expansion_node` already embeds the physical spawn offsets into the fixed canvases, the canvases are strictly pre-aligned. The central `multirobot_map_merge` node must therefore act strictly as a dumb pixel overlay.

To achieve this, the configuration explicitly zeroes out all internal transformation attempts:
```yaml
map_merge:
  ros__parameters:
    known_init_poses: true
    # ...
    /ausra_1/map_merge/init_pose_x: 0.0
    /ausra_1/map_merge/init_pose_y: 0.0
    /ausra_1/map_merge/init_pose_z: 0.0
    /ausra_1/map_merge/init_pose_yaw: 0.0
```
Setting `init_pose` values to anything other than `0.0` would cause the maps to double-shift, breaking the global alignment established by the expansion nodes.
