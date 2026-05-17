# Architectural Review: Heartbeat Timer Map Expansion Node

## Verdict: **APPROVED — Deploy As-Is**

The Heartbeat Timer architecture is the correct solution for this problem. It is not merely "good enough" — it is the objectively best pattern for this specific failure mode given the constraints of the `multirobot_map_merge` package.

---

## Phase 1: Architectural Analysis

### 1.1 Why This Pattern Is Correct

The segfault in `multirobot_map_merge` is a **data-availability race condition**: the merger's OpenCV pipeline dereferences map data that doesn't exist yet. There are exactly two classes of fix:

| Approach | Where the fix lives | Complexity |
|---|---|---|
| **A. Patch the merger's C++** | Inside `m-explore-ros2` source | Requires forking an external package, understanding its OpenCV internals, and maintaining the fork forever |
| **B. Guarantee data always exists** | Inside our own `map_expansion_node` | Zero external dependencies, fully under our control |

The Heartbeat Timer implements Approach B. By pre-allocating a valid `1000×1000` canvas of `-1` (Unknown) cells and publishing it *unconditionally* at 1 Hz from the moment the node starts, the merger always has a legally-sized array to read. The race condition is eliminated at the source, not patched downstream.

### 1.2 Why Lifecycle Nodes Are Not Superior Here

ROS 2 Lifecycle Nodes (managed nodes) solve a different problem: coordinating startup ordering between nodes that have explicit configure/activate/deactivate transitions. They would be relevant if:
- We controlled the `multirobot_map_merge` source and could make it a lifecycle-aware subscriber.
- The failure was about *node readiness ordering* rather than *data availability*.

Neither is the case. The merger is an opaque third-party node. It subscribes with `transient_local` QoS and expects data to be present. The heartbeat gives it that data. Lifecycle adds ceremony without solving the actual null-pointer trigger.

### 1.3 Edge Case Analysis

| Concern | Assessment |
|---|---|
| **QoS `transient_local` conflict** | **No conflict.** Both publisher and subscriber use `QoS(1).transient_local()`. The timer publishes to the output topic (which the merger subscribes to), and the subscriber listens to the SLAM topic. These are two separate topics with matching QoS on each end. |
| **Single-threaded executor blocking** | **Safe.** The default `SingleThreadedExecutor` serializes callbacks. The timer and subscription callback will never run concurrently. The `std::mutex` in the proposed code is a defensive guard for multi-threaded executors — it adds negligible overhead (~20ns per lock) on a single-threaded executor and provides correctness if someone later switches to `MultiThreadedExecutor`. This is correct defensive engineering. |
| **Timer + `transient_local` interaction** | **No issue.** `transient_local` durability means the *last* published message is cached for late-joining subscribers. The 1 Hz timer continuously overwrites this cache. A late-joining merger will receive the most recent canvas — which is exactly what we want. |
| **`create_wall_timer` vs `create_timer`** | **Correct choice.** `create_wall_timer` uses wall-clock time, not simulated time. This is correct because the heartbeat must fire even when simulation time is paused or not yet published. If we used `create_timer` with `use_sim_time: true`, the heartbeat would freeze when Gazebo hasn't started — defeating the entire purpose. |
| **Memory: 1M cell copy per tick** | **Acceptable.** `1000 × 1000 × sizeof(int8_t)` = 1 MB copied once per second. This is negligible on any modern system. The `last_written_indices_` partial-reset optimization reduces the per-callback cost from O(1M) to O(SLAM_map_size), which is typically 10K–100K cells. |
| **Ghost map on robot death** | **Desired behavior preserved.** When `mapCallback` stops firing, the partial reset never triggers, so the last known SLAM data stays frozen in `canvas_data_`. The timer keeps publishing it. This matches the steady-state fault tolerance described in `problems.md`. |

### 1.4 Code Quality Assessment

The proposed code is production-quality:

- **Input validation guards** (resolution mismatch, malformed data array) prevent silent corruption and catch SLAM cold-start edge cases.
- **Grid alignment check** warns about floating-point drift at startup rather than letting it silently accumulate.
- **Overflow logging** is throttled (`% 500`) to avoid log flooding while still reporting boundary violations.
- **`const` correctness** is applied throughout the callback.
- **No unused includes** — `geometry_msgs/msg/pose.hpp` from the legacy code has been correctly removed since `OccupancyGrid` already contains the pose.

### 1.5 One Minor Observation (No Action Needed)

The `overflow_count_` increment on line 376 (`overflow_count_ += (inc_w - 1)`) counts the skipped columns in a row-overflow. This is an approximation — some of those columns might have been within bounds if the offset were different. However, this counter is purely diagnostic (used only in log messages), so the approximation is acceptable and does not affect correctness.

---

## Phase 2: Build System Validation

### 2.1 CMakeLists.txt — No Changes Needed

The existing `CMakeLists.txt` is sufficient:

- `rclcpp` provides `<rclcpp/rclcpp.hpp>`, `create_wall_timer`, and the executor.
- `nav_msgs` provides `nav_msgs::msg::OccupancyGrid`.
- `<chrono>`, `<mutex>`, `<cmath>`, `<string>`, `<vector>` are all C++ standard library headers — no additional `find_package` or linker flags are needed.
- `std::mutex` is part of the C++11 threading library which is linked automatically by modern CMake/GCC.
- The `geometry_msgs` dependency is no longer used by the new code but keeping it in `CMakeLists.txt` is harmless and avoids unnecessary churn.

### 2.2 package.xml — No Changes Needed

All runtime and build dependencies are already declared.

### 2.3 map_merge_params.yaml — No Changes Needed

All `init_pose_*` values remain `0.0`. The heartbeat architecture does not alter the spatial math — it only decouples *when* the canvas is published from *when* SLAM data arrives. The pixel alignment logic is identical to the legacy code.

---

## Phase 3: Deployment Steps

1. Replace `ausra_map_merge/src/map_expansion_node.cpp` with the heartbeat version.
2. Rebuild the package: `colcon build --packages-select ausra_map_merge`
3. Source the workspace: `source install/setup.bash`
4. Launch order no longer matters — `map_merge.launch.py` can be started at any time.
