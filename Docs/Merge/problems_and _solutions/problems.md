# Diagnostic Report: `multirobot_map_merge` Segmentation Faults (Exit Code -11)

## 1. Issue Overview
The `multirobot_map_merge` node is suffering from a fatal crash (Segmentation Fault, `exit code -11`) during the launch sequence under specific conditions. This crash brings down the entire mapping pipeline. 

The crashes occur specifically when the node is launched without the full set of expected robot maps being actively published on the network.

## 2. Crash Triggers
We have identified two exact scenarios that instantly trigger the Segfault:

* **Trigger A (Zero Maps):** Launching the `map_merge` node *before* any robots have been spawned in Gazebo and before SLAM has started.
* **Trigger B (One Map):** Launching the `map_merge` node when only Robot 1 has spawned and started SLAM, but Robot 2 has not yet initialized.

## 3. High-Level Root Cause
The root cause is brittle memory management in the ROS 2 port of the `m-explore-ros2` package. The internal OpenCV compositing pipeline makes unsafe assumptions about the state of the incoming data.

1.  **Missing Null Checks:** When the node boots up, it discovers namespaces but does not verify that the actual `nav_msgs::msg::OccupancyGrid` data structures exist or contain data before passing them into the OpenCV matrix builders.
2.  **Matrix Addition Failure:** The package's core logic (`composeGrids`) is mathematically hardcoded to stitch *multiple* images together. When only one map exists (Trigger B), the pipeline attempts to perform an affine transformation against a `NULL` or missing second matrix.
3.  **The Segfault:** Hitting these null pointers causes the operating system to instantly kill the process to protect memory integrity.

## 4. Current Operational Workaround
To prevent the crash on physical deployments, the swarm must adhere to a strict launch sequence:
1. Spawn all robots.
2. Initialize SLAM for all robots.
3. Ensure the custom `map_expansion_node` is actively publishing 1000x1000 canvases for *all* robots.
4. Only then, launch `map_merge.launch.py`.

## 5. Required Agent Action (Task)
**Agent:** Do not rewrite the external Python launch files. The flaw is internal to the C++ map merger.

Please inspect the source code of `multirobot_map_merge` in our workspace (specifically around the map subscriber callbacks and the OpenCV `composeGrids` logic). 
1. Identify exactly where the code attempts to access array elements or construct `cv::Mat` objects without verifying array bounds or pointers.
2. Propose a C++ patch to add safety guards (e.g., `try/catch` blocks or `if (images.size() < 2)` checks) so the node gracefully idles and waits for the remaining maps instead of crashing.

## 6. Crucial Observation: Steady-State Survival (The Desired Behavior)
While the node crashes if a robot is missing at *launch*, we have observed that if both robots are successfully launched first, the `map_merge` node works perfectly. 

Furthermore, **if Robot 1 dies or crashes AFTER the successful initialization, the `map_merge` node does NOT fail.** It successfully continues operating, continuously taking the new map updates from the surviving Robot 2 and merging them with the last known data of Robot 1. 

**This steady-state fault tolerance is our exact desired behavior for the swarm.** Your C++ patch must only fix the boot-up initialization crash without breaking this excellent decentralized survival behavior!