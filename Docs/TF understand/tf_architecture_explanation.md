# Multi-Robot TF Architecture and Coordinate Flow

## Overview

In the AUSRA multi-robot swarm architecture, managing the Transform (TF) tree effectively is critical. Because multiple robots operate in the same simulation and navigate independently, the system uses an isolated but anchored coordinate hierarchy. This guarantees that each robot's SLAM algorithm can build maps without interfering with other robots, while still allowing the entire fleet to share a single global perspective.

## 1. Tracing the Launch Arguments to the TF Tree

When a robot is launched via `spawn_ausra_full.launch.py`, it accepts the initial spawn coordinates: `x`, `y`, and `yaw`. These arguments define where the robot physically appears in the Gazebo simulation.

To translate this physical spawn location into the ROS 2 TF tree, the launch file passes these exact coordinates into a `tf2_ros` `static_transform_publisher` node (`map_offset_node`). This node acts as the foundational glue, continuously broadcasting a **Static Transform** from the global `map` frame to the robot's dedicated `ausra_X_map` frame.

## 2. Coordinate Relationship Hierarchy

The coordinate flow forms an unbroken chain from the global simulation environment down to the physical robot chassis.

*   **`map` (Global Canvas):** The absolute origin `(0,0,0)` of the Gazebo simulation. All robots ultimately tie back to this single anchor point.
*   **`ausra_X_map` (Local Map Frame):** The localized map frame assigned exclusively to Robot X. It is statically offset from the global `map` based on the initial spawn coordinates.
*   **`ausra_X_odom` (Odometry Frame):** The smooth, short-term reference frame. The transform from `ausra_X_map` $\rightarrow$ `ausra_X_odom` is dynamically published by **SLAM Toolbox**, which corrects long-term drift by matching lidar scans against the local map.
*   **`ausra_X_robot_footprint` (and `ausra_X_base_link`):** The physical representation of the robot in space. The transform from `ausra_X_odom` $\rightarrow$ `ausra_X_robot_footprint` is dynamically published by the **EKF (`robot_localization`)**, which fuses wheel odometry and IMU data to track the robot's immediate movement.

## 3. Why the Map Starts at the Spawn Position

SLAM Toolbox is inherently designed to initialize its mapping process at the coordinate `(0,0,0)` relative to its assigned `map_frame`. 

If multiple robots were configured to publish their SLAM corrections directly to the shared global `map` frame, the algorithms would erroneously assume they all started at the exact same location `(0,0,0)`. Their conflicting TF publications would cause severe map tearing and corrupt the system's spatial awareness.

To solve this, the architecture employs **map isolation**. Each robot executes SLAM within its own local `ausra_X_map` frame, freely starting at `(0,0,0)` as the algorithm expects. Because `ausra_X_map` is statically attached to the global `map` using the robot's true physical spawn coordinates, the locally generated map and odometry data are automatically offset. This perfectly projects the robot's local perspective into its true location on the global canvas, aligning the entire fleet for downstream processes like `multirobot_map_merge`.

## 4. TF Tree Diagram

```mermaid
graph TD
    Map["map<br>(Global Gazebo Origin)"] -->|Static Transform<br>Spawn x, y, yaw| LocalMap["ausra_X_map<br>(Local Map Frame)"]
    LocalMap -->|SLAM Toolbox<br>Drift Correction| Odom["ausra_X_odom<br>(Odometry Frame)"]
    Odom -->|EKF (robot_localization)<br>Fused Wheel + IMU| Footprint["ausra_X_robot_footprint<br>(Robot Footprint)"]
    Footprint -->|Static Offset| BaseLink["ausra_X_base_link<br>(Robot Chassis)"]
    BaseLink --> Sensors["Sensors (Lidar, IMU, Camera, Wheels)"]
```

## 5. Spawn & Map Initialization Mechanics

1. **The Spawn:**
   The physical placement of the robot in the simulation is handled in `spawn_ausra_full.launch.py` by the `spawn_entity` node (using `gazebo_ros/spawn_entity.py`). It consumes the launch arguments (`x`, `y`, `yaw`) and passes them directly to Gazebo via the `-x`, `-y`, and `-Y` flags.

2. **The TF Anchor:**
   The static offset between the global `map` and the local `ausra_X_map` is published in `spawn_ausra_full.launch.py` by the `map_offset_node`. This node runs `tf2_ros/static_transform_publisher` and broadcasts the exact `x`, `y`, `yaw` spawn coordinates as a static TF transform.

3. **The Map Start:**
   By default, SLAM Toolbox initializes its internal map origin at `(0, 0, 0)`. In `slam_multirobot.yaml`, the `map_frame` parameter is set to `<robot_namespace>_map` (e.g., `ausra_1_map`). Because `ausra_X_map` is statically offset from the global `map` by the spawn coordinates, SLAM Toolbox maps locally starting from `(0, 0, 0)`, while the global TF tree automatically shifts this map to the correct real-world coordinates on the shared canvas.
