# Swarm TF Tree Architecture Summary

This document outlines the live TF (Transform) tree structure for the AUSRA multi-robot swarm, verified via `view_frames`.

## Global Anchor
* **`map`**: The global origin `(0,0,0)` of the Gazebo simulation. All robots are anchored to this single frame.

## Robot 1 Coordinate Branch (Unbroken)
1. **`map` -> `ausra_1_map`**: Static transform. Defines the exact launch spawn offset (`x, y, yaw`) relative to the global world.
2. **`ausra_1_map` -> `ausra_1_odom`**: Dynamic transform published by SLAM Toolbox. Corrects odometry drift by comparing lidar scans to the local map.
3. **`ausra_1_odom` -> `ausra_1_robot_footprint`**: Dynamic transform published by the `robot_localization` EKF node. Fuses raw wheel odometry and IMU data.
4. **`ausra_1_robot_footprint` -> `ausra_1_base_link`**: Static offset elevating the footprint to the physical center of the robot chassis.
5. **`ausra_1_base_link` -> Sensors**: Branches out to individual hardware links (`ausra_1_lidar`, `ausra_1_imu_link`, `ausra_1_oak_camera`, and all 4 wheels).

## Robot 2 Coordinate Branch (Unbroken)
* Mirrors Robot 1 exactly, operating entirely within the `ausra_2` namespace.
* Chain: `map` -> `ausra_2_map` -> `ausra_2_odom` -> `ausra_2_robot_footprint` -> `ausra_2_base_link`.

## Architectural Conclusion
The TF tree is mathematically sound. There are no detached frames, and both robots correctly trace their physical sensors all the way back to the single global `map`. The system is fully prepped for `multirobot_map_merge` using `known_init_poses: true`, as the static `map -> ausra_X_map` transforms provide the exact spatial glue needed to align the local occupancy grids.