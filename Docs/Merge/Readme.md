# AUSRA Map Merge Deployment Guide

Welcome to the operational deployment guide for the `ausra_map_merge` package. This document is designed for the field operations team to quickly understand the behavior, configuration, and launch procedures for our fully decentralized, fault-tolerant map merging pipeline. 

---

## 1. High-Level Workflow

Our map merging system utilizes a **"Smart Canvas + Dumb Overlay"** architecture to ensure perfect global alignment across the entire swarm.

* **The Smart Canvas (`map_expansion_node`)**: Instead of relying on complex, error-prone relative offset calculations, we use a custom expansion node. This node takes the raw, dynamically growing SLAM maps from each robot and places them onto a massive, fixed 1000x1000 global canvas. It uses the physical spawn coordinates of each robot to permanently lock their map to the correct location on the canvas. This node continuously updates and publishes these pre-aligned canvases at 1 Hz.
* **The Dumb Overlay (`multirobot_map_merge`)**: Because the canvases are perfectly aligned in global space and are the exact same dimensions, the standard `multirobot_map_merge` node has a very simple job. It simply stacks these transparent canvases on top of each other, effortlessly creating the final, unified global map.

---

## 2. True Decentralized Behavior (Real-Life Operations)

The `ausra_map_merge` package was built with real-world physical swarms in mind. Its behavior is characterized by extreme fault tolerance and decentralization:

* **Launch Order Independence**: Thanks to the built-in heartbeat timer, you can launch the map merger **first**, before any robots are even turned on. The system will patiently wait and begin merging maps as soon as they appear on the network.
* **Fully Decentralized Execution**: In a physical swarm, *every* robot runs this package locally. They all listen to the same network traffic, and they all independently generate the exact same `/map_merged` output. There is no single point of failure (no centralized master server).
* **The Survival Mechanic**: Real-world operations are unpredictable. If a robot's battery dies, its network drops, or its SLAM module crashes, the system will adapt. Its last known map is securely "frozen" on its canvas, and the overall merger continues flawlessly with the surviving robots. When a late-joining robot boots up or reconnects, it will seamlessly integrate into the pipeline without disrupting the swarm.

---

## 3. How to Add a New Robot (The Golden Rules)

When scaling up the swarm (e.g., adding `ausra_3`), the team must strictly adhere to two golden rules to guarantee the system works out-of-the-box:

* **Rule A (The Config)**: In `map_merge_params.yaml`, the new robot **MUST** have all of its `init_pose_x`, `init_pose_y`, and `init_pose_z` values set exactly to `0.0`. Since the `map_expansion_node` handles all spatial shifting, providing non-zero values here will cause catastrophic double-shifting.
* **Rule B (The Launch)**: In `map_merge.launch.py`, the new robot's **EXACT physical spawn coordinates** must be added to the launch dictionary. The expansion node absolutely needs these coordinates to accurately align the robot's map onto the global canvas.

---

## 4. Launch Commands & RViz Visualization

### Launching the Pipeline

To start the map merging system, open a terminal on your machine (or ssh into the robot), source your ROS 2 workspace, and run the standard launch command:

```bash
# Source the ROS 2 workspace
source install/setup.bash

# Launch the map merger
ros2 launch ausra_map_merge map_merge.launch.py
```

```bash
# launch robots
ros2 launch ausra_spawner spawn_ausra_full.launch.py robot_id:=2 x:=0.0 y:=2.0 yaw:=0.0 use_ekf:=true use_slam:=true use_nav2:=true use_exploration:=true

```
### Visualizing in RViz2

Open a new terminal and launch RViz2:

```bash
# Source the ROS 2 workspace
source install/setup.bash

# Run RViz2
rviz2 
```

To monitor the swarm in RViz2, you only need to visualize the raw maps and the final merged output. 

*(Note: The `map_fixed` topics are strictly backend bridge topics used to transport the 1000x1000 canvases into the overlay process. You do **not** need to visualize them.)*

Add the following exact topics as **Map** displays in RViz2:

1. **`/map_merged`** - The final global map showing the combined environment of the whole swarm.
2. **`/ausra_1/map`** - Robot 1's raw, actively growing SLAM map.
3. **`/ausra_2/map`** - Robot 2's raw, actively growing SLAM map.
*(Add subsequent `/ausra_X/map` topics for any additional robots deployed).*
