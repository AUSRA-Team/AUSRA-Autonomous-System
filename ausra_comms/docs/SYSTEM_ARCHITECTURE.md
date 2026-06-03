# Swarm System Architecture & Data Flow

This document explains exactly how the different components of the swarm system talk to each other, how the test modes work, and how the files in `ausra_comms` orchestrate everything.

---

## 1. The Core Concept: How ROS 2 DDS Replaces the "Bridge"

In ROS 1, if you had a Jetson and a Laptop, you needed a "bridge" or a "master node" to forward messages over the network. 

In **ROS 2**, the network layer (Data Distribution Service, or DDS) handles this natively. If two machines meet these three conditions:
1. Connected to the **same WiFi network** (same subnet)
2. Using the **same `ROS_DOMAIN_ID`** (we use `0`)
3. Multicast is allowed on the router

...then **they automatically see each other's topics**. If Jetson publishes `/ausra_1/map`, the Laptop can subscribe to `/ausra_1/map` instantly, just as if it were running locally. 

Because of this, **we deleted the `swarm_bridge` entirely.** It was an overcomplicated ROS 1 relic. The system now relies on native, robust ROS 2 communication.

---

## 2. Laptop-Only Test Mode (Simulation)

This mode verifies that the data processing pipeline (specifically map merging) works without needing physical hardware.

**What you run:** `./start_single_robot_test.sh`

### Data Flow Diagram:
```text
[ LAPTOP - ALL LOCAL ]

fake_robot_pub (ID:1) ─── publishes ───> /ausra_1/map (50x50 grid)
                                   ───> /ausra_1/pose
                                   ───> /ausra_1/heartbeat

fake_robot_pub (ID:2) ─── publishes ───> /ausra_2/map (50x50 grid)
                                   ───> /ausra_2/pose
                                   ───> /ausra_2/heartbeat

map_merge.launch.py:
  │
  ├── map_expansion_node (1) <── subscribes ── /ausra_1/map
  │   └── stamps onto 1000x1000 canvas ──> /ausra_1/map_fixed
  │
  ├── map_expansion_node (2) <── subscribes ── /ausra_2/map
  │   └── stamps onto 1000x1000 canvas ──> /ausra_2/map_fixed
  │
  ├── map_expansion_node (phantom) ────────> /ausra_99/map_fixed (blank, prevents crash)
  │
  └── multirobot_map_merge   <── subscribes ── /ausra_1/map_fixed, /ausra_2/map_fixed
      └── merges canvases together ──────> /map_merged
```
**Why this matters:** It proves that if the Laptop receives topics named `/robotX/map`, the map merging pipeline (`map_expansion_node` + `multirobot_map_merge`) successfully processes them into a `/map_merged`.

---

## 3. Mode A: Hardware Jetson + Laptop Base Station

This is how the system runs with real robots. The heavy SLAM processing happens on the Jetson, but the map merging and visualization happen on the Laptop.

### Jetson Side (Hardware)
**What you run:** `ros2 launch ausra_comms hardware_with_comms.launch.py robot_name:=ausra_1`

1. The launch file starts the **AUSRA Hardware Stack** (`hardware_full_stack.launch.py`). This runs the LiDAR, EKF, and SLAM Toolbox.
2. SLAM Toolbox publishes the real map to the global topic `/map` and the pose to `/pose`.
3. Ten seconds later, the launch file starts the **`relay_node.py`**.
4. The **`relay_node`** is the translator between the local robot and the swarm:
   - It subscribes to `/map`.
   - It publishes `/ausra_1/map` (and throttles it to 1 message every 5 seconds to save WiFi bandwidth).
   - It subscribes to `/pose` and publishes `/ausra_1/pose`.
   - It publishes `/ausra_1/heartbeat` at 1 Hz.

### The Network (ROS 2 DDS)
The topics `/ausra_1/map`, `/ausra_1/pose`, and `/ausra_1/heartbeat` are automatically broadcast over the WiFi network because `ROS_DOMAIN_ID=0`.

### Laptop Side (Base Station)
**What you run:** `./start_base_2robots.sh`

1. The script starts a **fake ausra_2** locally (just so `map_merge` has two maps to merge, otherwise it crashes).
2. It launches **`map_merge.launch.py`**.
3. Because of DDS, the Laptop automatically discovers `/ausra_1/map` coming through the WiFi from the Jetson.
4. The **`map_expansion_node`** picks up `/ausra_1/map`, applies the physical spawn offset, and outputs `/ausra_1/map_fixed`.
5. **`multirobot_map_merge`** merges the real Jetson map and the fake local map into `/map_merged`.
6. **RViz2** displays the result.

### Data Flow Diagram:
```text
[ JETSON 1 ]                                            [ WIFI (DDS) ]        [ LAPTOP BASE STATION ]

SLAM Toolbox ──> /map  ──> relay_node ──(throttle)──> /ausra_1/map  ======>  map_expansion_node(1) ──> /ausra_1/map_fixed ──┐
             ──> /pose ──> relay_node ──────────────> /ausra_1/pose ======>                                                │
                           relay_node (1Hz) ────────> /ausra_1/hb   ======>                                                │
                                                                                                                          │
                                                                       [ LOCAL ON LAPTOP ]                                │
                                                                       fake_robot_pub(2) ──> /ausra_2/map  ──> exp_node ──> /ausra_2/map_fixed ──┼──> multirobot_map_merge ──> /map_merged
                                                                                         ──> /ausra_2/hb                                    │
                                                                                                                                           │
                                                                       phantom_node ────────────────────────> /ausra_99/map_fixed ──────────┘
```

---

## 4. Why Your Teammates Shouldn't Manually Rename Topics

Your teammates are currently trying to manually rename every topic in the AUSRA hardware stack to `ausra1/cmd_vel`, `ausra1/scan`, etc. 

**This is a bad idea because:**
1. It requires editing dozens of config files, URDFs, and launch files.
2. It's highly prone to errors (mismatched frame IDs will break TF).
3. The codebase becomes hardcoded to a specific robot.

### The Correct Way: ROS 2 Namespaces
ROS 2 has a built-in feature to solve this instantly without changing a single line of code in the hardware packages. You simply launch the existing package inside a namespace:

```bash
# This automatically prefixes EVERY topic and node with /ausra_1/
ros2 launch lidar_slam_pkg hardware_full_stack.launch.py --ros-args --namespace ausra_1
```

If you do this, `/cmd_vel` instantly becomes `/ausra_1/cmd_vel`, and `/map` becomes `/ausra_1/map`. 

**Why we are using `relay_node` for now:**
Currently, we want to isolate the Jetson's heavy internal topics (`/scan`, `/odom`, TF data) from the WiFi network to prevent flooding the network bandwidth. 
- If we put the whole robot in a namespace, DDS will try to broadcast `/ausra_1/scan` (huge LiDAR data) over WiFi.
- By using `relay_node`, the Jetson keeps `/scan` local, and ONLY broadcasts the throttled `/ausra_1/map` and `/ausra_1/pose` to the swarm.

---

## 5. Overview of the `ausra_comms` Files

Here is exactly what each file in your new package does:

### Core Nodes (`ausra_comms/`)
* **`fake_robot_pub.py`**: Generates fake grid maps and poses for testing.
* **`relay_node.py`**: Subscribes to local SLAM topics and republishes them with a `robotX/` prefix and throttled rates to save WiFi bandwidth.

### Launch Files (`launch/`)
* **`hardware_with_comms.launch.py`**: The master file for the Jetson. Starts the hardware stack, waits 10s, then starts the `relay_node`.
* **`map_merge.launch.py`**: The master file for the Laptop. Implements the AUSRA architecture: wraps incoming maps in `map_expansion_node`s, adds a phantom node to prevent crashes, and starts the actual merge engine.
* **`robot_comms.launch.py` / `base_station_comms.launch.py`**: Utility launch files used by the bash scripts.

### Scripts (`scripts/`)
* **`start_single_robot_test.sh`**: Runs everything on one laptop (fake data).
* **`start_comms_2robots.sh`**: Runs on the Jetson. Pings the laptop, then starts the relay.
* **`start_base_2robots.sh`**: Runs on the Laptop. Starts a fake robot, map merge, and RViz2.

### Config (`config/`)
* **`map_merge_swarm_params.yaml`**: Configuration for the merge engine (rates, topics, and forcing `init_pose` offsets to 0.0 because the expansion nodes handle it).
