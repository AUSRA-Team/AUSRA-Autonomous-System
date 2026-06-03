# Laptop ↔ Jetson Test Run Guide — AUSRA Comms + Map Merge

Complete step-by-step guide to test the comms and map merge pipeline between one Jetson (ausra_1) and the Laptop (base station + fake ausra_2).

---

## Overview

| Machine | Workspace | Role | What it runs |
|---------|-----------|------|-------------|
| **Jetson** | `~/ausra_NM_ws/` | ausra_1 (real SLAM) | `hardware_full_stack.launch.py` + `relay_node` |
| **Laptop** | `~/ausra_ws/` | Base station + fake ausra_2 | Fake publisher + map merge + RViz2 |

> **Important:** The Jetson does **NOT** have a `~/ausra_ws/` folder. Its workspace is `~/ausra_NM_ws/` only.

### What You Need to Copy to the Jetson

Only the `ausra_comms` package needs to be copied. Everything else (`lidar_slam_pkg`, `ausra_map_merge_HW`, drivers, etc.) is already on the Jetson.

```bash
# From the laptop, SCP the package to the Jetson:
scp -r ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms \
       user@<JETSON_IP>:~/ausra_NM_ws/src/AUSRA-Autonomous-System/ausra_comms
```

Then on the Jetson, build it:
```bash
cd ~/ausra_NM_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ausra_comms
source install/setup.bash
```

That's it — copy, build, run.

### Launch Files on the Jetson

| Launch File | Package | Purpose |
|-------------|---------|---------|
| `hardware_full_stack.launch.py` | `lidar_slam_pkg` | **Already exists** — starts drivers, EKF, SLAM, Nav2, exploration |
| `hardware_with_comms.launch.py` | `ausra_comms` | **The one you run** — calls `hardware_full_stack` + adds relay_node (10s delay) |
| `robot_comms.launch.py` | `ausra_comms` | Standalone relay_node only (if hardware stack is already running separately) |

**The final launch command on the Jetson is:**
```bash
ros2 launch ausra_comms hardware_with_comms.launch.py robot_name:=ausra_1
```

This internally calls `hardware_full_stack.launch.py` from `lidar_slam_pkg`, waits 10 seconds, then starts `relay_node` to throttle and relay `/ausra_1/map`, `/ausra_1/pose`, and `/ausra_1/heartbeat` over DDS to the laptop.

### Data Flow

```
[ JETSON — ausra_1 ]                             [ WIFI (DDS) ]    [ LAPTOP — Base Station ]

SLAM ──► /ausra_1/map  ──► relay_node ──(throttle)──► /ausra_1/map  ═══►  map_expansion_node ──► /ausra_1/map_fixed ──┐
     ──► /ausra_1/pose ──► relay_node ──────────────► /ausra_1/pose ═══►                                               │
                            relay_node (1Hz) ────────► /ausra_1/hb   ═══►                                               │
                                                                                                                         │
                                                              [ LOCAL ON LAPTOP ]                                        │
                                                              fake_robot_pub ──► /ausra_2/map  ──► exp_node ──► /ausra_2/map_fixed ──┼──► map_merge ──► /map_merged
                                                                             ──► /ausra_2/hb                                         │
                                                                                                                                      │
                                                              phantom_node ────────────────────► /ausra_99/map_fixed ────────────────┘
```

---

## Prerequisites

### On Jetson — ROS2 Dependencies

These should already be installed. Verify with `ros2 pkg list | grep slam_toolbox`.

```bash
sudo apt update
sudo apt install -y \
  ros-humble-nav-msgs \
  ros-humble-geometry-msgs \
  ros-humble-std-msgs \
  ros-humble-slam-toolbox
```

### On Laptop — ROS2 Dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-nav-msgs \
  ros-humble-geometry-msgs \
  ros-humble-std-msgs \
  ros-humble-slam-toolbox
```

### On Laptop — `multirobot_map_merge` (Built From Source)

> **`ros-humble-multirobot-map-merge` does NOT exist as an apt package.** It must be built from source using the `m-explore-ros2` repository that is already in your laptop workspace.

```bash
cd ~/ausra_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select multirobot_map_merge
source install/setup.bash
```

The source is at `~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/m-explore-ros2/map_merge/`. It was moved there in the previous step. If it's not found, clone it:

```bash
cd ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2
git clone https://github.com/robo-friends/m-explore-ros2.git
cd ~/ausra_ws
colcon build --packages-select multirobot_map_merge
source install/setup.bash
```

### Network — Same WiFi

Both machines **must** be on the **same WiFi network** (same subnet). They must be able to ping each other.

---

## Step 0 — Get IP Addresses (One Time)

**On Jetson:**
```bash
ip addr show wlan0 | grep "inet "
```
→ Note the IP (e.g., `192.168.1.50`)

**On Laptop:**
```bash
ip addr show wlan0 | grep "inet "
```
→ Note the IP (e.g., `192.168.1.100`)

---

## Step 1 — Build on Both Machines

### Jetson

The Jetson workspace is at `~/ausra_NM_ws/` (there is **no** `~/ausra_ws/` on the Jetson).

```bash
cd ~/ausra_NM_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ausra_comms
source install/setup.bash
```

> **Note:** `lidar_slam_pkg`, `ausra_map_merge_HW`, drivers, etc. should already be built. If not:
> ```bash
> colcon build
> source install/setup.bash
> ```

### Laptop

The laptop workspace is at `~/ausra_ws/`.

```bash
cd ~/ausra_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ausra_comms ausra_map_merge_HW multirobot_map_merge
source install/setup.bash
```

---

## Step 2 — Edit Placeholder IPs (One Time Only)

### On Jetson — edit `start_comms_2robots.sh`

```bash
nano ~/ausra_NM_ws/src/AUSRA-Autonomous-System/ausra_comms/scripts/start_comms_2robots.sh
```

Replace `INSERT_LAPTOP_IP_HERE` with the laptop's WiFi IP.

### On Laptop — edit `start_base_2robots.sh`

```bash
nano ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms/scripts/start_base_2robots.sh
```

Replace `INSERT_JETSON_IP_HERE` with the Jetson's WiFi IP.

Then rebuild on both machines so the edited scripts get installed:
```bash
# On Jetson:
cd ~/ausra_NM_ws && colcon build --packages-select ausra_comms && source install/setup.bash

# On Laptop:
cd ~/ausra_ws && colcon build --packages-select ausra_comms && source install/setup.bash
```

---

## Step 3 — Run (In Order)

### 3A — Jetson: Start Hardware + Comms

Open **Terminal 1** on Jetson:
```bash
cd ~/ausra_NM_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 launch ausra_comms hardware_with_comms.launch.py robot_name:=ausra_1 x:=0.0 y:=0.0 yaw:=0.0
```

This starts (in stages):
- **Stage 0** (immediate): Drivers — LiDAR, omni wheels, robot_state_publisher, MPU6050
- **Stage 1** (5s delay): EKF + SLAM Toolbox
- **Stage 2** (15s delay): Nav2 Navigation
- **Stage 3** (30s delay): Frontier Exploration
- **relay_node** (10s delay): `/map` → `/ausra_1/map` (throttled), `/pose` → `/ausra_1/pose`, `/ausra_1/heartbeat` at 1 Hz

Wait until you see:
```
>>> Starting relay_node (AUSRA comms layer)...
[relay_node]: Relay active → ausra_1 | map throttle every 5.0s
```

> **Alternative:** If `hardware_full_stack` is already running in another terminal, you can start just the relay:
> ```bash
> ros2 launch ausra_comms robot_comms.launch.py robot_name:=ausra_1
> ```

### 3B — Laptop: Start Base Station

Open **Terminal 1** on Laptop:
```bash
cd ~/ausra_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

cd src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms/scripts
chmod +x start_base_2robots.sh
./start_base_2robots.sh
```

This starts:
- **Fake ausra_2** publisher (pose + heartbeat + map)
- **Map merge** (AUSRA architecture with map_expansion_nodes + phantom node)
- **RViz2**

---

## Step 4 — Verify

### Quick Check (Laptop Terminal 2)

```bash
source /opt/ros/humble/setup.bash
source ~/ausra_ws/install/setup.bash
export ROS_DOMAIN_ID=0

# 1. Does ausra_1 heartbeat arrive from Jetson?
ros2 topic echo /ausra_1/heartbeat --once
# Expected: "ausra_1 alive"

# 2. Does ausra_2 heartbeat exist locally?
ros2 topic echo /ausra_2/heartbeat --once
# Expected: "ausra_2 alive"

# 3. List all robot topics
ros2 topic list | grep ausra
# Expected:
#   /ausra_1/heartbeat   ← real Jetson via DDS
#   /ausra_1/map         ← real SLAM via DDS
#   /ausra_1/pose        ← real pose via DDS
#   /ausra_2/heartbeat   ← fake publisher (local)
#   /ausra_2/map         ← fake publisher (local)
#   /ausra_2/pose        ← fake publisher (local)

# 4. Check map_fixed topics (from expansion nodes)
ros2 topic list | grep map_fixed
# Expected:
#   /ausra_1/map_fixed
#   /ausra_2/map_fixed
#   /ausra_99/map_fixed  (phantom)

# 5. Check merged map
ros2 topic echo /map_merged --no-arr --once
# Expected: OccupancyGrid with frame_id='map'
```

### RViz2 Setup

1. Set **Fixed Frame** to `map`
2. **Add → By topic → /map_merged → Map**
3. **Add → By topic → /ausra_1/map → Map** (set Color Scheme to `costmap`, Alpha to 0.5)
4. **Add → By topic → /ausra_2/map → Map** (set Alpha to 0.3)

---

## Laptop-Only Test (No Jetson Needed)

If you want to test the pipeline without any Jetson at all:

```bash
cd ~/ausra_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

cd src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms/scripts
chmod +x start_single_robot_test.sh
./start_single_robot_test.sh
```

This launches two fake publishers (ausra_1 + ausra_2), map merge, and verifies all topics automatically.

---

## Troubleshooting

### `/ausra_1/heartbeat` doesn't appear on Laptop

DDS discovery is failing. Check:

```bash
# On BOTH machines, verify:
echo $ROS_DOMAIN_ID      # Must be 0
echo $ROS_LOCALHOST_ONLY  # Must be 0

# Ping test (from laptop)
ping <JETSON_IP>  # Must succeed
```

If ping works but topics don't appear:
- Your router may be blocking multicast. Try a mobile hotspot instead.
- Check if firewall is blocking UDP: `sudo ufw status`

### `/ausra_1/map` topic exists but no data

The relay_node's QoS must match slam_toolbox. Check:
```bash
# On Jetson:
ros2 topic info /ausra_1/map -v
```
Should show `Reliability: Reliable` and `Durability: Transient local`.

### `map_expansion_node` shows RESOLUTION MISMATCH

Your SLAM config uses `resolution: 0.05` — this must match the canvas resolution in `map_merge.launch.py` (currently set to `0.05`). If you changed SLAM resolution, update `CANVAS_RESOLUTION` in the launch file.

### `/map_merged` not appearing

- `map_merge` needs at least 2 maps discovered. Check that both `/ausra_1/map_fixed` and `/ausra_2/map_fixed` topics exist.
- Check map_merge logs for: `adding robot [/ausra_1] to system`
- If you see `Couldn't get initial position for robot [/ausra_1]` → the init_pose parameters are missing. Check `map_merge_swarm_params.yaml`.

### `colcon build` can't find `multirobot_map_merge`

This package is **NOT available via apt**. It must be built from source:
```bash
cd ~/ausra_ws
colcon build --packages-select multirobot_map_merge
```
The source is in `AUSRA-Autonomous-System-hardware_with_nav2/m-explore-ros2/map_merge/`.

---

## Files Summary

### Jetson (`~/ausra_NM_ws/`)

| File | Package | Purpose |
|------|---------|---------|
| `launch/hardware_full_stack.launch.py` | `lidar_slam_pkg` | Drivers, EKF, SLAM, Nav2, exploration (already exists) |
| `launch/hardware_with_comms.launch.py` | `ausra_comms` | **THE ONE YOU RUN** — wraps hardware_full_stack + relay_node |
| `launch/robot_comms.launch.py` | `ausra_comms` | Standalone relay_node (if hardware already running) |
| `ausra_comms/relay_node.py` | `ausra_comms` | Throttles map, relays pose, publishes heartbeat |
| `scripts/start_comms_2robots.sh` | `ausra_comms` | Convenience script for 2-robot mode |

### Laptop (`~/ausra_ws/`)

| File | Package | Purpose |
|------|---------|---------|
| `launch/base_station_comms.launch.py` | `ausra_comms` | Map merge + RViz2 |
| `launch/map_merge.launch.py` | `ausra_comms` | Expansion nodes + merge engine |
| `ausra_comms/fake_robot_pub.py` | `ausra_comms` | Fake data for testing |
| `config/map_merge_swarm_params.yaml` | `ausra_comms` | Merge params (ausra_ namespace) |
| `scripts/start_base_2robots.sh` | `ausra_comms` | Convenience: fake ausra_2 + merge + RViz2 |
| `scripts/start_single_robot_test.sh` | `ausra_comms` | Full local test (no Jetson) |

---

## Quick Reference — Copy-Paste Commands

### Jetson Setup (First Time)
```bash
# 1. Copy ausra_comms to Jetson (run from laptop):
scp -r ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms \
       user@<JETSON_IP>:~/ausra_NM_ws/src/AUSRA-Autonomous-System/

# 2. On Jetson — build:
cd ~/ausra_NM_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ausra_comms
source install/setup.bash
```

### Jetson Run
```bash
cd ~/ausra_NM_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 launch ausra_comms hardware_with_comms.launch.py robot_name:=ausra_1
```

### Laptop Run
```bash
cd ~/ausra_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
cd src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms/scripts
./start_base_2robots.sh
```
