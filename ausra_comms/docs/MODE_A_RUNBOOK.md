# Mode A Runbook — 1 Jetson + Laptop (2-Robot Mode)

Jetson runs real SLAM and relays `/ausra_1/*` topics. Laptop runs fake ausra_2 + map merge + RViz2.
Communication happens over native ROS2 DDS — no bridge binary needed.

---

## Before You Start

### 1. Both machines must have the workspace built

**On Jetson:**
```bash
cd ~/ausra_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

**On Laptop:**
```bash
cd ~/ausra_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 2. Both machines must be on the same WiFi network

Connect Jetson and Laptop to the **same router WiFi**.

### 3. Get your IPs

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

### 4. Edit the placeholder IPs (one time only)

**On Jetson** — edit `start_comms_2robots.sh`:
```bash
nano ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms/scripts/start_comms_2robots.sh
```
Replace `INSERT_LAPTOP_IP_HERE` with the laptop's WiFi IP.

**On Laptop** — edit `start_base_2robots.sh`:
```bash
nano ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms/scripts/start_base_2robots.sh
```
Replace `INSERT_JETSON_IP_HERE` with the Jetson's WiFi IP.

---

## Run (in order)

### Step 1 — Jetson: Start Hardware + Comms

Open **Terminal 1** on Jetson:
```bash
source /opt/ros/humble/setup.bash
source ~/ausra_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 launch ausra_comms hardware_with_comms.launch.py robot_name:=ausra_1
```

This starts:
- Drivers (LiDAR, omni wheels, robot_state_publisher) — immediately
- EKF + SLAM — after 5 seconds
- Nav2 — after 15 seconds
- Exploration — after 30 seconds (robot won't move without micro-ROS agent)
- **relay_node** — after 10 seconds (`/map` → `/ausra_1/map`, etc.)

Wait until you see:
```
>>> Starting relay_node (swarm comms layer)...
[relay_node]: Relay active → ausra_1 | map throttle every 5.0s
```

### Step 2 — Laptop: Start Base Station

Open **Terminal 1** on Laptop:
```bash
cd ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms/scripts
./start_base_2robots.sh
```

This starts:
- Fake ausra_2 publisher (pose/heartbeat/map)
- Map merge (AUSRA architecture with map_expansion_nodes)
- RViz2

---

## Verify

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
ros2 topic list | grep robot
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

## Troubleshooting

### `/ausra_1/heartbeat` doesn't appear on laptop

DDS discovery is failing. Check:

```bash
# On BOTH machines, verify:
echo $ROS_DOMAIN_ID      # Must be 0
echo $ROS_LOCALHOST_ONLY  # Must be 0

# Ping test
ping <other_machine_ip>  # Must succeed
```

If ping works but topics don't appear:
- Your router may be blocking multicast. Try a mobile hotspot instead.
- Check if firewall is blocking UDP: `sudo ufw status`

### `/ausra_1/map` topic exists but no data

The relay_node's QoS must match slam_toolbox. Check:
```bash
# On Jetson:
ros2 topic info /map -v    # Check QoS shown
ros2 topic info /ausra_1/map -v
```
Both should show `Reliability: Reliable` and `Durability: Transient local`.

### `map_expansion_node` shows RESOLUTION MISMATCH

Your SLAM config uses `resolution: 0.05` — this must match the canvas resolution in `map_merge.launch.py` (currently set to `0.05`). If you changed SLAM resolution, update `CANVAS_RESOLUTION` in the launch file.

---

## Files to Copy to Jetson

The Jetson needs these packages built:
- `lidar_slam_pkg` (+ its hardware dependencies)
- `ausra_comms`

The Jetson does NOT need:
- `ausra_map_merge_HW` (runs on laptop only)
- `m-explore-ros2` / `multirobot_map_merge` (runs on laptop only)
