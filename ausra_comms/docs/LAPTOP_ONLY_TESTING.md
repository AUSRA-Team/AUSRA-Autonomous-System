# Laptop-Only Communication Test (Mode B)

This guide walks you through testing the swarm communication and map merging pipeline **on a single laptop**, without requiring any Jetsons, WiFi networks, or physical hardware.

It uses two fake robot publishers and `multirobot_map_merge` to simulate the full pipeline locally.

---

## 1. Initial Setup

Run these steps once on your laptop to prepare the workspace.

### 1.1 Install ROS2 Dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-nav-msgs \
  ros-humble-geometry-msgs \
  ros-humble-std-msgs \
  ros-humble-slam-toolbox
```

### 1.2 Clone and Build m-explore-ros2 (map merge)

```bash
cd ~/ausra_ws/src
git clone https://github.com/robo-friends/m-explore-ros2.git
```

### 1.3 Build the Workspace

```bash
cd ~/ausra_ws
colcon build --packages-select multirobot_map_merge ausra_comms
source install/setup.bash
```

Verify zero errors before continuing.

### 1.4 Make the test script executable

```bash
chmod +x ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms/scripts/start_single_robot_test.sh
```

---

## 2. Run the Test

The test script launches everything you need:
- **Fake ausra_1 publisher** — publishes `/ausra_1/pose` (10 Hz), `/ausra_1/heartbeat` (1 Hz), `/ausra_1/map` (every 10s)
- **Fake ausra_2 publisher** — publishes `/ausra_2/pose` (10 Hz), `/ausra_2/heartbeat` (1 Hz), `/ausra_2/map` (every 10s)
- **map_merge node** — discovers `/ausra_1/map` and `/ausra_2/map`, merges them into `/map_merged`

The bridge is **not used** in this mode — all topics are local, so no network layer is needed.

```bash
cd ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms/scripts
./start_single_robot_test.sh
```

### What to Expect

The script will:
1. Start both fake publishers
2. Start `map_merge`
3. Verify all 6 robot topics appear (✓ or ✗ for each)
4. Wait ~12 seconds for the first maps to publish
5. Check if `/map_merged` appears

Leave this terminal running. Press `Ctrl+C` to cleanly shut everything down.

---

## 3. Manual Verification

Open a **new terminal** while the test is running:

```bash
source /opt/ros/humble/setup.bash
source ~/ausra_ws/install/setup.bash
export ROS_DOMAIN_ID=0
```

### Check heartbeats (should print immediately)

```bash
ros2 topic echo /ausra_1/heartbeat
```

Expected output:
```
data: 'ausra_1 alive'
---
data: 'ausra_1 alive'
```

### Check pose rate

```bash
ros2 topic hz /ausra_1/pose
```

Expected output: `average rate: ~10.0`

### Check map topics exist

```bash
ros2 topic list | grep ausra
```

Expected:
```
/ausra_1/heartbeat
/ausra_1/map
/ausra_1/pose
/ausra_2/heartbeat
/ausra_2/map
/ausra_2/pose
```

### Check global merged map (wait at least 15 seconds after start)

```bash
ros2 topic echo /map_merged --no-arr
```

Expected: OccupancyGrid message headers with `frame_id: map` and non-zero width/height.

### Visualize in RViz2

```bash
ros2 run rviz2 rviz2
```

In RViz2:
1. Set **Fixed Frame** to `map`
2. **Add → By topic → /map_merged → Map**
3. **Add → By topic → /ausra_1/map → Map** (set Alpha to 0.4)
4. **Add → By topic → /ausra_2/map → Map** (set Alpha to 0.4)

---

## 4. How map_merge Discovers Robots

Understanding this prevents most debugging headaches:

1. `map_merge` calls `get_topic_names_and_types()` periodically (every `1/discovery_rate` seconds)
2. For each topic, it checks:
   - Is the type `nav_msgs/msg/OccupancyGrid`?
   - Does the topic name **contain** `robot_namespace` ("robot")?
   - Does the topic name **end with** `robot_map_topic` ("map")?
3. If all three match (e.g., `/ausra_1/map` ✓), it extracts the namespace (`/ausra_1`)
4. When `known_init_poses=True`, it looks for parameters **on the map_merge node itself**:
   - `/ausra_1.map_merge.init_pose_x`
   - `/ausra_1.map_merge.init_pose_y`
   - `/ausra_1.map_merge.init_pose_z`
   - `/ausra_1.map_merge.init_pose_yaw`
5. If any init_pose parameter is missing, **that robot is silently skipped** (only a WARN log)
6. The merged map is published to `merged_map_topic` (set to `/map_merged`)

These init_pose parameters are already configured in `map_merge.launch.py` for robots 1, 2, and 3.

---

## 5. Troubleshooting

### `/map_merged` not appearing

**Most likely cause:** `map_merge` hasn't discovered the robot maps yet, or the init_pose params are missing.

Check map_merge logs for these lines:
```
[map_merge]: Robot discovery started.
[map_merge]: adding robot [/ausra_1] to system
[map_merge]: Subscribing to MAP topic: /ausra_1/map.
[map_merge]: adding robot [/ausra_2] to system
[map_merge]: Subscribing to MAP topic: /ausra_2/map.
```

If you see this warning instead:
```
Couldn't get initial position for robot [/ausra_1]
```
→ The init_pose parameters are missing. Check that `map_merge.launch.py` includes them.

### Maps are discovered but merge produces nothing

Check that the fake publishers are using `transient_local` + `reliable` QoS for the map topic. The `map_merge` node subscribes with this QoS:
```python
rclcpp::QoS(rclcpp::KeepLast(50)).transient_local().reliable()
```

If the publisher uses a different QoS, the subscriber won't receive messages even though the topic exists.

### Check map_merge parameters at runtime

```bash
ros2 param list /map_merge
```

You should see the init_pose params listed:
```
/ausra_1.map_merge.init_pose_x
/ausra_1.map_merge.init_pose_y
...
```
