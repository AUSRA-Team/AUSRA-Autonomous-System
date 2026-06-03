# ausra_comms — Usage Guide

3-robot swarm communication layer over WiFi using `auto_swarm_bridge` and `multirobot_map_merge`.

**Machines:**

| Machine | Role | Hardware |
|---------|------|----------|
| ausra_1 | SLAM + relay + bridge | Jetson Orin Nano |
| ausra_2 | SLAM + relay + bridge | Jetson Orin Nano |
| ausra_3 | SLAM + relay + bridge | Jetson Orin Nano |
| Laptop  | Bridge + map merge + RViz2 | Ubuntu laptop |

---

## 1. One-Time Setup

Run these steps **once on every machine** (all 3 robots + laptop).

### 1.1 — Install ROS2 dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-nav-msgs \
  ros-humble-geometry-msgs \
  ros-humble-std-msgs \
  ros-humble-slam-toolbox \
  ros-humble-multirobot-map-merge
```

### 1.2 — Clone auto_swarm_bridge

```bash
cd ~/ausra_ws/src
git clone https://github.com/fast-fire/auto_swarm_bridge.git
```

### 1.3 — Copy ausra_comms to the machine

Copy this entire `ausra_comms/` folder into `~/ausra_ws/src/` on every machine.
The final path must be `~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms/`.

### 1.4 — Build

```bash
cd ~/ausra_ws
colcon build --packages-select auto_swarm_bridge ausra_comms
source install/setup.bash
```

Verify zero errors before continuing.

### 1.5 — Make scripts executable

```bash
chmod +x ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms/scripts/start_comms.sh
chmod +x ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms/scripts/start_base.sh
```

### 1.6 — Set ROS2 environment (optional — add to ~/.bashrc)

```bash
echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc
echo 'export ROS_LOCALHOST_ONLY=0' >> ~/.bashrc
source ~/.bashrc
```

The startup scripts already set these, but adding to `.bashrc` helps when debugging with manual `ros2 topic` commands.

---

## 2. What to Edit

You must replace **4 placeholder strings** with real IP addresses. The placeholders are literal strings — search and replace them exactly.

### Placeholder table

| Placeholder | Replace with | How to find it |
|-------------|-------------|----------------|
| `ROBOT1_IP` | IP of ausra_1 | Run `ip addr show wlan0 \| grep "inet "` on ausra_1 |
| `ROBOT2_IP` | IP of ausra_2 | Run `ip addr show wlan0 \| grep "inet "` on ausra_2 |
| `ROBOT3_IP` | IP of ausra_3 | Run `ip addr show wlan0 \| grep "inet "` on ausra_3 |
| `LAPTOP_IP` | IP of Laptop  | Run `ip addr show wlan0 \| grep "inet "` on Laptop |

> If your WiFi interface is not `wlan0`, replace it in the command above.
> Run `iw dev` or `ip link` to find the correct interface name.

### Files to edit

Edit these files **before building** (i.e., edit in `~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms/`, then rebuild).

#### Bridge configs — edit `self_ip` on the machine that owns the file

| File | Edit on | Change `self_ip` to |
|------|---------|---------------------|
| `config/bridge_ausra_1.yaml` | ausra_1 | ausra_1's real IP |
| `config/bridge_ausra_2.yaml` | ausra_2 | ausra_2's real IP |
| `config/bridge_ausra_3.yaml` | ausra_3 | ausra_3's real IP |
| `config/bridge_laptop.yaml` | Laptop  | Laptop's real IP |

Example — if ausra_1's IP is `192.168.1.101`:
```yaml
# In config/bridge_ausra_1.yaml, change:
self_ip: "ROBOT1_IP"
# To:
self_ip: "192.168.1.101"
```

#### Startup scripts — edit all 4 IP variables

**`scripts/start_comms.sh`** — edit these 4 lines (runs on robots):
```bash
ROBOT1_IP="ROBOT1_IP"    # ← replace with real IP
ROBOT2_IP="ROBOT2_IP"    # ← replace with real IP
ROBOT3_IP="ROBOT3_IP"    # ← replace with real IP
LAPTOP_IP="LAPTOP_IP"    # ← replace with real IP
```

**`scripts/start_base.sh`** — edit these 3 lines (runs on laptop):
```bash
ROBOT1_IP="ROBOT1_IP"    # ← replace with real IP
ROBOT2_IP="ROBOT2_IP"    # ← replace with real IP
ROBOT3_IP="ROBOT3_IP"    # ← replace with real IP
```

### Quick sed replacement (optional)

If all machines share the same source folder, run this once:
```bash
cd ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms

# Replace with your actual IPs
sed -i 's/ROBOT1_IP/192.168.1.101/g' config/*.yaml scripts/*.sh
sed -i 's/ROBOT2_IP/192.168.1.102/g' config/*.yaml scripts/*.sh
sed -i 's/ROBOT3_IP/192.168.1.103/g' config/*.yaml scripts/*.sh
sed -i 's/LAPTOP_IP/192.168.1.100/g' config/*.yaml scripts/*.sh
```

### After editing — rebuild

```bash
cd ~/ausra_ws
colcon build --packages-select ausra_comms
source install/setup.bash
```

You must rebuild after editing config files so they get installed to `install/`.

---

## 3. How to Run

### Step 1 — Verify network first

From the laptop, ping all robots:
```bash
ping -c 3 192.168.1.101   # ausra_1
ping -c 3 192.168.1.102   # ausra_2
ping -c 3 192.168.1.103   # ausra_3
```

All must reply. Fix networking before continuing.

### Step 2 — Start robots (one SSH session per robot)

```bash
# SSH into ausra_1
ssh user@192.168.1.101
cd ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms/scripts
./start_comms.sh 1
```

```bash
# SSH into ausra_2
ssh user@192.168.1.102
cd ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms/scripts
./start_comms.sh 2
```

```bash
# SSH into ausra_3
ssh user@192.168.1.103
cd ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms/scripts
./start_comms.sh 3
```

### Step 3 — Start base station (on laptop)

```bash
cd ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms/scripts
./start_base.sh
```

This launches the bridge, map merge, and RViz2 together.

### Order matters

Start **robots first**, then **laptop**. The bridge on each robot begins sending immediately. The laptop bridge connects and starts receiving.

---

## 4. How to Verify

Open a **new terminal** on the laptop (separate from `start_base.sh`). Source the workspace first:

```bash
source /opt/ros/humble/setup.bash
source ~/ausra_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

### 4.1 — Check topics exist

```bash
ros2 topic list | grep robot
```

**Good output:**
```
/ausra_1/heartbeat
/ausra_1/map
/ausra_1/pose
/ausra_2/heartbeat
/ausra_2/map
/ausra_2/pose
/ausra_3/heartbeat
/ausra_3/map
/ausra_3/pose
```

### 4.2 — Check heartbeats (should print immediately)

```bash
ros2 topic echo /ausra_1/heartbeat
```

**Good output:**
```
data: 'ausra_1 alive'
---
data: 'ausra_1 alive'
```

Repeat for `/ausra_2/heartbeat` and `/ausra_3/heartbeat`.

### 4.3 — Check heartbeat rate (should be ~1 Hz)

```bash
ros2 topic hz /ausra_1/heartbeat
```

**Good output:**
```
average rate: 1.000
```

### 4.4 — Check pose (requires SLAM running on the robot)

```bash
ros2 topic echo /ausra_1/pose
```

**Good output:** PoseStamped messages with changing x/y values as the robot moves.

```bash
ros2 topic hz /ausra_1/pose
```

**Good output:** `average rate: ~10.0`

### 4.5 — Check map (requires SLAM running — arrives every ~30 seconds)

```bash
ros2 topic echo /ausra_1/map --no-arr
```

**Good output:** OccupancyGrid header data (without printing the full data array).

```bash
ros2 topic hz /ausra_1/map
```

**Good output:** `average rate: ~0.033` (one message per 30 seconds).

### 4.6 — Check global merged map

```bash
ros2 topic echo /map_merged --no-arr
```

**Good output:** OccupancyGrid headers. This only works after at least 2 robots have sent maps with overlapping areas.

### 4.7 — RViz2 visualization

RViz2 opens automatically with `start_base.sh`. Add these displays manually:

1. **Add → By topic → /map_merged → Map** — the merged map
2. **Add → By topic → /ausra_1/map → Map** — set alpha to 0.4
3. **Add → By topic → /ausra_2/map → Map** — set alpha to 0.4
4. **Add → By topic → /ausra_3/map → Map** — set alpha to 0.4

Set **Fixed Frame** to `map` in the top-left panel.

---

## 5. Common Problems

### "Bridge config not found"

```
[Swarm] ERROR: Bridge config not found: .../bridge_ausra_1.yaml
```

**Fix:** You edited source files but forgot to rebuild.
```bash
cd ~/ausra_ws
colcon build --packages-select ausra_comms
source install/setup.bash
```

### Topics exist locally but not on other machines

**Symptom:** `ros2 topic list` on ausra_1 shows `/ausra_1/heartbeat`, but the laptop does not see it.

**Check:**
```bash
# On both machines — must be identical
echo $ROS_DOMAIN_ID        # must print 0
echo $ROS_LOCALHOST_ONLY    # must print 0
```

```bash
# On the laptop — is the bridge node running?
ros2 node list | grep bridge
```

If the bridge is not running, check the bridge config has a valid `self_ip` (not the placeholder string).

### Heartbeat works but pose/map does not

**Cause:** SLAM (slam_toolbox) is not running on the robot. The relay node subscribes to `/map` and `/pose` — if SLAM doesn't publish them, there's nothing to relay.

**Check on the robot:**
```bash
ros2 topic list | grep -E "^/map$|^/pose$"
```

If `/map` and `/pose` don't appear, start slam_toolbox first.

### Map merge shows nothing / global_map is empty

**Cause 1:** Only one robot's map has arrived. Map merge needs at least 2 maps with overlapping areas.

**Cause 2:** Maps don't overlap. Start robots in the same room so their initial scans share features, then drive them apart.

**Check:**
```bash
ros2 topic hz /ausra_1/map
ros2 topic hz /ausra_2/map
```

Both must show messages arriving. If one is missing, check that robot's relay + bridge.

### Ping works but bridge doesn't connect

**Check:** Is the `self_ip` in the bridge config correct? It must match the IP of the machine running that config.

```bash
# On the robot
ip addr show wlan0 | grep "inet "
# Compare with self_ip in the bridge yaml
```

### "address already in use" error from the bridge

**Cause:** A previous bridge instance is still running.

```bash
# Kill all ROS2 nodes
pkill -f ros2
# Or kill the specific bridge process
pkill -f auto_swarm_bridge
```

Then restart.

---

## 6. Switching to IBSS Later

If you start on a WiFi router (Path B) and later switch to IBSS ad-hoc (Path A), here is what changes.

### What changes

**Only the IP addresses.** Edit these files with the new IBSS IPs:

| File | Field to change |
|------|----------------|
| `config/bridge_ausra_1.yaml` | `self_ip` |
| `config/bridge_ausra_2.yaml` | `self_ip` |
| `config/bridge_ausra_3.yaml` | `self_ip` |
| `config/bridge_laptop.yaml` | `self_ip` |
| `scripts/start_comms.sh` | `ROBOT1_IP`, `ROBOT2_IP`, `ROBOT3_IP`, `LAPTOP_IP` |
| `scripts/start_base.sh` | `ROBOT1_IP`, `ROBOT2_IP`, `ROBOT3_IP` |

Example IBSS IP mapping:

| Machine | Router IP | IBSS IP |
|---------|-----------|---------|
| Laptop  | 192.168.1.100 | 192.168.10.1 |
| ausra_1 | 192.168.1.101 | 192.168.10.2 |
| ausra_2 | 192.168.1.102 | 192.168.10.3 |
| ausra_3 | 192.168.1.103 | 192.168.10.4 |

### Quick sed swap (router → IBSS)

```bash
cd ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms

sed -i 's/192.168.1.100/192.168.10.1/g' config/*.yaml scripts/*.sh
sed -i 's/192.168.1.101/192.168.10.2/g' config/*.yaml scripts/*.sh
sed -i 's/192.168.1.102/192.168.10.3/g' config/*.yaml scripts/*.sh
sed -i 's/192.168.1.103/192.168.10.4/g' config/*.yaml scripts/*.sh
```

Then rebuild:
```bash
cd ~/ausra_ws
colcon build --packages-select ausra_comms
source install/setup.bash
```

### What stays the same

Everything else:

- `relay_node.py` — no IP references
- `robot_comms.launch.py` — no IP references
- `base_station_comms.launch.py` — no IP references
- `map_merge.launch.py` — no IP references
- `package.xml`, `setup.py`, `setup.cfg` — no IP references
- `ROS_DOMAIN_ID=0` — unchanged
- Topic names (`/robotX/map`, `/robotX/pose`, `/robotX/heartbeat`) — unchanged
- Map merge parameters — unchanged

The IBSS network itself must be configured separately on each machine at the OS level (see `COMMS_MILESTONES.md`, Path A). The ROS2 comms layer doesn't care how the network was formed — it only needs the correct IPs.

---

## 7. Testing With Limited Hardware

You don't need all 3 Jetsons to test the comms pipeline. Two modes are available:

### Mode A — 1 Jetson + Laptop (2-robot mode)

The laptop acts as **ausra_2 AND base station simultaneously**. ausra_1 runs on the real Jetson with real SLAM. ausra_2 data is generated by a fake publisher on the laptop.

#### Files used

| File | Machine | Purpose |
|------|---------|---------|
| `config/bridge_ausra_1_2robots.yaml` | Jetson | ausra_1 sends, receives from ausra_2 only |
| `config/bridge_laptop_2robots.yaml` | Laptop | ausra_2 sends, receives from ausra_1 only |
| `scripts/start_comms_2robots.sh` | Jetson | Starts relay + bridge for ausra_1 |
| `scripts/start_base_2robots.sh` | Laptop | Starts bridge + fake ausra_2 + map merge + RViz2 |

#### Setup

Edit placeholders in the 2-robot files:

```bash
cd ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms

# In config/bridge_ausra_1_2robots.yaml — set self_ip to ausra_1's real IP
# In config/bridge_laptop_2robots.yaml — set self_ip to Laptop's real IP
# In scripts/start_comms_2robots.sh — set LAPTOP_IP
# In scripts/start_base_2robots.sh — set ROBOT1_IP

# Rebuild
cd ~/ausra_ws
colcon build --packages-select ausra_comms
source install/setup.bash
```

#### Run

```bash
# On Jetson (ausra_1) — start SLAM first, then:
cd ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms/scripts
chmod +x start_comms_2robots.sh
./start_comms_2robots.sh

# On Laptop — in a separate terminal:
cd ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms/scripts
chmod +x start_base_2robots.sh
./start_base_2robots.sh
```

#### Verify

```bash
# In a new laptop terminal:
source /opt/ros/humble/setup.bash
source ~/ausra_ws/install/setup.bash
export ROS_DOMAIN_ID=0

ros2 topic list | grep robot
# Expected:
#   /ausra_1/heartbeat   ← from real Jetson
#   /ausra_1/map         ← from real Jetson (real SLAM)
#   /ausra_1/pose        ← from real Jetson (real SLAM)
#   /ausra_2/heartbeat   ← from fake publisher on laptop
#   /ausra_2/map         ← from fake publisher on laptop
#   /ausra_2/pose        ← from fake publisher on laptop

ros2 topic echo /ausra_1/heartbeat   # real data from Jetson
ros2 topic echo /ausra_2/heartbeat   # fake data from laptop
ros2 topic echo /map_merged --no-arr  # merged map (may take 30-60s)
```

#### What this mode tests

- ✓ Real SLAM data flows from Jetson → laptop over WiFi
- ✓ Bridge serialization/deserialization works end-to-end
- ✓ Relay node correctly namespaces `/map` → `/ausra_1/map`
- ✓ Map merge receives maps from 2 sources
- ✓ RViz2 visualization works
- ✗ Cannot test real multi-robot SLAM overlap (ausra_2 map is fake)
- ✗ Cannot test 3-robot bridge topology

---

### Mode B — Laptop Only (no Jetsons at all)

Everything runs on the laptop with fake data. No WiFi needed. No hardware needed.

#### Run

```bash
cd ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms/scripts
chmod +x start_single_robot_test.sh
./start_single_robot_test.sh
```

The script automatically:
1. Creates a temporary loopback bridge config (127.0.0.1)
2. Starts two fake robot publishers (ausra_1 + ausra_2)
3. Starts the bridge
4. Starts map merge
5. Verifies all expected topics exist and prints ✓/✗ for each
6. Cleans up temp files on Ctrl+C

#### Verify

The script prints verification results automatically. For manual checking:

```bash
# In a new terminal:
source /opt/ros/humble/setup.bash
source ~/ausra_ws/install/setup.bash
export ROS_DOMAIN_ID=0

ros2 topic echo /ausra_1/heartbeat    # prints "ausra_1 alive" at 1 Hz
ros2 topic hz /ausra_1/pose           # ~10 Hz
ros2 topic echo /ausra_2/pose         # fake circular motion
ros2 topic echo /map_merged --no-arr # merged map (wait ~30s)
```

#### What this mode tests

- ✓ All ROS2 nodes start without errors
- ✓ Topic naming conventions are correct (`/robotX/topic`)
- ✓ Message types match between publishers and subscribers
- ✓ Bridge can serialize/deserialize all message types
- ✓ Map merge node launches and discovers robot namespaces
- ✓ Full pipeline runs without crashing for extended periods
- ✗ Cannot test real WiFi network latency or packet loss
- ✗ Cannot test real SLAM data
- ✗ Cannot test cross-machine discovery

---

### Comparison table

| Capability | Laptop Only | 1 Jetson + Laptop | Full 3-Robot |
|------------|:-----------:|:------------------:|:------------:|
| Nodes start without errors | ✓ | ✓ | ✓ |
| Topic names correct | ✓ | ✓ | ✓ |
| Bridge serialization | ✓ | ✓ | ✓ |
| Map merge launches | ✓ | ✓ | ✓ |
| Real SLAM data | ✗ | ✓ (1 robot) | ✓ |
| WiFi cross-machine | ✗ | ✓ | ✓ |
| Multi-robot map overlap | ✗ | ✗ | ✓ |
| 3-robot topology | ✗ | ✗ | ✓ |

**Recommended testing order:** Laptop Only → 1 Jetson + Laptop → Full 3-Robot.
