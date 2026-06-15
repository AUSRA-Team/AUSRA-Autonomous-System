# AUSRA Jetson Supervisor System — Agent Implementation Prompt

## Role and task

You are implementing the on-robot software infrastructure for the **AUSRA swarm robotics system**. Your task is to create all files needed on each Jetson Orin Nano so that robots boot autonomously, self-assign a namespace, and wait for commands from a base-station fleet supervisor — with zero SSH interaction required during normal operation.

Read this entire prompt before creating any file. The order of implementation matters because systemd units have explicit dependencies.

---

## System context you must understand before writing any code

### Hardware per robot
- **Jetson Orin Nano** (Ubuntu 22.04, ROS 2 Humble) — runs all high-level autonomy
- **ESP32-S3** (micro-ROS firmware) — handles real-time motor control, encoder feedback, IMU
- Connection: ESP32 ↔ Jetson via **USB serial at `/dev/ttyACM0`**
- Lidar: **RPLIDAR A1M8** on `/dev/ttyUSB0`
- IMU: **MPU6050** via I2C

### Network topology
- All Jetsons and the base-station laptop connect to a **single Wi-Fi router** (LAN: `192.168.1.x`)
- Inter-robot and robot-to-laptop communication uses **`zenoh-bridge-ros2dds`** (Zenoh DDS bridge)
- Each Jetson runs its own local **ROS 2 domain** — raw sensor topics (`/odom`, `/scan`, `/tf`, `/joint_states`) **never cross Wi-Fi**
- Only a small **allowlisted topic set** is bridged via Zenoh (see Zenoh config section below)

### ROS 2 stack already in the workspace
The following packages already exist in `/opt/ausra/src/` (the workspace is at `/opt/ausra/`):
- `omnidirectional_driver` — omni-wheel kinematics driver, executable `omni_driver`
- `lidar_slam_pkg` — contains `slam_toolbox_config.yaml`, `nav2_holonomic_params.yaml`, `explore_params.yaml`
- `ausra_localization` — EKF config at `config/ekf.yaml`, IMU filter config at `config/imu_complimentary_filter.yaml`
- `ausrabot_description` — URDF/xacro at `urdf/robot.urdf.xacro`, hardware params at `config/hardware_params.yaml`
- `sllidar_ros2` — RPLIDAR A1 driver
- `mpu6050driver` — IMU driver, params at `params/mpu6050.yaml`

The **main launch file** is `ausra_bringup/launch/hardware_full_stack.launch.py`. It accepts the following launch arguments:
```
robot_name     (string)  — e.g. "ausra_1" — sets ROS namespace AND all TF frame prefixes
use_sim_time   (bool)    — always false on hardware
nudge_robot    (bool)    — optional SLAM seeding nudge
x, y, yaw     (float)   — initial pose hint for the static TF offset (map → {robot_name}_map)
```

The launch file derives all TF frame names from `robot_name`:
```
{robot_name}_robot_footprint   ← base_link / robot footprint frame
{robot_name}_odom              ← odometry frame (EKF publishes this → base_frame)
{robot_name}_lidar             ← lidar TF frame
{robot_name}_imu_link          ← IMU TF frame
{robot_name}_map               ← per-robot map frame (offset from global "map" via static TF)
```

### Namespace naming convention
Robots are named `ausra_1`, `ausra_2`, ..., `ausra_N`. The namespace is **not hardcoded** — it is dynamically negotiated at boot over Zenoh before any ROS process starts. `ausra_1` is the lowest available slot, not a fixed identity.

### ESP32 namespace injection
Before the micro-ROS agent starts, the ESP32 must be told its namespace. The firmware reads a namespace string from the serial port on startup and uses it to prefix all micro-ROS entities (topics, services). The injection is done by writing the namespace string to `/dev/ttyACM0` after setting baud rate to 6000000. After writing, the ESP32 performs a DTR reset, so the Jetson must wait **3 seconds** before starting the micro-ROS agent.

---

## What you must create

### New packages to create (inside `/opt/ausra/src/`)

```
/opt/ausra/src/
├── ausra_msgs/                        ← NEW: custom ROS 2 message and action types
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── msg/
│   │   ├── RobotStatus.msg
│   │   └── FleetStatus.msg
│   └── action/
│       └── SendTask.action
│
├── ausra_supervisor/                  ← NEW: per-robot sidecar node (runs on Jetson)
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── resource/
│   │   └── ausra_supervisor
│   └── ausra_supervisor/
│       ├── __init__.py
│       ├── supervisor_node.py
│       └── topic_watchdog.py
│
└── ausra_bringup/                     ← NEW: systemd units, scripts, Zenoh configs, install tooling
    ├── CMakeLists.txt
    ├── package.xml
    ├── launch/
    │   └── hardware_full_stack.launch.py   ← MOVE existing launch file here (do not rewrite its logic)
    ├── scripts/
    │   ├── ns_resolver.py                  ← Zenoh-based namespace negotiation
    │   ├── send_namespace_to_esp32.sh      ← Serial namespace injection + wait
    │   ├── gen_zenoh_config.sh             ← Templates robot_bridge.json5 with actual namespace
    │   └── install_jetson.sh               ← One-time setup script per Jetson
    ├── systemd/
    │   ├── ausra-ns-resolver.service
    │   ├── ausra-micro-ros.service
    │   ├── ausra-ros-stack.service
    │   ├── ausra-zenoh-bridge.service
    │   └── ausra-supervisor.service
    └── zenoh/
        ├── robot_bridge.json5              ← Template: uses __ROBOT_NS__ placeholder
        └── laptop_bridge.json5             ← Static config for base-station (reference only)
```

### System-level files to create (outside the ROS workspace)

```
/etc/ausra/
    .gitkeep                   ← directory placeholder; namespace and namespace.env are written at runtime

/etc/systemd/system/           ← symlinked or copied by install_jetson.sh, not created directly here
```

---

## Implementation instructions — follow this order exactly

---

### PHASE 1: `ausra_msgs` package

#### `ausra_msgs/msg/RobotStatus.msg`
```
string   robot_name
uint8    state
# State constants: 0=IDLE 1=NAVIGATING 2=DEGRADED 3=ESTOP 4=LOST
float32  battery_pct
float32  pose_x
float32  pose_y
float32  pose_yaw
string   active_task_id
string[] degraded_systems
builtin_interfaces/Time stamp
```

#### `ausra_msgs/msg/FleetStatus.msg`
```
ausra_msgs/RobotStatus[] robots
uint32 active_count
uint32 degraded_count
uint32 lost_count
builtin_interfaces/Time stamp
```

#### `ausra_msgs/action/SendTask.action`
```
# Goal
string robot_name
geometry_msgs/PoseStamped goal
string task_id
---
# Result
bool success
string failure_reason
---
# Feedback
ausra_msgs/RobotStatus feedback
```

#### `ausra_msgs/package.xml`
Standard ROS 2 ament_cmake package. Dependencies: `rosidl_default_generators` (buildtool), `geometry_msgs`, `builtin_interfaces`, `rosidl_default_runtime` (exec). Export `rosidl_interface_packages` in the `<member_of_group>` tag.

#### `ausra_msgs/CMakeLists.txt`
Use `rosidl_generate_interfaces()` to register all `.msg` and `.action` files. Add dependency on `geometry_msgs`.

---

### PHASE 2: `ausra_bringup` package

#### `ausra_bringup/scripts/ns_resolver.py`

This is the most critical file. It runs as a systemd service BEFORE any ROS process. It uses the raw **Zenoh Python SDK** (`import zenoh`) — NOT rclpy — because ROS is not started yet.

Logic (implement exactly in this order):

1. Import: `zenoh`, `json`, `time`, `socket`, `os`, `sys`, and `sdnotify` (for systemd `Type=notify` integration).

2. **Constants at module top:**
   ```python
   ZENOH_ROUTER     = "tcp/192.168.1.1:7447"
   NS_FILE          = "/etc/ausra/namespace"
   NS_ENV_FILE      = "/etc/ausra/namespace.env"
   HEARTBEAT_KEY    = "ausra/heartbeat/{ns}"
   CLAIM_KEY        = "ausra/ns-claim/{ns}"
   LISTEN_WINDOW    = 2.5   # seconds to collect existing heartbeats
   COLLISION_WINDOW = 0.6   # seconds to watch for a collision after publishing claim
   MAX_N            = 20
   ```

3. **`wait_for_network(router_ip, port, timeout_sec=60)`** — loop every 1 second trying `socket.connect()` to the Zenoh router. If timeout exceeded, call `_fallback_and_exit()`.

4. **`_fallback_and_exit(notifier)`** — writes `ausra_fallback` to `NS_FILE` and `NS_ENV_FILE`, calls `notifier.notify("READY=1")`, then `sys.exit(0)`. This ensures the systemd chain continues even if network is unavailable (robot operates locally).

5. **`_write_namespace(ns)`** — writes `ns` (plain string) to `NS_FILE`. Also writes `AUSRA_ROBOT_NAME={ns}\n` to `NS_ENV_FILE` (this is the `EnvironmentFile=` format that systemd reads).

6. **`get_mac_tail()`** — returns last 4 hex chars of `uuid.getnode()` as a string. Used as a tiebreaker in collision detection.

7. **`main()`:**
   - Create `sdnotify.SystemdNotifier()` and call `notifier.notify("STATUS=Waiting for Wi-Fi...")`.
   - Call `wait_for_network(...)`.
   - Open a Zenoh `session` in `client` mode connecting to `ZENOH_ROUTER`. Do NOT set any namespace on the session — this is a pre-namespace anonymous session.
   - Declare a subscriber on `"ausra/heartbeat/*"`. In the callback, extract the last path segment (e.g. `"ausra/heartbeat/ausra_2"` → `"ausra_2"`) and add it to a `set` called `seen`.
   - `time.sleep(LISTEN_WINDOW)` then undeclare the subscriber.
   - Find `candidate`: iterate `n = 1, 2, ..., MAX_N`, pick the first `ausra_N` not in `seen`.
   - Set `collision = False`. Declare a subscriber on `f"ausra/ns-claim/{candidate}"`. In the callback, if `json.loads(payload)["mac"] != get_mac_tail()`, set `collision = True`.
   - Publish `json.dumps({"claiming": candidate, "mac": get_mac_tail()})` to `f"ausra/ns-claim/{candidate}"`.
   - `time.sleep(COLLISION_WINDOW)`.
   - Undeclare the claim subscriber. Close the session.
   - If `collision is True`: `time.sleep(0.5 + random.uniform(0, 0.3))` then call `main()` recursively (the retry will re-open a fresh session and re-observe the heartbeat bus, where the colliding robot's heartbeat will now appear).
   - If no collision: call `_write_namespace(candidate)`, then `notifier.notify(f"STATUS=Namespace: {candidate}")`, then `notifier.notify("READY=1")`.

8. **`if __name__ == "__main__": main()`**

---

#### `ausra_bringup/scripts/send_namespace_to_esp32.sh`

```bash
#!/bin/bash
# Sends the robot namespace to the ESP32-S3 via USB serial.
# Called by ausra-micro-ros.service as ExecStartPre=.
# Reads namespace from /etc/ausra/namespace (written by ns_resolver.py).
set -e

NS_FILE="/etc/ausra/namespace"
SERIAL_DEV="/dev/ttyACM0"
BAUD=6000000
WAIT_AFTER_DTR=3  # seconds — ESP32 resets on DTR, must wait before agent connects

if [ ! -f "$NS_FILE" ]; then
    echo "[send_ns] ERROR: $NS_FILE not found. ns_resolver must run first." >&2
    exit 1
fi

NS=$(cat "$NS_FILE")
echo "[send_ns] Sending namespace '$NS' to ESP32 on $SERIAL_DEV"

stty -F "$SERIAL_DEV" "$BAUD"
echo "$NS" > "$SERIAL_DEV"

echo "[send_ns] Waiting ${WAIT_AFTER_DTR}s for ESP32 DTR reset..."
sleep "$WAIT_AFTER_DTR"
echo "[send_ns] Done."
```

---

#### `ausra_bringup/scripts/gen_zenoh_config.sh`

```bash
#!/bin/bash
# Templates /opt/ausra/zenoh/robot_bridge.json5 → /tmp/ausra_zenoh_bridge.json5
# Substitutes __ROBOT_NS__ with the actual namespace from /etc/ausra/namespace.
# Called by ausra-zenoh-bridge.service as ExecStartPre=.
set -e

NS=$(cat /etc/ausra/namespace)
TEMPLATE="/opt/ausra/zenoh/robot_bridge.json5"
OUTPUT="/tmp/ausra_zenoh_bridge.json5"

sed "s/__ROBOT_NS__/${NS}/g" "$TEMPLATE" > "$OUTPUT"
echo "[gen_zenoh_config] Generated $OUTPUT for namespace: $NS"
```

---

#### `ausra_bringup/zenoh/robot_bridge.json5`

Use the token `__ROBOT_NS__` everywhere the robot name appears. `gen_zenoh_config.sh` will substitute it.

```json5
{
  mode: "client",
  connect: {
    endpoints: ["tcp/192.168.1.1:7447"]
  },

  plugins: {
    ros2dds: {
      namespace: "/__ROBOT_NS__",

      // ── Topics this robot publishes OUTWARD onto the shared Zenoh session ──
      // These are the ONLY topics that cross Wi-Fi. Everything else is local.
      pub_topics: [
        "/__ROBOT_NS__/supervisor/status",
        "/__ROBOT_NS__/supervisor/heartbeat",
        "/__ROBOT_NS__/map",
        "/localized_scan",
        "ausra/heartbeat/__ROBOT_NS__"
      ],

      // ── Topics this robot subscribes to FROM the shared Zenoh session ──
      sub_topics: [
        "/__ROBOT_NS__/supervisor/task_goal",
        "/fleet/estop_all",
        "/fleet/task_broadcast"
      ]

      // CRITICAL: topics NOT listed above (odom, scan, tf, joint_states,
      // filtered_odometry, cmd_vel, joint_group_velocity_controller/commands)
      // are never bridged. They stay in the Jetson's local ROS 2 domain.
    }
  }
}
```

#### `ausra_bringup/zenoh/laptop_bridge.json5`

```json5
{
  mode: "client",
  connect: {
    endpoints: ["tcp/192.168.1.1:7447"]
  },

  plugins: {
    ros2dds: {
      // Subscribe to all robots' status and maps dynamically
      sub_topics: [
        "/**/supervisor/status",
        "/**/supervisor/heartbeat",
        "/**/map",
        "/localized_scan",
        "ausra/heartbeat/**"
      ],

      // Publish tasks and fleet commands to robots
      pub_topics: [
        "/**/supervisor/task_goal",
        "/fleet/estop_all",
        "/fleet/task_broadcast",
        "ausra/ns-claim/**"
      ]
    }
  }
}
```

---

#### `ausra_bringup/systemd/ausra-ns-resolver.service`

```ini
[Unit]
Description=AUSRA dynamic namespace resolver
Documentation=https://github.com/AUSRA-Team
After=network-online.target
Wants=network-online.target

[Service]
Type=notify
ExecStart=/usr/bin/python3 /opt/ausra/scripts/ns_resolver.py
Restart=on-failure
RestartSec=5
TimeoutStartSec=120
User=root
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

**Why `Type=notify`:** The resolver calls `sdnotify` to signal READY only after the namespace file is written. This makes all dependent services (`After=ausra-ns-resolver.service`) wait for the actual namespace to be available, not just for the process to start.

---

#### `ausra_bringup/systemd/ausra-micro-ros.service`

```ini
[Unit]
Description=AUSRA micro-ROS agent (ESP32-S3 bridge)
After=ausra-ns-resolver.service
Requires=ausra-ns-resolver.service
BindsTo=ausra-ns-resolver.service

[Service]
Type=simple
EnvironmentFile=/etc/ausra/namespace.env
ExecStartPre=/bin/bash /opt/ausra/scripts/send_namespace_to_esp32.sh
ExecStart=/bin/bash -c '\
    source /opt/ros/humble/setup.bash && \
    source /opt/ausra/install/setup.bash && \
    exec ros2 run micro_ros_agent micro_ros_agent \
        serial --dev /dev/ttyACM0 \
        --ros-args -r __ns:=/${AUSRA_ROBOT_NAME}'
Restart=always
RestartSec=5
User=ausra
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

**Note:** `EnvironmentFile=/etc/ausra/namespace.env` reads `AUSRA_ROBOT_NAME=ausra_1` (written by `ns_resolver.py`). The `ExecStart` uses this variable to inject `__ns:=/{AUSRA_ROBOT_NAME}` into the micro-ROS agent so all ESP32-published entities appear under the correct namespace.

---

#### `ausra_bringup/systemd/ausra-ros-stack.service`

```ini
[Unit]
Description=AUSRA ROS 2 hardware stack (SLAM, Nav2, EKF, drivers)
After=ausra-micro-ros.service
Requires=ausra-micro-ros.service

[Service]
Type=simple
EnvironmentFile=/etc/ausra/namespace.env
ExecStart=/bin/bash -c '\
    source /opt/ros/humble/setup.bash && \
    source /opt/ausra/install/setup.bash && \
    exec ros2 launch ausra_bringup hardware_full_stack.launch.py \
        robot_name:=${AUSRA_ROBOT_NAME} \
        use_sim_time:=false'
Restart=always
RestartSec=5
TimeoutStartSec=120
User=ausra
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

---

#### `ausra_bringup/systemd/ausra-zenoh-bridge.service`

```ini
[Unit]
Description=AUSRA Zenoh-ROS2 bridge (selective topic bridging over Wi-Fi)
After=ausra-ros-stack.service
Requires=ausra-ros-stack.service

[Service]
Type=simple
EnvironmentFile=/etc/ausra/namespace.env
ExecStartPre=/bin/bash /opt/ausra/scripts/gen_zenoh_config.sh
ExecStart=/usr/local/bin/zenoh-bridge-ros2dds \
    --config /tmp/ausra_zenoh_bridge.json5
Restart=always
RestartSec=3
User=ausra
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

---

#### `ausra_bringup/systemd/ausra-supervisor.service`

```ini
[Unit]
Description=AUSRA robot supervisor sidecar (health, task FSM, e-stop)
After=ausra-ros-stack.service
Requires=ausra-ros-stack.service

[Service]
Type=simple
EnvironmentFile=/etc/ausra/namespace.env
ExecStart=/bin/bash -c '\
    source /opt/ros/humble/setup.bash && \
    source /opt/ausra/install/setup.bash && \
    exec ros2 run ausra_supervisor supervisor_node \
        --ros-args \
        -r __ns:=/${AUSRA_ROBOT_NAME} \
        -p robot_name:=${AUSRA_ROBOT_NAME} \
        -p max_retries:=2 \
        -p watchdog_timeout_odom:=0.5 \
        -p watchdog_timeout_scan:=0.4 \
        -p watchdog_timeout_ekf:=0.5 \
        -p heartbeat_hz:=2.0 \
        -p status_hz:=1.0 \
        -p estop_duration_sec:=3.0'
Restart=always
RestartSec=5
User=ausra
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

---

### PHASE 3: `ausra_supervisor` package

This is a standard ROS 2 Python package. The node runs on the Jetson under the robot's namespace.

#### `ausra_supervisor/ausra_supervisor/topic_watchdog.py`

Implement a `TopicWatchdog` class:
- Constructor: `__init__(self, node, topic: str, msg_type, timeout_sec: float)`
- Creates a ROS 2 subscription that, on every received message, records `time.monotonic()` as `self._last_recv`.
- Property `healthy -> bool`: returns `True` if `(time.monotonic() - self._last_recv) < self._timeout`.
- Starts with `self._last_recv = 0.0` (will be `healthy=False` until first message arrives — intentional).

#### `ausra_supervisor/ausra_supervisor/supervisor_node.py`

Implement `SupervisorNode(Node)` with the following behavior. Use only `rclpy`, `nav2_msgs`, `geometry_msgs`, `std_msgs`, `nav_msgs`, `sensor_msgs`, and `ausra_msgs`.

**Parameters declared in `__init__`:**
```
robot_name           (string,  default='ausra_1')
max_retries          (int,     default=2)
watchdog_timeout_odom (double, default=0.5)
watchdog_timeout_scan (double, default=0.4)
watchdog_timeout_ekf  (double, default=0.5)
heartbeat_hz         (double,  default=2.0)
status_hz            (double,  default=1.0)
estop_duration_sec   (double,  default=3.0)
```

**State machine (internal `int` field `_state`):**
- `0` = IDLE
- `1` = NAVIGATING
- `2` = DEGRADED
- `3` = ESTOP

**Subscriptions (all topic paths use the `robot_name` parameter):**
- `/{robot_name}/filtered_odometry` → `nav_msgs/Odometry` → updates `(self._pose_x, self._pose_y, self._pose_yaw)` by extracting x, y, and yaw from quaternion
- `/{robot_name}/supervisor/task_goal` → `geometry_msgs/PoseStamped` → `_on_task_goal`
- `/fleet/estop_all` → `std_msgs/Bool` → `_on_estop`

**Watchdogs (created in `__init__` after parameters are loaded):**
```python
self._watchdogs = {
    'odom': TopicWatchdog(self, f'/{ns}/odom',               Odometry,   timeout_odom),
    'scan': TopicWatchdog(self, f'/{ns}/scan',               LaserScan,  timeout_scan),
    'ekf':  TopicWatchdog(self, f'/{ns}/filtered_odometry',  Odometry,   timeout_ekf),
}
```

**Nav2 action client:** `ActionClient(self, NavigateToPose, f'/{ns}/navigate_to_pose')`

Store the last `PoseStamped` received in `_last_goal` for retry.

**Publishers:**
- `/{robot_name}/supervisor/status` → `ausra_msgs/RobotStatus` (1 Hz)
- `/{robot_name}/supervisor/heartbeat` → `std_msgs/Header` (2 Hz)
- `/{robot_name}/cmd_vel` → `geometry_msgs/Twist` (used only for e-stop zero-velocity flood)

**`_on_task_goal(msg)`:**
- If `_state == ESTOP`: log warning, return
- If `_state == NAVIGATING`: log warning, return
- Store `msg` as `_last_goal`. Set `_retries = 0`. Call `_send_nav_goal(msg)`.

**`_send_nav_goal(pose)`:**
- Set `_state = NAVIGATING`
- Generate a new `_active_task_id` (8-char UUID fragment)
- Wait up to 5 seconds for action server. If unavailable, call `_handle_failure("action server unavailable")`
- `send_goal_async` → callback `_on_goal_response`

**`_on_goal_response(future)`:**
- If not accepted: `_handle_failure("goal rejected")`
- Else: `get_result_async()` → callback `_on_nav_result`

**`_on_nav_result(future)`:**
- Extract status. If `STATUS_SUCCEEDED`: set `_state = IDLE`, clear `_active_task_id`, `_retries = 0`
- Else: `_handle_failure(f"nav2 status {status}")`

**`_handle_failure(reason)`:**
- Log warning with reason and retry count
- If `_retries < max_retries`: increment `_retries`, call `_send_nav_goal(self._last_goal)`
- Else: log error "max retries reached", set `_state = IDLE`, clear `_active_task_id`

**`_on_estop(msg)`:**
- If `msg.data is False`: return (this is the "release e-stop" signal — currently just ignored, robot stays IDLE until new goal)
- Set `_state = ESTOP`
- If Nav2 goal is active, cancel it
- Create a one-shot repeating timer that publishes `Twist()` (all zeros) to `cmd_vel` at 10 Hz. The timer stops itself after `estop_duration_sec` seconds using `time.monotonic()`.

**`_check_health()`:**
- Build `degraded = [name for name, wd in self._watchdogs.items() if not wd.healthy]`
- Store in `self._degraded_systems`
- If `degraded` is non-empty AND `_state` is `IDLE` or `NAVIGATING`: set `_state = DEGRADED`
- If `degraded` is empty AND `_state == DEGRADED`: set `_state = IDLE`
- Do NOT override `ESTOP` state

**`_publish_heartbeat()`:** Publish a `std_msgs/Header` with `stamp = self.get_clock().now().to_msg()`

**`_publish_status()`:**
- Call `_check_health()` first
- Build and publish `ausra_msgs/RobotStatus` with all fields filled from current state

**`main()`:**
```python
def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

#### `ausra_supervisor/package.xml`
ament_python package. Exec depends: `rclpy`, `nav2_msgs`, `geometry_msgs`, `std_msgs`, `nav_msgs`, `sensor_msgs`, `ausra_msgs`.

#### `ausra_supervisor/setup.py`
Entry point: `supervisor_node = ausra_supervisor.supervisor_node:main`

---

### PHASE 4: `ausra_bringup/scripts/install_jetson.sh`

This script is run **once per Jetson** via SSH during first-time setup. After it runs, the Jetson never needs SSH again for operation.

```bash
#!/bin/bash
# AUSRA Jetson one-time installer
# Usage: sudo bash install_jetson.sh [ZENOH_ROUTER_IP] [AUSRA_USER]
# Example: sudo bash install_jetson.sh 192.168.1.1 ausra
set -e

ZENOH_ROUTER_IP=${1:-"192.168.1.1"}
AUSRA_USER=${2:-"ausra"}
AUSRA_WS="/opt/ausra"
ROS_DISTRO="humble"

echo "╔══════════════════════════════════════════╗"
echo "║   AUSRA Jetson Installer                 ║"
echo "║   Router: $ZENOH_ROUTER_IP               ║"
echo "║   User:   $AUSRA_USER                    ║"
echo "╚══════════════════════════════════════════╝"

# ── 1. Create ausra user if not exists ───────────────────────────
if ! id "$AUSRA_USER" &>/dev/null; then
    useradd -m -s /bin/bash "$AUSRA_USER"
    echo "[install] Created user: $AUSRA_USER"
fi

# Add to required groups (dialout for serial, video for GPU)
usermod -aG dialout,video,sudo "$AUSRA_USER"

# ── 2. Create directory structure ────────────────────────────────
mkdir -p /etc/ausra
mkdir -p /opt/ausra/scripts
mkdir -p /opt/ausra/zenoh
chown -R "$AUSRA_USER:$AUSRA_USER" /opt/ausra
chmod 755 /etc/ausra

# ── 3. Install Python dependencies for ns_resolver ───────────────
pip3 install eclipse-zenoh sdnotify

# ── 4. Build the ROS workspace ───────────────────────────────────
echo "[install] Building ausra workspace..."
cd "$AUSRA_WS"
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install \
    --packages-select ausra_msgs ausra_supervisor ausra_bringup \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# ── 5. Copy scripts to /opt/ausra/scripts/ ───────────────────────
BRINGUP_SRC="$AUSRA_WS/src/ausra_bringup"
cp "$BRINGUP_SRC/scripts/ns_resolver.py"             /opt/ausra/scripts/
cp "$BRINGUP_SRC/scripts/send_namespace_to_esp32.sh" /opt/ausra/scripts/
cp "$BRINGUP_SRC/scripts/gen_zenoh_config.sh"        /opt/ausra/scripts/
chmod +x /opt/ausra/scripts/*.sh
chmod +x /opt/ausra/scripts/ns_resolver.py

# ── 6. Template and install Zenoh config ─────────────────────────
# Inject the actual router IP into the template
sed "s/192.168.1.1/$ZENOH_ROUTER_IP/g" \
    "$BRINGUP_SRC/zenoh/robot_bridge.json5" \
    > /opt/ausra/zenoh/robot_bridge.json5

echo "[install] Zenoh config written to /opt/ausra/zenoh/robot_bridge.json5"

# ── 7. Install systemd units ─────────────────────────────────────
cp "$BRINGUP_SRC/systemd/"*.service /etc/systemd/system/
systemctl daemon-reload

for svc in ausra-ns-resolver ausra-micro-ros ausra-ros-stack ausra-zenoh-bridge ausra-supervisor; do
    systemctl enable "$svc"
    echo "[install] Enabled: $svc"
done

# ── 8. Verify zenoh-bridge-ros2dds binary exists ─────────────────
if ! command -v zenoh-bridge-ros2dds &>/dev/null; then
    echo "[install] WARNING: zenoh-bridge-ros2dds not found in PATH."
    echo "          Install it from: https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases"
    echo "          Expected location: /usr/local/bin/zenoh-bridge-ros2dds"
fi

echo ""
echo "╔══════════════════════════════════════════╗"
echo "║   Installation complete.                 ║"
echo "║   Reboot to start the AUSRA stack.       ║"
echo "║                                          ║"
echo "║   Monitor boot:                          ║"
echo "║   journalctl -u ausra-ns-resolver -f     ║"
echo "╚══════════════════════════════════════════╝"
```

---

## `ausra_bringup` package metadata files

#### `ausra_bringup/package.xml`
ament_cmake package. No exec or build dependencies needed (it installs data files only). Add `<buildtool_depend>ament_cmake</buildtool_depend>`.

#### `ausra_bringup/CMakeLists.txt`
```cmake
cmake_minimum_required(VERSION 3.8)
project(ausra_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config scripts systemd zenoh
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/ns_resolver.py
  scripts/send_namespace_to_esp32.sh
  scripts/gen_zenoh_config.sh
  scripts/install_jetson.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

---

## Constraints and rules the agent must follow

1. **Do not modify `hardware_full_stack.launch.py` logic.** Only move it to `ausra_bringup/launch/`. The launch file is correct as-is.

2. **`ns_resolver.py` must not import `rclpy` or any ROS package.** It uses only the raw Zenoh Python SDK. ROS is not yet started when it runs.

3. **All systemd `ExecStart` lines that run ROS commands must source both `/opt/ros/humble/setup.bash` AND `/opt/ausra/install/setup.bash`** before the `exec ros2 ...` call. Without both sources the commands will fail silently.

4. **All shell scripts must have `set -e` at the top** so failures surface immediately rather than being silently swallowed by systemd.

5. **The `ausra_supervisor` node must NOT have any startup race with the ROS stack.** The watchdogs will start `healthy=False` (no messages received yet) and will transition to `healthy=True` once each topic starts publishing. This is correct behavior — the node should NOT block startup waiting for topics.

6. **Do not hardcode `ausra_1` anywhere except as a parameter default.** All runtime values come from the `robot_name` parameter or the `AUSRA_ROBOT_NAME` environment variable.

7. **The Zenoh bridge config template must use `__ROBOT_NS__` (double underscores on both sides) as the placeholder**, matching what `gen_zenoh_config.sh` substitutes with `sed`.

8. **Do not create `/etc/ausra/namespace` or `/etc/ausra/namespace.env` as static files.** These are written at runtime by `ns_resolver.py`. Only create the `/etc/ausra/` directory (or a `.gitkeep` placeholder).

---

## Validation checklist — verify before finishing

After creating all files, confirm:

- [ ] `ausra_msgs/CMakeLists.txt` registers all three interface files (`RobotStatus.msg`, `FleetStatus.msg`, `SendTask.action`)
- [ ] `ausra_msgs/msg/RobotStatus.msg` contains `state uint8` and `degraded_systems string[]`
- [ ] `ns_resolver.py` imports `sdnotify` and calls `notifier.notify("READY=1")` at the end
- [ ] `ns_resolver.py` handles the `ausra_fallback` case (Wi-Fi unavailable at boot)
- [ ] Every systemd service file has `EnvironmentFile=/etc/ausra/namespace.env` (except `ausra-ns-resolver.service`, which runs before the file exists)
- [ ] `ausra-ns-resolver.service` has `Type=notify` (NOT `Type=simple`) — this is what makes downstream services wait for the namespace to be written
- [ ] `ausra-micro-ros.service` has `ExecStartPre=` pointing to `send_namespace_to_esp32.sh`
- [ ] `ausra-zenoh-bridge.service` has `ExecStartPre=` pointing to `gen_zenoh_config.sh`
- [ ] `robot_bridge.json5` does NOT list `odom`, `scan`, `tf`, `joint_states`, `cmd_vel`, or `filtered_odometry` in any `pub_topics` or `sub_topics`
- [ ] `supervisor_node.py` entry point is registered in `setup.py`
- [ ] `install_jetson.sh` calls `systemctl enable` for all five services
- [ ] `ausra_bringup/CMakeLists.txt` installs the `scripts/` directory and marks `.sh` files and `ns_resolver.py` as executable via `install(PROGRAMS ...)`