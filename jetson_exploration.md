# AUSRA Jetson — Exploration Mode Additions (Agent Prompt)

## Context

This prompt extends an EXISTING AUSRA Jetson installation that already has:
- `ausra_msgs` package (RobotStatus, FleetStatus, SendTask)
- `ausra_supervisor` package with `supervisor_node.py` implementing states `0=IDLE 1=NAVIGATING 2=DEGRADED 3=ESTOP`
- `ausra_bringup` package with `hardware_full_stack.launch.py`, systemd units, Zenoh bridge config
- `explore_lite` package already installed (frontier exploration)

Your task is to add **frontier exploration mode**: a fleet-commandable state where the robot autonomously explores unknown space using `explore_lite`, transitions out of `IDLE` into `EXPLORING`, and returns to `IDLE` when exploration completes or is cancelled.

Read this entire prompt before editing any file.

---

## Design summary

1. A new supervisor state `EXPLORING = 5` is added.
2. The fleet supervisor (laptop side) commands exploration via a new topic `/{robot_name}/supervisor/explore_cmd` (`std_msgs/Bool`).
3. `explore_lite`'s `explore` node is **removed from the always-on launch sequence** (currently Stage 3, auto-starts at 30s in `hardware_full_stack.launch.py`). It becomes a subprocess that the supervisor starts/stops on demand.
4. The supervisor detects exploration completion by monitoring `/{robot_name}/explore/frontiers` (or `/explore/frontiers_marker` if that's what `explore_lite` Humble publishes — verify against the installed `explore_lite` package's source before assuming the topic name). If no frontiers are reported for a configurable timeout, exploration is considered complete.
5. While `EXPLORING`, the supervisor rejects new `task_goal` commands (same as `NAVIGATING`) but DOES respond to `/fleet/estop_all` and to `explore_cmd=false` (manual stop).

---

## Files to modify

### 1. `ausra_bringup/launch/hardware_full_stack.launch.py`

**Remove** the Stage 3 `TimerAction` block that auto-launches `exploration_server` at 30 seconds. Locate this block:

```python
# Stage 3: Exploration (30 s delay)
TimerAction(
    period=30.0,
    actions=[
        GroupAction(actions=[
            PushRosNamespace(robot_name),
            LogInfo(msg=f'>>> [{robot_name}] Stage 3: Starting Frontier Exploration...'),
            exploration_server,
        ])
    ]
),
```

**Delete this entire `TimerAction` block** and remove it from the `namespaced_actions` list. Also delete the now-unused `exploration_server` `Node(...)` definition and the `explore_params_file` variable IF it is not referenced elsewhere — but first check whether `explore_params_file` is still needed by the supervisor's subprocess launch (it is — see step 2 below). If still needed, keep the variable definition but remove only the `exploration_server` Node object and its TimerAction.

**Do not modify** Stage 0, Stage 1, Stage 2, or the nudge logic. Only Stage 3 changes.

After this edit, the launch file should go directly from Stage 2 (Nav2) to the nudge stage (Stage 4), with no Stage 3.

---

### 2. `ausra_supervisor/ausra_supervisor/supervisor_node.py`

#### 2a. Add new state constant

```python
class SupervisorNode(Node):
    STATE_IDLE       = 0
    STATE_NAVIGATING = 1
    STATE_DEGRADED   = 2
    STATE_ESTOP      = 3
    # STATE 4 = LOST is fleet-side only, never set by this node
    STATE_EXPLORING  = 5
```

#### 2b. Add new declared parameters

```python
self.declare_parameter('explore_params_file', '')
# Full path to lidar_slam_pkg/config/explore_params.yaml — passed in via
# the systemd ExecStart so the supervisor can forward it to the
# explore_lite subprocess.

self.declare_parameter('frontier_idle_timeout_sec', 15.0)
# If no new frontier markers are seen for this long while EXPLORING,
# consider exploration complete and stop explore_lite.

self.declare_parameter('explore_costmap_topic', 'global_costmap/costmap')
self.declare_parameter('explore_costmap_updates_topic', 'global_costmap/costmap_updates')
```

#### 2c. Add new subscription in `__init__`

```python
self.create_subscription(Bool, f'/{ns}/supervisor/explore_cmd',
    self._on_explore_cmd, 10)
```

(`Bool` is already imported from `std_msgs.msg` for `_on_estop`.)

#### 2d. Add frontier marker subscription (created lazily — see 2f)

You will need `visualization_msgs.msg.MarkerArray`. Add the import:
```python
from visualization_msgs.msg import MarkerArray
```

Store the subscription handle in `self._frontier_sub = None` (initialized in `__init__`, created/destroyed dynamically).

#### 2e. Add instance fields in `__init__`

```python
self._explore_process = None       # subprocess.Popen handle
self._last_frontier_time = 0.0     # time.monotonic() of last non-empty MarkerArray
self._frontier_sub = None
```

Add `import subprocess` and `import time` (time is likely already imported).

#### 2f. Implement `_on_explore_cmd(self, msg: Bool)`

```python
def _on_explore_cmd(self, msg: Bool):
    if msg.data:
        self._start_exploration()
    else:
        self._stop_exploration(reason='manual stop command')
```

#### 2g. Implement `_start_exploration(self)`

Logic:
- If `_state` is `NAVIGATING` or `ESTOP`: log a warning ("cannot start exploration in current state") and return — do not start.
- If `_state == EXPLORING` already: log info ("already exploring") and return (idempotent).
- Set `_state = STATE_EXPLORING`.
- Build the subprocess command. The robot's namespace is `self.get_parameter('robot_name').value`. Use `ros2 run explore_lite explore` with `--ros-args` remappings so it operates inside the namespace (the supervisor node itself already runs under `-r __ns:=/{robot_name}` per the systemd unit, but a subprocess spawned via `subprocess.Popen` does NOT inherit that remap automatically — you must pass it explicitly):

```python
ns = self.get_parameter('robot_name').value
params_file = self.get_parameter('explore_params_file').value
base_frame = f'{ns}_robot_footprint'
costmap_topic = self.get_parameter('explore_costmap_topic').value
costmap_updates_topic = self.get_parameter('explore_costmap_updates_topic').value

cmd = [
    'ros2', 'run', 'explore_lite', 'explore',
    '--ros-args',
    '-r', f'__ns:=/{ns}',
    '--params-file', params_file,
    '-p', f'use_sim_time:=false',
    '-p', f'robot_base_frame:={base_frame}',
    '-p', f'costmap_topic:={costmap_topic}',
    '-p', f'costmap_updates_topic:={costmap_updates_topic}',
]

self._explore_process = subprocess.Popen(cmd)
self._last_frontier_time = time.monotonic()

# Subscribe to frontier markers to detect completion
self._frontier_sub = self.create_subscription(
    MarkerArray, f'/{ns}/explore/frontiers',
    self._on_frontier_markers, 10)

self.get_logger().info(f'[{ns}] Exploration started (pid={self._explore_process.pid})')
```

**IMPORTANT — verify the actual frontier topic name.** Before finalizing, inspect the installed `explore_lite` source (likely at `/opt/ros/humble/share/explore_lite` or the workspace's `explore_lite` package if it's a source build) for the topic it publishes frontier markers on. The commonly used topic in `explore_lite` is `explore/frontiers` (relative, becomes `/{ns}/explore/frontiers` under the namespace remap). If the installed version differs, update both `_start_exploration` and `_on_frontier_markers` accordingly, and add a code comment noting which version/commit was checked.

#### 2h. Implement `_on_frontier_markers(self, msg: MarkerArray)`

```python
def _on_frontier_markers(self, msg: MarkerArray):
    if len(msg.markers) > 0:
        self._last_frontier_time = time.monotonic()
```

This just records "frontiers still exist" timestamps. The actual completion check happens in a periodic timer (2i).

#### 2i. Add a periodic exploration-completion check

In `__init__`, add a timer (e.g. 1 Hz):
```python
self.create_timer(1.0, self._check_exploration_progress)
```

Implement:
```python
def _check_exploration_progress(self):
    if self._state != self.STATE_EXPLORING:
        return
    timeout = self.get_parameter('frontier_idle_timeout_sec').value
    if (time.monotonic() - self._last_frontier_time) > timeout:
        self.get_logger().info('No frontiers detected for timeout period. Exploration complete.')
        self._stop_exploration(reason='exploration complete (no frontiers)')
```

#### 2j. Implement `_stop_exploration(self, reason: str)`

```python
def _stop_exploration(self, reason: str):
    if self._explore_process is not None:
        self._explore_process.terminate()
        try:
            self._explore_process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self._explore_process.kill()
        self._explore_process = None

    if self._frontier_sub is not None:
        self.destroy_subscription(self._frontier_sub)
        self._frontier_sub = None

    if self._state == self.STATE_EXPLORING:
        self._state = self.STATE_IDLE

    self.get_logger().info(f'Exploration stopped: {reason}')
```

#### 2k. Update `_on_estop` to also stop exploration

In the existing `_on_estop(self, msg)` method, after setting `_state = ESTOP`, add a call to stop any running exploration subprocess WITHOUT resetting state back to IDLE (since e-stop takes priority):

```python
def _on_estop(self, msg):
    if not msg.data:
        return
    if self._explore_process is not None:
        self._explore_process.terminate()
        self._explore_process = None
    if self._frontier_sub is not None:
        self.destroy_subscription(self._frontier_sub)
        self._frontier_sub = None
    self._state = self.STATE_ESTOP
    # ... existing cancel-nav-goal and zero-velocity-flood logic stays unchanged
```

#### 2l. Update `_on_task_goal` to reject goals while exploring

In the existing `_on_task_goal` method, add `EXPLORING` to the rejection check:

```python
def _on_task_goal(self, msg):
    if self._state in (self.STATE_ESTOP, self.STATE_EXPLORING):
        self.get_logger().warn('Task goal rejected: robot is in ESTOP or EXPLORING state.')
        return
    if self._state == self.STATE_NAVIGATING:
        self.get_logger().warn('Task goal rejected: already navigating.')
        return
    # ... existing logic unchanged
```

#### 2m. Update `_check_health` to not override `EXPLORING` incorrectly

In the existing `_check_health` method, the `DEGRADED` transition currently checks `if degraded and self._state not in (self.STATE_ESTOP,)`. Update this to also exclude `EXPLORING` from being silently overridden — but DEGRADED should still be settable while exploring, since a sensor failure during exploration is important to surface. Change the condition to:

```python
def _check_health(self):
    degraded = [name for name, wd in self._watchdogs.items() if not wd.healthy]
    self._degraded_systems = degraded
    if degraded and self._state == self.STATE_ESTOP:
        pass  # ESTOP takes priority, never overridden
    elif degraded and self._state != self.STATE_DEGRADED:
        # Remember what we were doing before degrading, if useful for future recovery.
        # For now, simply transition to DEGRADED from any non-ESTOP state.
        self._state = self.STATE_DEGRADED
    elif not degraded and self._state == self.STATE_DEGRADED:
        self._state = self.STATE_IDLE
```

Note: this means a sensor dropout during EXPLORING will stop reporting EXPLORING and report DEGRADED instead, but the `explore_lite` subprocess will keep running until `_stop_exploration` is explicitly called. Add a TODO comment noting that a future improvement could auto-stop exploration on DEGRADED — but do not implement that now, keep this prompt's scope to the state machine and process lifecycle only.

---

### 3. `ausra_bringup/systemd/ausra-supervisor.service`

Add the new `explore_params_file` parameter to the `ExecStart` line. The file path is inside the installed share directory of `lidar_slam_pkg`. Update the `ExecStart` block:

```ini
ExecStart=/bin/bash -c '\
    source /opt/ros/humble/setup.bash && \
    source /opt/ausra/install/setup.bash && \
    EXPLORE_PARAMS=$(ros2 pkg prefix lidar_slam_pkg)/share/lidar_slam_pkg/config/explore_params.yaml && \
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
        -p estop_duration_sec:=3.0 \
        -p explore_params_file:=${EXPLORE_PARAMS} \
        -p frontier_idle_timeout_sec:=15.0'
```

---

### 4. `ausra_bringup/zenoh/robot_bridge.json5`

Add the new exploration command topic to `sub_topics` (this is a command FROM the fleet supervisor TO the robot, so it's something this robot subscribes to from the shared Zenoh session):

```json5
sub_topics: [
  "/__ROBOT_NS__/supervisor/task_goal",
  "/__ROBOT_NS__/supervisor/explore_cmd",
  "/fleet/estop_all",
  "/fleet/task_broadcast",
  "/fleet/explore_all"
]
```

Also add `/fleet/explore_all` (a fleet-wide "all robots start exploring" broadcast) — the supervisor does not need a new subscription for this; instead, ADD a subscription to `/fleet/explore_all` (`std_msgs/Bool`) in `supervisor_node.py` `__init__` that calls the same `_on_explore_cmd` handler:

```python
self.create_subscription(Bool, '/fleet/explore_all',
    self._on_explore_cmd, 10)
```

Add this alongside the existing `explore_cmd` subscription in step 2c.

---

## Constraints

1. **Do not change the SLAM, EKF, Nav2, or driver node configurations.** Only the exploration lifecycle and supervisor state machine change.
2. **`explore_lite` must run as a subprocess managed by the supervisor**, not as a `Node(...)` in the launch file. This is the core architectural change.
3. **Verify the actual `explore_lite` Humble frontier topic name** against the installed package before finalizing — do not assume `explore/frontiers` is correct without checking, since this varies by `explore_lite` fork/version. Document which version was checked in a code comment.
4. **State `EXPLORING = 5` must not collide with the fleet-side `LOST = 4`** — these are in the same `uint8 state` field of `RobotStatus.msg`, so both robot-side and fleet-side code must agree on the full enum: `0=IDLE 1=NAVIGATING 2=DEGRADED 3=ESTOP 4=LOST 5=EXPLORING`.
5. **Subprocess cleanup must be defensive.** If `supervisor_node` is killed/restarted by systemd while `explore_lite` is running, the orphaned subprocess will keep running. Add a `destroy_node` override (or use `rclpy.shutdown` hook) that calls `_stop_exploration('node shutdown')` if `_explore_process is not None`.

---

## Validation checklist

- [ ] `hardware_full_stack.launch.py` no longer auto-starts `explore_lite` at any fixed delay
- [ ] `supervisor_node.py` defines `STATE_EXPLORING = 5` and the full enum is documented in a comment matching `ausra_msgs/msg/RobotStatus.msg`
- [ ] `_start_exploration` rejects starting from `NAVIGATING` and `ESTOP` states
- [ ] `_on_task_goal` rejects new goals while `EXPLORING`
- [ ] `_on_estop` terminates the `explore_lite` subprocess
- [ ] Frontier topic name is verified against installed `explore_lite` and documented
- [ ] `ausra-supervisor.service` resolves `explore_params_file` via `ros2 pkg prefix`
- [ ] `robot_bridge.json5` includes `explore_cmd` and `/fleet/explore_all` in `sub_topics`
- [ ] Node shutdown cleanly terminates any running `explore_lite` subprocess