"""
AUSRA Supervisor Node — Per-robot sidecar that runs on each Jetson.

Responsibilities:
  - Topic health watchdogs (odom, scan, filtered_odometry)
  - State machine: IDLE → NAVIGATING / EXPLORING → DEGRADED → ESTOP
  - Nav2 NavigateToPose action client with retry logic
  - Fleet heartbeat and status publishing
  - Emergency stop handling (zero-velocity flood)
  - Frontier exploration lifecycle (subprocess management of explore_lite)

State enum (uint8 state in RobotStatus.msg — must match fleet-side):
  0 = IDLE
  1 = NAVIGATING
  2 = DEGRADED
  3 = ESTOP
  4 = LOST (fleet-side only, never set by this node)
  5 = EXPLORING
"""

import math
import subprocess
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Header
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import MarkerArray

from ausra_msgs.msg import RobotStatus

from ausra_supervisor.topic_watchdog import TopicWatchdog


# ── State constants ──────────────────────────────────────────
# Full enum (robot-side + fleet-side): must match RobotStatus.msg comment
STATE_IDLE       = 0
STATE_NAVIGATING = 1
STATE_DEGRADED   = 2
STATE_ESTOP      = 3
# STATE_LOST = 4 is fleet-side only — this node never sets it
STATE_EXPLORING  = 5

_STATE_NAMES = {
    STATE_IDLE:       "IDLE",
    STATE_NAVIGATING: "NAVIGATING",
    STATE_DEGRADED:   "DEGRADED",
    STATE_ESTOP:      "ESTOP",
    STATE_EXPLORING:  "EXPLORING",
}


class SupervisorNode(Node):
    """Per-robot supervisor sidecar: health, task FSM, e-stop, heartbeat, exploration."""

    def __init__(self):
        super().__init__('supervisor_node')

        # ── Declare parameters ───────────────────────────────
        self.declare_parameter('robot_name',            'ausra_1')
        self.declare_parameter('max_retries',           2)
        self.declare_parameter('watchdog_timeout_odom',  0.5)
        self.declare_parameter('watchdog_timeout_scan',  0.4)
        self.declare_parameter('watchdog_timeout_ekf',   0.5)
        self.declare_parameter('heartbeat_hz',           2.0)
        self.declare_parameter('status_hz',              1.0)
        self.declare_parameter('estop_duration_sec',     3.0)

        # Exploration parameters
        self.declare_parameter('explore_params_file',           '')
        # Full path to lidar_slam_pkg/config/explore_params.yaml
        # Resolved by the systemd ExecStart via 'ros2 pkg prefix lidar_slam_pkg'
        self.declare_parameter('frontier_idle_timeout_sec',     15.0)
        # Seconds without new frontiers before exploration is declared complete
        self.declare_parameter('explore_costmap_topic',         'global_costmap/costmap')
        self.declare_parameter('explore_costmap_updates_topic', 'global_costmap/costmap_updates')

        # ── Read parameters ──────────────────────────────────
        self._robot_name   = self.get_parameter('robot_name').get_parameter_value().string_value
        self._max_retries  = self.get_parameter('max_retries').get_parameter_value().integer_value
        timeout_odom       = self.get_parameter('watchdog_timeout_odom').get_parameter_value().double_value
        timeout_scan       = self.get_parameter('watchdog_timeout_scan').get_parameter_value().double_value
        timeout_ekf        = self.get_parameter('watchdog_timeout_ekf').get_parameter_value().double_value
        heartbeat_hz       = self.get_parameter('heartbeat_hz').get_parameter_value().double_value
        status_hz          = self.get_parameter('status_hz').get_parameter_value().double_value
        self._estop_dur    = self.get_parameter('estop_duration_sec').get_parameter_value().double_value

        ns = self._robot_name
        self.get_logger().info(f"SupervisorNode starting for robot: {ns}")

        # ── State machine ────────────────────────────────────
        self._state = STATE_IDLE
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._pose_yaw = 0.0
        self._battery_pct = 100.0   # placeholder — ESP32 may report real values later
        self._active_task_id = ''
        self._degraded_systems: list = []
        self._retries = 0
        self._last_goal: PoseStamped = None
        self._nav_goal_handle = None
        self._estop_timer = None
        self._estop_start: float = 0.0

        # ── Exploration state ────────────────────────────────
        self._explore_process = None       # subprocess.Popen handle for explore_lite
        self._last_frontier_time = 0.0    # time.monotonic() of last non-empty MarkerArray
        self._frontier_sub = None          # created/destroyed dynamically

        # ── Watchdogs ────────────────────────────────────────
        self._watchdogs = {
            'odom': TopicWatchdog(self, f'/{ns}/odom',              Odometry,   timeout_odom),
            'scan': TopicWatchdog(self, f'/{ns}/scan',              LaserScan,  timeout_scan),
            'ekf':  TopicWatchdog(self, f'/{ns}/filtered_odometry', Odometry,   timeout_ekf),
        }

        # ── Subscriptions ────────────────────────────────────
        self._sub_odom = self.create_subscription(
            Odometry,
            f'/{ns}/filtered_odometry',
            self._on_filtered_odom,
            10,
        )

        self._sub_task_goal = self.create_subscription(
            PoseStamped,
            f'/{ns}/supervisor/task_goal',
            self._on_task_goal,
            10,
        )

        # Fleet-wide e-stop — all robots listen on the same topic
        self._sub_estop = self.create_subscription(
            Bool,
            '/fleet/estop_all',
            self._on_estop,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1,
            ),
        )

        # Per-robot exploration command topic: /{ns}/supervisor/explore_cmd (std_msgs/Bool)
        # true  = start exploration, false = stop exploration
        self.create_subscription(
            Bool,
            f'/{ns}/supervisor/explore_cmd',
            self._on_explore_cmd,
            10,
        )

        # Fleet-wide "all robots start/stop exploring" broadcast
        self.create_subscription(
            Bool,
            '/fleet/explore_all',
            self._on_explore_cmd,
            10,
        )

        # ── Nav2 action client ───────────────────────────────
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            f'/{ns}/navigate_to_pose',
        )

        # ── Publishers ───────────────────────────────────────
        self._pub_status = self.create_publisher(
            RobotStatus,
            f'/{ns}/supervisor/status',
            10,
        )

        self._pub_heartbeat = self.create_publisher(
            Header,
            f'/{ns}/supervisor/heartbeat',
            10,
        )

        self._pub_cmd_vel = self.create_publisher(
            Twist,
            f'/{ns}/cmd_vel',
            10,
        )

        # ── Timers ───────────────────────────────────────────
        if heartbeat_hz > 0:
            self.create_timer(1.0 / heartbeat_hz, self._publish_heartbeat)
        if status_hz > 0:
            self.create_timer(1.0 / status_hz, self._publish_status)

        # Exploration completion check — 1 Hz
        self.create_timer(1.0, self._check_exploration_progress)

        self.get_logger().info(
            f"SupervisorNode ready: watchdogs={list(self._watchdogs.keys())}, "
            f"heartbeat={heartbeat_hz}Hz, status={status_hz}Hz"
        )

    # ── Pose tracking ────────────────────────────────────────

    def _on_filtered_odom(self, msg: Odometry) -> None:
        """Extract pose from filtered odometry (EKF output)."""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self._pose_x = pos.x
        self._pose_y = pos.y
        self._pose_yaw = self._quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)

    @staticmethod
    def _quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
        """Convert quaternion to yaw (rotation around Z axis)."""
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    # ── Task handling ────────────────────────────────────────

    def _on_task_goal(self, msg: PoseStamped) -> None:
        """Handle incoming task goal from the fleet supervisor."""
        if self._state in (STATE_ESTOP, STATE_EXPLORING):
            self.get_logger().warn(
                f"Task goal rejected: robot is in {_STATE_NAMES.get(self._state, self._state)} state."
            )
            return
        if self._state == STATE_NAVIGATING:
            self.get_logger().warn("Task goal rejected: already navigating.")
            return

        self._last_goal = msg
        self._retries = 0
        self._send_nav_goal(msg)

    def _send_nav_goal(self, pose: PoseStamped) -> None:
        """Send a NavigateToPose goal to Nav2."""
        self._state = STATE_NAVIGATING
        self._active_task_id = uuid.uuid4().hex[:8]

        self.get_logger().info(
            f"Sending nav goal (task={self._active_task_id}, "
            f"retry={self._retries}/{self._max_retries}): "
            f"x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}"
        )

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self._handle_failure("Nav2 action server unavailable")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        send_future = self._nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        """Called when Nav2 accepts or rejects the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._handle_failure("Goal rejected by Nav2")
            return

        self.get_logger().info(f"Goal accepted (task={self._active_task_id})")
        self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
        """Called when Nav2 finishes (success or failure)."""
        result = future.result()
        status = result.status

        # action_msgs/GoalStatus: STATUS_SUCCEEDED = 4
        if status == 4:
            self.get_logger().info(
                f"Navigation succeeded (task={self._active_task_id})"
            )
            self._state = STATE_IDLE
            self._active_task_id = ''
            self._retries = 0
            self._nav_goal_handle = None
        else:
            self._handle_failure(f"Nav2 finished with status {status}")

    def _handle_failure(self, reason: str) -> None:
        """Handle navigation failure with retry logic."""
        self.get_logger().warn(
            f"Navigation failure: {reason} "
            f"(retry {self._retries}/{self._max_retries})"
        )
        self._nav_goal_handle = None

        if self._retries < self._max_retries:
            self._retries += 1
            if self._last_goal is not None:
                self._send_nav_goal(self._last_goal)
        else:
            self.get_logger().error(
                f"Max retries ({self._max_retries}) reached — giving up on task "
                f"{self._active_task_id}"
            )
            self._state = STATE_IDLE
            self._active_task_id = ''

    # ── E-Stop ───────────────────────────────────────────────

    def _on_estop(self, msg: Bool) -> None:
        """Handle fleet-wide emergency stop."""
        if not msg.data:
            # Release signal — return to IDLE (robot waits for new goal)
            return

        self.get_logger().warn("E-STOP ACTIVATED — halting robot!")

        # Terminate any running explore_lite subprocess without resetting
        # state back to IDLE — ESTOP takes priority over EXPLORING.
        if self._explore_process is not None:
            self._explore_process.terminate()
            self._explore_process = None
        if self._frontier_sub is not None:
            self.destroy_subscription(self._frontier_sub)
            self._frontier_sub = None

        self._state = STATE_ESTOP

        # Cancel any active Nav2 goal
        if self._nav_goal_handle is not None:
            self.get_logger().info("Cancelling active Nav2 goal...")
            cancel_future = self._nav_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda f: self.get_logger().info("Nav2 goal cancel acknowledged.")
            )
            self._nav_goal_handle = None

        # Flood zero-velocity commands to ensure the robot stops
        self._estop_start = time.monotonic()
        if self._estop_timer is not None:
            self._estop_timer.cancel()
        self._estop_timer = self.create_timer(0.1, self._estop_tick)  # 10 Hz

    def _estop_tick(self) -> None:
        """Publish zero-velocity Twist at 10 Hz during e-stop duration."""
        elapsed = time.monotonic() - self._estop_start
        if elapsed >= self._estop_dur:
            # E-stop duration elapsed — stop flooding, transition to IDLE
            if self._estop_timer is not None:
                self._estop_timer.cancel()
                self._estop_timer = None
            self._state = STATE_IDLE
            self.get_logger().info(
                f"E-stop flood ended after {self._estop_dur:.1f}s — returning to IDLE."
            )
            return

        # Publish zero velocity
        self._pub_cmd_vel.publish(Twist())

    # ── Exploration ──────────────────────────────────────────

    def _on_explore_cmd(self, msg: Bool) -> None:
        """Handle per-robot or fleet-wide exploration command."""
        if msg.data:
            self._start_exploration()
        else:
            self._stop_exploration(reason='manual stop command')

    def _start_exploration(self) -> None:
        """Start the explore_lite subprocess and enter EXPLORING state."""
        if self._state in (STATE_NAVIGATING, STATE_ESTOP):
            self.get_logger().warn(
                f"Cannot start exploration in state "
                f"{_STATE_NAMES.get(self._state, self._state)} — ignoring."
            )
            return
        if self._state == STATE_EXPLORING:
            self.get_logger().info("Already in EXPLORING state — ignoring.")
            return

        ns = self._robot_name
        params_file = self.get_parameter('explore_params_file').get_parameter_value().string_value
        base_frame = f'{ns}_robot_footprint'
        costmap_topic = self.get_parameter('explore_costmap_topic').get_parameter_value().string_value
        costmap_updates_topic = self.get_parameter('explore_costmap_updates_topic').get_parameter_value().string_value

        # Frontier marker topic verified against source:
        # m-explore-ros2 (installed in /home/ausranano/ausra_ws) at
        # Navigation/m-explore-ros2/explore/src/explore.cpp line 94-97:
        # create_publisher<MarkerArray>("explore/frontiers", 10)
        # This is a relative topic → becomes /{ns}/explore/frontiers under namespace remap.

        cmd = [
            'ros2', 'run', 'explore_lite', 'explore',
            '--ros-args',
            '-r', f'__ns:=/{ns}',
            '-p', 'use_sim_time:=false',
            '-p', f'robot_base_frame:={base_frame}',
            '-p', f'costmap_topic:={costmap_topic}',
            '-p', f'costmap_updates_topic:={costmap_updates_topic}',
        ]

        if params_file:
            cmd.extend(['--params-file', params_file])
        else:
            self.get_logger().warn(
                "explore_params_file is empty — starting explore_lite without params file. "
                "Pass -p explore_params_file:=<path> to the supervisor node."
            )

        self._state = STATE_EXPLORING
        self._explore_process = subprocess.Popen(cmd)
        self._last_frontier_time = time.monotonic()

        # Subscribe to frontier markers to detect exploration completion.
        # explore_lite (m-explore-ros2) publishes MarkerArray on relative topic
        # "explore/frontiers" → /{ns}/explore/frontiers under the namespace remap.
        self._frontier_sub = self.create_subscription(
            MarkerArray,
            f'/{ns}/explore/frontiers',
            self._on_frontier_markers,
            10,
        )

        self.get_logger().info(
            f"[{ns}] Exploration started (pid={self._explore_process.pid})"
        )

    def _on_frontier_markers(self, msg: MarkerArray) -> None:
        """Record timestamp whenever frontiers are still visible."""
        if len(msg.markers) > 0:
            self._last_frontier_time = time.monotonic()

    def _check_exploration_progress(self) -> None:
        """Called at 1 Hz — stop exploration if no frontiers seen for timeout."""
        if self._state != STATE_EXPLORING:
            return
        timeout = self.get_parameter('frontier_idle_timeout_sec').get_parameter_value().double_value
        elapsed = time.monotonic() - self._last_frontier_time
        if elapsed > timeout:
            self.get_logger().info(
                f"No frontiers detected for {elapsed:.1f}s (timeout={timeout:.1f}s). "
                "Exploration complete."
            )
            self._stop_exploration(reason='exploration complete (no frontiers)')

    def _stop_exploration(self, reason: str) -> None:
        """Terminate the explore_lite subprocess and return to IDLE."""
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

        if self._state == STATE_EXPLORING:
            self._state = STATE_IDLE

        self.get_logger().info(f"Exploration stopped: {reason}")

    # ── Health check ─────────────────────────────────────────

    def _check_health(self) -> None:
        """Evaluate watchdog health and update state if needed."""
        degraded = [
            name for name, wd in self._watchdogs.items()
            if not wd.healthy
        ]
        self._degraded_systems = degraded

        if degraded and self._state == STATE_ESTOP:
            pass  # ESTOP takes priority — never overridden by health checks
        elif degraded and self._state != STATE_DEGRADED:
            # Transition to DEGRADED from any non-ESTOP state (including EXPLORING).
            # NOTE: the explore_lite subprocess keeps running while DEGRADED.
            # A sensor dropout during EXPLORING surfaces as DEGRADED state.
            # TODO: future improvement — auto-stop exploration on DEGRADED.
            self._state = STATE_DEGRADED
        elif not degraded and self._state == STATE_DEGRADED:
            self._state = STATE_IDLE
        # Never override ESTOP or NAVIGATING/EXPLORING states from health checks
        # when no sensors are degraded (healthy → no change needed).

    # ── Publishers ───────────────────────────────────────────

    def _publish_heartbeat(self) -> None:
        """Publish a lightweight heartbeat (Header with timestamp)."""
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = self._robot_name
        self._pub_heartbeat.publish(msg)

    def _publish_status(self) -> None:
        """Publish full robot status (runs health check first)."""
        self._check_health()

        msg = RobotStatus()
        msg.robot_name = self._robot_name
        msg.state = self._state
        msg.battery_pct = self._battery_pct
        msg.pose_x = self._pose_x
        msg.pose_y = self._pose_y
        msg.pose_yaw = self._pose_yaw
        msg.active_task_id = self._active_task_id
        msg.degraded_systems = self._degraded_systems
        msg.stamp = self.get_clock().now().to_msg()

        self._pub_status.publish(msg)

    # ── Cleanup ──────────────────────────────────────────────

    def destroy_node(self) -> None:
        """Clean up explore_lite subprocess on node shutdown."""
        if self._explore_process is not None:
            self.get_logger().info(
                f"Node shutting down — stopping explore_lite (pid={self._explore_process.pid})"
            )
            self._stop_exploration(reason='node shutdown')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
