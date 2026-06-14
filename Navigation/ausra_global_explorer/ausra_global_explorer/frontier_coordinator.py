#!/usr/bin/env python3
"""
frontier_coordinator.py — Centralized global frontier coordinator for the AUSRA swarm.

Simulation-adapted version. Key simulation facts (verified from the codebase):

  * Merged map topic  : /map_merged
      Source: ausra_map_merge/config/map_merge_params.yaml → merged_map_topic
  * Robot base frame  : {robot_name}_robot_footprint
      Source: spawn_ausra_full.launch.py → get_omni_driver_params() base_frame_id
              and exploration params robot_base_frame
  * Nav2 action server: /{robot_name}/navigate_to_pose
      Source: spawn_ausra_full.launch.py — all Nav2 nodes are namespace=robot_name
  * Per-robot explore : ausra_frontier_exploration / exploration_server_enhanced
      Source: spawn_ausra_full.launch.py → controlled by use_exploration arg

Architecture:
  One instance of this node runs on the "base station" (or any machine that can
  see /map_merged and the TF tree).  It is the *only* entity assigning Nav2 goals.
  Each robot's local explore_lite / ausra_frontier_exploration MUST be disabled
  (set use_exploration:=false when launching robots).

Topic / action graph:
  Subscriptions:
    /map_merged                    → OccupancyGrid (merged SLAM canvas)
  Publications:
    /frontier_coordinator/frontiers → MarkerArray (RViz2 visualization)
  Action clients:
    /{robot}/navigate_to_pose      → nav2_msgs/action/NavigateToPose
  TF lookups:
    map → {robot}_robot_footprint  → live global position of each robot
"""

import math
import time
from collections import deque
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray


# ─────────────────────────────────────────────────────────────────────────────
# Data classes
# ─────────────────────────────────────────────────────────────────────────────

class Frontier:
    """A detected frontier cluster: centroid in map frame + cell count."""

    __slots__ = ('x', 'y', 'size')

    def __init__(self, x: float, y: float, size: int):
        self.x = x
        self.y = y
        self.size = size

    def dist_to(self, px: float, py: float) -> float:
        return math.hypot(self.x - px, self.y - py)

    def __repr__(self):
        return f'Frontier(x={self.x:.2f}, y={self.y:.2f}, size={self.size})'


class RobotState:
    """Tracks one robot's assignment state."""

    def __init__(self, name: str):
        self.name = name
        # Simulation frame: confirmed from get_omni_driver_params() in
        # spawn_ausra_full.launch.py (base_frame_id = f'{robot_name}_robot_footprint')
        self.base_frame: str = f'{name}_robot_footprint'
        self.is_idle: bool = True
        self.current_goal: Optional[Frontier] = None
        self.goal_handle: Optional[ClientGoalHandle] = None
        self.goal_start_time: float = 0.0
        # How many times in a row this robot got a trivial success (already-at-goal)
        # When this reaches 3, we force a bootstrap move to seed the SLAM map.
        self.trivial_count: int = 0
        # Monotonically increasing sequence number bumped on every new goal send.
        # Captured in the callback closure so stale callbacks (from a cancelled goal
        # that arrives after a new goal was already assigned) can be discarded safely.
        self.goal_seq: int = 0

    def __repr__(self):
        state = 'IDLE' if self.is_idle else f'→({self.current_goal.x:.1f},{self.current_goal.y:.1f})'
        return f'RobotState({self.name}: {state})'


# ─────────────────────────────────────────────────────────────────────────────
# Main node
# ─────────────────────────────────────────────────────────────────────────────

class FrontierCoordinator(Node):
    """
    Centralized frontier coordinator.

    Runs a planning loop at `planning_rate_hz`.  Each cycle:
      1. Cancels robots that have been navigating for > progress_timeout_s
      2. Detects frontier clusters in /map_merged using a NumPy BFS
      3. Assigns the best (closest, largest) unassigned frontier to each idle robot
    """

    def __init__(self):
        super().__init__('frontier_coordinator')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('robot_names', 'ausra_1,ausra_2')
        self.declare_parameter('map_topic', '/map_merged')
        self.declare_parameter('planning_rate_hz', 0.5)
        self.declare_parameter('min_frontier_cells', 5)
        self.declare_parameter('blacklist_radius_m', 0.5)
        self.declare_parameter('progress_timeout_s', 30.0)
        self.declare_parameter('free_threshold', 50)
        self.declare_parameter('visualize', True)
        # Minimum distance (m) a frontier must be from the robot to be assigned.
        # Prevents assigning spawn-point frontiers that Nav2 considers "already reached".
        self.declare_parameter('min_frontier_dist_m', 1.5)
        # If a goal SUCCEEDS in less than this many seconds it was trivial (robot
        # was already there). Blacklist the frontier so it is never re-assigned.
        self.declare_parameter('trivial_goal_s', 5.0)
        # Minimum distance (m) a frontier must be from ANY OTHER robot's position
        # or that robot's active goal. Prevents head-on routing.
        self.declare_parameter('min_inter_robot_dist_m', 1.5)

        robot_names_str: str = self.get_parameter('robot_names').value
        self._robot_names: List[str] = [n.strip() for n in robot_names_str.split(',') if n.strip()]
        self._map_topic: str = self.get_parameter('map_topic').value
        self._planning_rate_hz: float = self.get_parameter('planning_rate_hz').value
        self._min_frontier_cells: int = self.get_parameter('min_frontier_cells').value
        self._blacklist_radius: float = self.get_parameter('blacklist_radius_m').value
        self._progress_timeout: float = self.get_parameter('progress_timeout_s').value
        self._free_threshold: int = self.get_parameter('free_threshold').value
        self._visualize: bool = self.get_parameter('visualize').value
        self._min_frontier_dist: float = self.get_parameter('min_frontier_dist_m').value
        self._trivial_goal_s: float = self.get_parameter('trivial_goal_s').value
        self._min_inter_robot_dist: float = self.get_parameter('min_inter_robot_dist_m').value

        # ── State ─────────────────────────────────────────────────────────────
        self._map: Optional[OccupancyGrid] = None
        self._robots: Dict[str, RobotState] = {n: RobotState(n) for n in self._robot_names}
        # Blacklist: list of (x, y) positions that have failed
        self._blacklist: List[Tuple[float, float]] = []

        # ── TF ────────────────────────────────────────────────────────────────
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── Action clients (one per robot) ────────────────────────────────────
        self._action_clients: Dict[str, ActionClient] = {}
        for name in self._robot_names:
            # Action topic: /{robot_name}/navigate_to_pose
            # Confirmed: Nav2 nodes all use namespace=robot_name in spawn_ausra_full.launch.py
            client = ActionClient(self, NavigateToPose, f'/{name}/navigate_to_pose')
            self._action_clients[name] = client

        # ── Subscriptions ─────────────────────────────────────────────────────
        self._map_sub = self.create_subscription(
            OccupancyGrid,
            self._map_topic,
            self._on_map,
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value,
        )

        # ── Visualization publisher ───────────────────────────────────────────
        self._marker_pub = self.create_publisher(
            MarkerArray,
            '/frontier_coordinator/frontiers',
            10,
        )

        # ── Planning timer ────────────────────────────────────────────────────
        period = 1.0 / self._planning_rate_hz
        self._plan_timer = self.create_timer(period, self._plan)

        self.get_logger().info(
            f'[FrontierCoordinator] Starting — robots: {self._robot_names} | '
            f'map: {self._map_topic} | rate: {self._planning_rate_hz} Hz'
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Map callback
    # ─────────────────────────────────────────────────────────────────────────

    def _on_map(self, msg: OccupancyGrid):
        self._map = msg

    # ─────────────────────────────────────────────────────────────────────────
    # Planning loop
    # ─────────────────────────────────────────────────────────────────────────

    def _plan(self):
        if self._map is None:
            self.get_logger().warn('[FrontierCoordinator] Waiting for /map_merged …')
            return

        now = time.monotonic()

        # 1. Cancel stuck robots
        for robot in self._robots.values():
            if not robot.is_idle and robot.goal_handle is not None:
                elapsed = now - robot.goal_start_time
                if elapsed > self._progress_timeout:
                    self.get_logger().warn(
                        f'[{robot.name}] Timed out after {elapsed:.0f}s — cancelling goal'
                    )
                    robot.goal_handle.cancel_goal_async()
                    if robot.current_goal:
                        self._blacklist.append((robot.current_goal.x, robot.current_goal.y))
                    robot.is_idle = True
                    robot.current_goal = None
                    robot.goal_handle = None

        # 2. Detect frontiers
        frontiers = self._find_frontiers(self._map)
        self.get_logger().info(f'[FrontierCoordinator] Found {len(frontiers)} frontier(s)')

        # Filter blacklisted frontiers
        frontiers = [
            f for f in frontiers
            if not any(
                math.hypot(f.x - bx, f.y - by) < self._blacklist_radius
                for bx, by in self._blacklist
            )
        ]

        # Collect frontiers already assigned to busy robots
        assigned_positions = set()
        for robot in self._robots.values():
            if not robot.is_idle and robot.current_goal:
                assigned_positions.add((robot.current_goal.x, robot.current_goal.y))

        # Remove already-assigned frontiers
        available_frontiers = [
            f for f in frontiers
            if (round(f.x, 3), round(f.y, 3)) not in {
                (round(ax, 3), round(ay, 3)) for ax, ay in assigned_positions
            }
        ]

        if self._visualize:
            self._publish_markers(frontiers, assigned_positions)

        if not available_frontiers:
            all_idle = all(r.is_idle for r in self._robots.values())
            if all_idle:
                self.get_logger().info('[FrontierCoordinator] No frontiers — exploration complete!')
            return

        # 3. Assign idle robots
        for robot in self._robots.values():
            if not robot.is_idle:
                continue
            if not available_frontiers:
                break

            # Get robot's current position
            pose = self._get_robot_pose(robot)
            if pose is None:
                self.get_logger().warn(f'[{robot.name}] Cannot get TF pose — skipping')
                continue

            rx, ry = pose

            # Filter: skip frontiers that are too close to this robot.
            # These are spawn-point frontiers — Nav2 will "succeed" immediately
            # without the robot actually moving, causing an infinite loop.
            reachable = [
                f for f in available_frontiers
                if f.dist_to(rx, ry) >= self._min_frontier_dist
            ]

            if not reachable:
                # ── Bootstrap mode (mirrors old exploration_server_enhanced.cpp L398-451) ──
                # All frontiers are within the robot's immediate vicinity — SLAM hasn't
                # built enough map yet. Force a random 1.5m movement to expand the map.
                robot.trivial_count += 1
                self.get_logger().info(
                    f'[{robot.name}] All frontiers within {self._min_frontier_dist:.1f}m '
                    f'(trivial_count={robot.trivial_count}) — '
                    f'waiting for map to grow'
                )

                if robot.trivial_count >= 2:   # was 3 — reduced so 3rd robot starts faster
                    # Force a bootstrap goal to seed SLAM — pick the direction of the
                    # largest frontier cluster so we move INTO unexplored space, not toward walls.
                    bootstrap_dist = 1.5
                    if available_frontiers:
                        # Head toward the biggest frontier (most unexplored space in that direction)
                        biggest = max(available_frontiers, key=lambda f: f.size)
                        dx = biggest.x - rx
                        dy = biggest.y - ry
                        d = math.hypot(dx, dy) or 1.0
                        bx = rx + (dx / d) * bootstrap_dist
                        by = ry + (dy / d) * bootstrap_dist
                    else:
                        # No frontiers at all — rotate in place (Nav2 spin recovery)
                        import random
                        angle = random.uniform(0, 2 * math.pi)
                        bx = rx + bootstrap_dist * math.cos(angle)
                        by = ry + bootstrap_dist * math.sin(angle)

                    bootstrap = Frontier(bx, by, 0)  # size=0 marks it as bootstrap

                    self.get_logger().warn(
                        f'[{robot.name}] Bootstrap mode: forcing move to '
                        f'({bx:.2f}, {by:.2f}) to seed SLAM map'
                    )
                    # NOTE: trivial_count is reset in _send_goal's is_idle=False path.
                    # DO NOT reset here — if the action server is unavailable,
                    # _send_goal returns early without setting is_idle=False,
                    # and trivial_count=0 would cause a permanent wait loop.
                    self._send_goal(robot, bootstrap)
                continue

            # Real frontier available — reset trivial counter
            robot.trivial_count = 0

            # ── Option 2: inter-robot separation filter ────────────────────────
            # Only filter frontiers that are within min_inter_robot_dist of another
            # robot's LIVE BODY position. This prevents head-on collisions without
            # blocking all frontiers at startup when robots are spawned close together.
            #
            # Separate smaller check: prevent two robots being sent to the SAME frontier
            # (already handled by available_frontiers.remove(best) below, but also guard
            # against active goals being too close — use a tighter 0.5m radius).
            other_body_positions = []
            other_goal_positions = []
            for other in self._robots.values():
                if other.name == robot.name:
                    continue
                other_pose = self._get_robot_pose(other)
                if other_pose is not None:
                    other_body_positions.append(other_pose)
                if other.current_goal is not None:
                    other_goal_positions.append((other.current_goal.x, other.current_goal.y))

            safe_frontiers = [
                f for f in reachable
                if all(
                    math.hypot(f.x - ox, f.y - oy) >= self._min_inter_robot_dist
                    for (ox, oy) in other_body_positions
                )
                and all(
                    math.hypot(f.x - gx, f.y - gy) >= 0.5   # don't overlap active goals
                    for (gx, gy) in other_goal_positions
                )
            ]

            if not safe_frontiers:
                # Deadlock guard: if ALL frontiers are blocked (e.g. robots very close
                # at startup), fall back to the full reachable list so robots can still move.
                # Option 1+3 (costmap + collision monitor) will handle physical safety.
                self.get_logger().info(
                    f'[{robot.name}] All frontiers filtered by inter-robot check '
                    f'(min={self._min_inter_robot_dist:.1f}m) — using best available fallback'
                )
                safe_frontiers = reachable  # fallback: assign anyway, let Nav2 handle it

            reachable = safe_frontiers  # use filtered list for scoring

            # Score: maximize size / distance (avoid division by zero)
            def score(f: Frontier) -> float:
                d = f.dist_to(rx, ry)
                return f.size / (d + 0.01)

            # ── Action server availability check (non-blocking) ──────────────
            # Check BEFORE picking a frontier. If Nav2 is not up for this robot,
            # skip it so the frontier stays available for other robots this cycle.
            client = self._action_clients[robot.name]
            if not client.server_is_ready():
                self.get_logger().warn(
                    f'[{robot.name}] navigate_to_pose not ready — skipping (Nav2 still loading?)'
                )
                continue

            best = max(reachable, key=score)
            # NOTE: available_frontiers.remove(best) is now done inside _send_goal
            # AFTER robot.is_idle=False is confirmed, so a failed send doesn't
            # waste a frontier slot for other robots.

            self.get_logger().info(
                f'[ASSIGN] {robot.name} → frontier ({best.x:.2f}, {best.y:.2f}) '
                f'| size={best.size} cells | dist={best.dist_to(rx, ry):.2f}m'
            )
            self._send_goal(robot, best, available_frontiers)

    # ─────────────────────────────────────────────────────────────────────────
    # Frontier detection — NumPy BFS
    # ─────────────────────────────────────────────────────────────────────────

    def _find_frontiers(self, grid: OccupancyGrid) -> List[Frontier]:
        """
        Detect frontier cells: free cells (0 ≤ value < free_threshold) that are
        adjacent (4-connected) to at least one unknown cell (value == -1, stored
        as 255 in uint8 array).

        Returns a list of Frontier objects (BFS-clustered centroids).
        """
        w = grid.info.width
        h = grid.info.height
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y

        data = np.array(grid.data, dtype=np.int8).reshape(h, w)

        # Free mask: 0 ≤ value < free_threshold
        free_mask = (data >= 0) & (data < self._free_threshold)
        # Unknown mask: value == -1
        unknown_mask = data == -1

        # Frontier cells: free AND adjacent to unknown
        # Shift unknown mask in 4 directions to find adjacency
        adj_unknown = (
            np.roll(unknown_mask, 1, axis=0)   # shift down
            | np.roll(unknown_mask, -1, axis=0) # shift up
            | np.roll(unknown_mask, 1, axis=1)  # shift right
            | np.roll(unknown_mask, -1, axis=1) # shift left
        )
        frontier_mask = free_mask & adj_unknown

        frontier_indices = list(zip(*np.where(frontier_mask)))
        if not frontier_indices:
            return []

        # BFS clustering
        visited = np.zeros((h, w), dtype=bool)
        clusters: List[Frontier] = []

        for (r, c) in frontier_indices:
            if visited[r, c]:
                continue
            # BFS from this seed
            cluster_cells = []
            q = deque([(r, c)])
            visited[r, c] = True
            while q:
                cr, cc = q.popleft()
                cluster_cells.append((cr, cc))
                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nr, nc = cr + dr, cc + dc
                    if 0 <= nr < h and 0 <= nc < w and not visited[nr, nc] and frontier_mask[nr, nc]:
                        visited[nr, nc] = True
                        q.append((nr, nc))

            if len(cluster_cells) < self._min_frontier_cells:
                continue

            # Centroid in map frame
            mean_r = sum(rc for rc, _ in cluster_cells) / len(cluster_cells)
            mean_c = sum(cc for _, cc in cluster_cells) / len(cluster_cells)
            cx = ox + (mean_c + 0.5) * res
            cy = oy + (mean_r + 0.5) * res
            clusters.append(Frontier(cx, cy, len(cluster_cells)))

        return clusters

    # ─────────────────────────────────────────────────────────────────────────
    # TF pose lookup
    # ─────────────────────────────────────────────────────────────────────────

    def _get_robot_pose(self, robot: RobotState) -> Optional[Tuple[float, float]]:
        """
        Look up the robot's position in the global 'map' frame.

        TF chain (simulation):
          map  ──static_transform──►  {robot_name}_map
                                            │
                                      SLAM Toolbox
                                            │
                                            ▼
                                  {robot_name}_odom
                                            │
                                     omni_driver
                                            │
                                            ▼
                                  {robot_name}_robot_footprint

        The base_frame '{robot_name}_robot_footprint' is confirmed from:
          spawn_ausra_full.launch.py → get_omni_driver_params() base_frame_id
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                'map',
                robot.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            return (x, y)
        except TransformException as e:
            self.get_logger().warn(
                f'[{robot.name}] TF lookup map→{robot.base_frame} failed: {e}'
            )
            return None

    # ─────────────────────────────────────────────────────────────────────────
    # Goal sending
    # ─────────────────────────────────────────────────────────────────────────

    def _send_goal(self, robot: RobotState, frontier: Frontier,
                   available_frontiers: Optional[list] = None):
        """Send a NavigateToPose goal to /{robot}/navigate_to_pose.

        available_frontiers: pass the current cycle's list so the frontier is
        removed only after the goal is confirmed dispatched. This prevents a
        down Nav2 from consuming frontier slots that other robots could use.
        """
        client = self._action_clients[robot.name]

        # server_is_ready() is checked non-blocking in _plan() for normal goals.
        # This fallback covers bootstrap goals which skip the pre-flight check.
        if not client.server_is_ready():
            if not client.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn(
                    f'[{robot.name}] navigate_to_pose action server not available — skipping'
                )
                return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # ── Safety offset (mirrors exploration_server_enhanced.cpp L526-543) ──
        pose = self._get_robot_pose(robot)
        if pose is not None and not (frontier.size == 0):  # skip bootstrap goals
            rx_now, ry_now = pose
            dx = frontier.x - rx_now
            dy = frontier.y - ry_now
            dist = math.hypot(dx, dy)
            safety_offset = 0.3
            if dist > safety_offset + 0.2:
                ratio = (dist - safety_offset) / dist
                tx = rx_now + dx * ratio
                ty = ry_now + dy * ratio
            else:
                tx, ty = frontier.x, frontier.y
        else:
            tx, ty = frontier.x, frontier.y

        goal_msg.pose.pose.position.x = tx
        goal_msg.pose.pose.position.y = ty
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        robot.goal_seq += 1
        my_seq = robot.goal_seq

        robot.is_idle = False
        robot.trivial_count = 0
        robot.current_goal = frontier
        robot.goal_start_time = time.monotonic()

        # Consume frontier slot NOW — goal is going to Nav2
        if available_frontiers is not None and frontier in available_frontiers:
            available_frontiers.remove(frontier)

        future = client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda f, seq=my_seq: self._on_goal_accepted(f, robot, seq)
        )


    def _on_goal_accepted(self, future, robot: RobotState, seq: int):
        # Discard if a newer goal was already issued (race condition guard)
        if seq != robot.goal_seq:
            self.get_logger().debug(
                f'[{robot.name}] Stale goal_accepted (seq={seq} vs current={robot.goal_seq}) — ignoring'
            )
            return

        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'[{robot.name}] Goal REJECTED by Nav2 — blacklisting frontier')
            if seq == robot.goal_seq:
                # Blacklist immediately so the same frontier is never re-assigned.
                # Previously only timeouts blacklisted; rejections looped forever.
                if robot.current_goal:
                    self._blacklist.append((robot.current_goal.x, robot.current_goal.y))
                robot.is_idle = True
                robot.current_goal = None
            return

        robot.goal_handle = goal_handle
        goal_label = (
            f'({robot.current_goal.x:.2f}, {robot.current_goal.y:.2f})'
            if robot.current_goal else '(cleared)'
        )
        self.get_logger().info(f'[{robot.name}] Goal ACCEPTED → {goal_label}')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, seq=seq: self._on_goal_result(f, robot, seq)
        )

    def _on_goal_result(self, future, robot: RobotState, seq: int):
        # Discard if a newer goal was already issued
        if seq != robot.goal_seq:
            self.get_logger().debug(
                f'[{robot.name}] Stale goal_result (seq={seq} vs current={robot.goal_seq}) — ignoring'
            )
            return

        result = future.result()
        status = result.status
        elapsed = time.monotonic() - robot.goal_start_time
        is_bootstrap = (robot.current_goal is not None and robot.current_goal.size == 0)

        if status == GoalStatus.STATUS_SUCCEEDED:
            if not is_bootstrap and elapsed < self._trivial_goal_s:
                # Nav2 "succeeded" instantly — robot was already at/near the goal.
                # Blacklist this frontier so it is never assigned again.
                robot.trivial_count += 1
                self.get_logger().warn(
                    f'[{robot.name}] Trivial goal — succeeded in {elapsed:.1f}s '
                    f'(< {self._trivial_goal_s:.0f}s threshold) → blacklisting '
                    f'({robot.current_goal.x:.2f}, {robot.current_goal.y:.2f}) '
                    f'trivial_count={robot.trivial_count}'
                )
                if robot.current_goal:
                    self._blacklist.append((robot.current_goal.x, robot.current_goal.y))
            elif is_bootstrap:
                self.get_logger().info(
                    f'[{robot.name}] Bootstrap move completed in {elapsed:.1f}s — '
                    f'SLAM map should now be larger'
                )
                robot.trivial_count = 0
            else:
                self.get_logger().info(
                    f'[{robot.name}] Goal SUCCEEDED in {elapsed:.1f}s'
                )
                robot.trivial_count = 0
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(f'[{robot.name}] Goal ABORTED — blacklisting frontier')
            if robot.current_goal:
                self._blacklist.append((robot.current_goal.x, robot.current_goal.y))
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'[{robot.name}] Goal CANCELLED after {elapsed:.1f}s')
        else:
            self.get_logger().warn(f'[{robot.name}] Goal ended with status {status}')

        robot.is_idle = True
        robot.current_goal = None
        robot.goal_handle = None

    # ─────────────────────────────────────────────────────────────────────────
    # RViz2 visualization
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_markers(self, frontiers: List[Frontier], assigned: set):
        ma = MarkerArray()

        # Clear old markers
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        for i, f in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'frontiers'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = f.x
            m.pose.position.y = f.y
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            scale = max(0.2, min(0.6, f.size * 0.01))
            m.scale.x = scale
            m.scale.y = scale
            m.scale.z = scale

            is_assigned = (round(f.x, 3), round(f.y, 3)) in {
                (round(ax, 3), round(ay, 3)) for ax, ay in assigned
            }
            if is_assigned:
                # Grey = assigned to a robot
                m.color.r = 0.5
                m.color.g = 0.5
                m.color.b = 0.5
                m.color.a = 0.8
            else:
                # Blue = available
                m.color.r = 0.0
                m.color.g = 0.5
                m.color.b = 1.0
                m.color.a = 0.9

            m.lifetime.sec = int(1.0 / self._planning_rate_hz * 3)
            ma.markers.append(m)

        self._marker_pub.publish(ma)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = FrontierCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
