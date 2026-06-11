# ============================================================
# FILE: fake_robot_pub.py
# RUNS ON: Laptop (for testing without real hardware)
# PURPOSE: Publishes fake /<robot_name>/pose, /<robot_name>/heartbeat,
#          and /<robot_name>/map at the correct rates so the full
#          comms pipeline can be tested without a real Jetson.
#
#          Pose: PoseStamped at 10 Hz with slowly drifting x/y
#          Heartbeat: String at 1 Hz
#          Map: small 50x50 OccupancyGrid at configurable interval
#
# PARAMETERS:
#   robot_name       — string robot name (e.g. 'ausra_1', 'ausra_2')
#   robot_index      — integer index for deterministic map generation (1, 2, 3)
#   map_interval_sec — seconds between fake map publishes
#
# PLACEHOLDERS: None
# ============================================================

import math
import random
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class FakeRobotPub(Node):
    def __init__(self):
        super().__init__('fake_robot_pub')

        self.declare_parameter('robot_name', 'ausra_1')
        self.declare_parameter('robot_index', 1)
        self.declare_parameter('map_interval_sec', 30.0)

        self.robot_name = self.get_parameter('robot_name').value
        self.robot_index = self.get_parameter('robot_index').value
        self.map_interval = self.get_parameter('map_interval_sec').value

        prefix = f'/{self.robot_name}'
        self.prefix = prefix
        self.start_time = time.time()
        self.last_map_sent = 0.0

        # Use transient_local + reliable QoS for the map publisher
        # to match what map_merge expects (latched topic behavior)
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # --- Publishers ---
        self.pose_pub = self.create_publisher(PoseStamped, f'{prefix}/pose', 10)
        self.hb_pub = self.create_publisher(String, f'{prefix}/heartbeat', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, f'{prefix}/map', map_qos)

        # --- Timers ---
        self.create_timer(0.1, self.pose_cb)       # 10 Hz
        self.create_timer(1.0, self.heartbeat_cb)   # 1 Hz
        self.create_timer(5.0, self.map_cb)         # check every 5s, throttle internally

        # Build the fake map once (reuse it on each publish)
        self._fake_map = self._build_fake_map()

        self.get_logger().info(
            f'Fake publisher active → {self.robot_name} (index {self.robot_index}) | '
            f'pose@10Hz, heartbeat@1Hz, map every {self.map_interval}s')

    def _build_fake_map(self):
        """Build a 50x50 occupancy grid with walls and internal structure.

        Each robot gets a different map origin offset so map_merge has
        distinct but overlapping grids to merge.
        """
        width = 50
        height = 50
        resolution = 0.05  # 2.5m x 2.5m total area

        # Empty map (-1 = unknown space) for testing map merge
        data = [-1] * (width * height)

        grid = OccupancyGrid()
        # frame_id = 'map' — the global frame. map_merge uses init_pose
        # parameters (not TF) to transform grids, so all maps should be
        # published in the common frame.
        grid.header.frame_id = 'map'
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = data
        return grid

    def pose_cb(self):
        """Publish a fake pose that slowly moves in a circle."""
        elapsed = time.time() - self.start_time
        # Each robot gets a different phase offset so poses don't overlap
        phase = (self.robot_index - 1) * (2.0 * math.pi / 3.0)
        radius = 2.0

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'
        ps.pose.position.x = radius * math.cos(0.1 * elapsed + phase)
        ps.pose.position.y = radius * math.sin(0.1 * elapsed + phase)
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0
        self.pose_pub.publish(ps)

    def heartbeat_cb(self):
        """Publish heartbeat at 1 Hz."""
        msg = String()
        msg.data = f'{self.robot_name} alive'
        self.hb_pub.publish(msg)

    def map_cb(self):
        """Publish the fake occupancy grid, throttled by map_interval_sec."""
        now = time.time()
        if now - self.last_map_sent < self.map_interval:
            return

        self.last_map_sent = now

        # Update the timestamp and publish
        self._fake_map.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self._fake_map)
        self.get_logger().info(
            f'Fake map sent ({self._fake_map.info.width}x'
            f'{self._fake_map.info.height}, '
            f'{len(self._fake_map.data)} cells)')


def main(args=None):
    rclpy.init(args=args)
    node = FakeRobotPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Fake publisher shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
