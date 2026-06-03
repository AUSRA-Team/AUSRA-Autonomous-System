# ============================================================
# FILE: relay_node.py
# RUNS ON: Robot (Jetson Orin Nano) — e.g. ausra_1, ausra_2
# PURPOSE: Subscribes to /map (OccupancyGrid) and /pose
#          (PoseWithCovarianceStamped) from slam_toolbox,
#          republishes as /<robot_name>/map (throttled) and
#          /<robot_name>/pose (as PoseStamped). Also publishes
#          /<robot_name>/heartbeat (String) at 1 Hz.
#
# QoS NOTES:
#   slam_toolbox publishes /map with transient_local + reliable.
#   map_expansion_node subscribes with transient_local + reliable.
#   This relay must match BOTH sides, otherwise messages are
#   silently dropped by DDS with no error or warning.
#
# PARAMETERS:
#   robot_name       — string robot name (e.g. 'ausra_1')
#   map_interval_sec — seconds between map republishes
#
# PLACEHOLDERS: None — robot_name is set via launch argument.
# ============================================================

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String


class RelayNode(Node):
    def __init__(self):
        super().__init__('relay_node')

        # Declare parameters
        self.declare_parameter('robot_name', 'ausra_1')
        self.declare_parameter('map_interval_sec', 5.0)

        self.robot_name = self.get_parameter('robot_name').value
        self.map_interval = self.get_parameter('map_interval_sec').value
        self.last_map_sent = 0.0

        prefix = f'/{self.robot_name}'

        # QoS matching slam_toolbox and map_expansion_node:
        # transient_local ensures late-joining subscribers get the last map
        # reliable ensures no silent message drops
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # ---- /map → /<robot_name>/map (throttled by map_interval_sec) ----
        self.map_pub = self.create_publisher(OccupancyGrid, f'{prefix}/map', map_qos)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_cb, map_qos)

        # ---- /pose → /<robot_name>/pose (PoseWithCovarianceStamped → PoseStamped) ----
        self.pose_pub = self.create_publisher(PoseStamped, f'{prefix}/pose', 10)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/pose', self.pose_cb, 10)

        # ---- /<robot_name>/heartbeat at 1 Hz ----
        self.hb_pub = self.create_publisher(String, f'{prefix}/heartbeat', 10)
        self.create_timer(1.0, self.heartbeat_cb)

        self.get_logger().info(
            f'Relay active → {self.robot_name} | '
            f'map throttle every {self.map_interval}s')

    def map_cb(self, msg):
        """Republish /map → /<robot_name>/map, throttled to one message per map_interval_sec."""
        now = time.time()
        if now - self.last_map_sent >= self.map_interval:
            self.map_pub.publish(msg)
            self.last_map_sent = now
            self.get_logger().info(
                f'Map relayed ({msg.info.width}x{msg.info.height}, '
                f'{len(msg.data)} cells)')

    def pose_cb(self, msg):
        """Convert PoseWithCovarianceStamped → PoseStamped and republish."""
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.pose_pub.publish(ps)

    def heartbeat_cb(self):
        """Publish heartbeat string at 1 Hz."""
        hb = String()
        hb.data = f'{self.robot_name} alive'
        self.hb_pub.publish(hb)


def main(args=None):
    rclpy.init(args=args)
    node = RelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Relay node shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
