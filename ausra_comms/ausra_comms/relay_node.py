# relay_node.py - Throttles map, extracts pose from TF, publishes heartbeat.
#
# Runs on Jetson. Subscribes to SLAM map topic (both namespaced and
# global), extracts robot pose from TF using the map's own frame_id,
# and republishes on *_relay topics for Zenoh bridging.
#
# Data flow:
#   /{robot}/map or /map  →  /{robot}/map_relay   (throttled)
#   TF lookup             →  /{robot}/pose_relay   (5 Hz)
#   heartbeat             →  /{robot}/heartbeat    (1 Hz)

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import tf2_ros


class RelayNode(Node):
    def __init__(self):
        super().__init__('relay_node')

        self.declare_parameter('robot_name', 'ausra_1')
        self.declare_parameter('map_interval_sec', 5.0)

        self.robot_name = self.get_parameter('robot_name').value
        self.map_interval = self.get_parameter('map_interval_sec').value
        self.last_map_sent = 0.0
        self.map_count = 0
        self.pose_count = 0
        self.map_source = 'none'
        self.pose_status = 'waiting for map frame_id'

        # Discovered from the first map message's header.frame_id
        self.map_frame = None

        prefix = f'/{self.robot_name}'

        # --- Map QoS ---
        map_pub_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        map_sub_qos = QoSProfile(
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # Map: subscribe to both namespaced and global, publish throttled
        self.map_pub = self.create_publisher(OccupancyGrid, f'{prefix}/map_relay', map_pub_qos)
        self.map_sub_ns = self.create_subscription(
            OccupancyGrid, f'{prefix}/map', self.map_cb_ns, map_sub_qos)
        self.map_sub_global = self.create_subscription(
            OccupancyGrid, '/map', self.map_cb_global, map_sub_qos)

        # Pose: extracted from TF, published as PoseStamped
        self.pose_pub = self.create_publisher(PoseStamped, f'{prefix}/pose_relay', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(0.2, self.publish_pose_from_tf)  # 5 Hz

        # Heartbeat at 1 Hz
        self.hb_pub = self.create_publisher(String, f'{prefix}/heartbeat', 10)
        self.create_timer(1.0, self.heartbeat_cb)

        self.get_logger().info(
            f'Relay active → {self.robot_name} | '
            f'map throttle {self.map_interval}s | '
            f'pose from TF (auto-detect map frame from first map msg)')

    # --- Map callbacks ---
    def map_cb_ns(self, msg):
        self.map_source = f'/{self.robot_name}/map'
        self._handle_map(msg)

    def map_cb_global(self, msg):
        self.map_source = '/map'
        self._handle_map(msg)

    def _handle_map(self, msg):
        """Throttle, republish, and learn the map frame_id."""
        # Learn the map frame from the first received map message
        if self.map_frame is None and msg.header.frame_id:
            self.map_frame = msg.header.frame_id
            self.get_logger().info(
                f'Discovered map frame_id: "{self.map_frame}" — '
                f'TF lookups will use this as parent frame')

        now = time.time()
        if now - self.last_map_sent >= self.map_interval:
            self.map_pub.publish(msg)
            self.last_map_sent = now
            self.map_count += 1
            self.get_logger().info(
                f'Map relayed #{self.map_count} from {self.map_source} '
                f'({msg.info.width}x{msg.info.height}, {len(msg.data)} cells)')

    # --- Pose from TF ---
    def publish_pose_from_tf(self):
        """Look up TF from map_frame → base_frame and publish as PoseStamped."""
        if self.map_frame is None:
            return  # Wait until we learn the map frame from a map message

        # Base frame candidates (try namespaced variations)
        base_candidates = [
            'ausrabot_robot_footprint',
            f'{self.robot_name}_ausrabot_robot_footprint',
            f'{self.robot_name}/ausrabot_robot_footprint',
        ]

        trans = None
        found_base = None
        last_err = None
        for base in base_candidates:
            try:
                trans = self.tf_buffer.lookup_transform(
                    self.map_frame, base, rclpy.time.Time())
                found_base = base
                break
            except Exception as e:
                last_err = e
                continue

        if trans is not None:
            ps = PoseStamped()
            ps.header.stamp = trans.header.stamp
            ps.header.frame_id = self.map_frame
            ps.pose.position.x = trans.transform.translation.x
            ps.pose.position.y = trans.transform.translation.y
            ps.pose.position.z = trans.transform.translation.z
            ps.pose.orientation = trans.transform.rotation
            self.pose_pub.publish(ps)
            self.pose_count += 1
            self.pose_status = f'OK ({self.map_frame} → {found_base})'
        else:
            self.pose_status = f'TF fail: {last_err}'
            self.get_logger().warning(
                f'TF lookup failed: {self.map_frame} → {base_candidates}. '
                f'Error: {last_err}',
                throttle_duration_sec=5.0)

    # --- Heartbeat ---
    def heartbeat_cb(self):
        hb = String()
        hb.data = (f'{self.robot_name} alive | '
                   f'maps={self.map_count} (src:{self.map_source}) | '
                   f'poses={self.pose_count} ({self.pose_status})')
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
