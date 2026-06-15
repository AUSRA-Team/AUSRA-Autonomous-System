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

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.pose_pub = self.create_publisher(PoseStamped, f'{prefix}/pose', 10)
        self.hb_pub = self.create_publisher(String, f'{prefix}/heartbeat', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, f'{prefix}/map', map_qos)

        self.create_timer(0.1, self.pose_cb)
        self.create_timer(1.0, self.heartbeat_cb)
        self.create_timer(5.0, self.map_cb)

        self._fake_map = self._build_fake_map()

        self.get_logger().info(
            f'Fake publisher active → {self.robot_name} (index {self.robot_index}) | '
            f'pose@10Hz, heartbeat@1Hz, map every {self.map_interval}s')

    def _build_fake_map(self):
        """Build a fake occupancy grid."""
        width = 50
        height = 50
        resolution = 0.05

        data = [-1] * (width * height)

        grid = OccupancyGrid()
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
        """Publish fake circular movement pose."""
        elapsed = time.time() - self.start_time
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
        """Publish fake heartbeat."""
        msg = String()
        msg.data = f'{self.robot_name} alive'
        self.hb_pub.publish(msg)

    def map_cb(self):
        """Publish fake map throttled by map_interval_sec."""
        now = time.time()
        if now - self.last_map_sent < self.map_interval:
            return

        self.last_map_sent = now

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
