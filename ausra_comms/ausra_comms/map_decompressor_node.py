import json
import zlib

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import UInt8MultiArray
from geometry_msgs.msg import Pose, Point, Quaternion
from builtin_interfaces.msg import Time


class MapDecompressorNode(Node):
    def __init__(self):
        super().__init__('map_decompressor_node')

        self.declare_parameter('robots', ['ausra_1', 'ausra_2'])
        self.declare_parameter('ignore_robot', '')

        robots = self.get_parameter('robots').value
        ignore_robot = self.get_parameter('ignore_robot').value

        self.decompress_count = {r: 0 for r in robots}

        # ReentrantCallbackGroup + MultiThreadedExecutor so multiple robots'
        # maps decompress concurrently instead of serializing behind one thread.
        self.cb_group = ReentrantCallbackGroup()

        # Match the relay's TRANSIENT_LOCAL + RELIABLE so a map published before
        # this node starts (or during a WiFi blip) is still delivered. depth=2
        # absorbs jitter without dropping.
        output_qos = QoSProfile(
            depth=2,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        input_qos = QoSProfile(
            depth=2,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        active_robots = []
        for robot in robots:
            if robot == ignore_robot:
                self.get_logger().info(f'Ignoring local robot: {robot}')
                continue

            active_robots.append(robot)

            pub = self.create_publisher(
                OccupancyGrid, f'/{robot}/map', output_qos)

            self.create_subscription(
                UInt8MultiArray,
                f'/{robot}/map_compressed',
                lambda msg, r=robot, p=pub: self._decompress_cb(msg, r, p),
                input_qos,
                callback_group=self.cb_group)

            self.get_logger().info(
                f'Decompressor: /{robot}/map_compressed → /{robot}/map')

        self.get_logger().info(
            f'Map decompressor active for {len(active_robots)} robots: {active_robots}')

    def _decompress_cb(self, msg, robot_name, publisher):
        """Decompress map data and publish."""
        try:
            if not msg.layout.dim:
                self.get_logger().error(f'[{robot_name}] No metadata in compressed message layout')
                return

            meta_json = msg.layout.dim[0].label
            metadata = json.loads(meta_json)

            compressed_data = bytes(msg.data)
            raw_data = zlib.decompress(compressed_data)

            expected_size = metadata['w'] * metadata['h']
            if len(raw_data) != expected_size:
                self.get_logger().error(
                    f'[{robot_name}] Decompressed size mismatch: '
                    f'got {len(raw_data)}, expected {expected_size}')
                return

            grid = OccupancyGrid()

            grid.header.frame_id = metadata['frame']
            grid.header.stamp = Time(
                sec=metadata['sec'],
                nanosec=metadata['nsec'])

            grid.info = MapMetaData()
            grid.info.resolution = float(metadata['res'])
            grid.info.width = int(metadata['w'])
            grid.info.height = int(metadata['h'])
            grid.info.origin = Pose(
                position=Point(
                    x=float(metadata['ox']),
                    y=float(metadata['oy']),
                    z=float(metadata['oz'])),
                orientation=Quaternion(
                    x=float(metadata['qx']),
                    y=float(metadata['qy']),
                    z=float(metadata['qz']),
                    w=float(metadata['qw'])))

            # raw_data is unsigned (0..255); OccupancyGrid.data is int8, so map
            # values >127 (e.g. the 255 that encodes unknown=-1) back to signed.
            grid.data = [b - 256 if b > 127 else b for b in raw_data]

            publisher.publish(grid)
            self.decompress_count[robot_name] += 1

            ratio = (1.0 - len(compressed_data) / len(raw_data)) * 100
            self.get_logger().info(
                f'[{robot_name}] Decompressed map #{self.decompress_count[robot_name]}: '
                f'{len(compressed_data)}B → {len(raw_data)}B '
                f'({ratio:.1f}% saved)',
                throttle_duration_sec=10.0)

        except json.JSONDecodeError as e:
            self.get_logger().error(f'[{robot_name}] Invalid metadata JSON: {e}')
        except zlib.error as e:
            self.get_logger().error(f'[{robot_name}] zlib decompression failed: {e}')
        except Exception as e:
            self.get_logger().error(f'[{robot_name}] Decompression error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MapDecompressorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Map decompressor shutting down')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
