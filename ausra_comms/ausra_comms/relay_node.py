import hashlib
import json
import subprocess
import threading
import time
import zlib

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import ByteMultiArray, MultiArrayDimension, MultiArrayLayout, String


class RelayNode(Node):
    def __init__(self):
        super().__init__('relay_node')

        self.declare_parameter('robot_name', 'ausra_1')
        self.declare_parameter('map_interval_sec', 5.0)
        self.declare_parameter('base_station_ip', '192.168.0.105')
        self.declare_parameter('enable_compression', False)
        self.declare_parameter('enable_adaptive_throttle', True)
        self.declare_parameter('enable_delta_detection', True)
        self.declare_parameter('delta_threshold', 0.01)

        self.robot_name = self.get_parameter('robot_name').value
        self.map_interval = self.get_parameter('map_interval_sec').value
        self.map_interval_default = self.map_interval
        self.base_station_ip = self.get_parameter('base_station_ip').value
        self.enable_compression = self.get_parameter('enable_compression').value
        self.enable_adaptive = self.get_parameter('enable_adaptive_throttle').value
        self.enable_delta = self.get_parameter('enable_delta_detection').value
        self.delta_threshold = self.get_parameter('delta_threshold').value

        self.last_map_sent = 0.0
        self.map_count = 0
        self.map_source = 'none'

        self.last_raw_size = 0
        self.last_compressed_size = 0
        self.total_bytes_saved = 0

        self.wifi_latency_ms = -1.0

        self.last_map_hash = None
        self.delta_skips = 0

        prefix = f'/{self.robot_name}'

        # Use TRANSIENT_LOCAL so relay receives the last map even if SLAM published before relay started
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid, f'{prefix}/map', map_qos)

        if self.enable_compression:
            self.map_compressed_pub = self.create_publisher(
                ByteMultiArray, f'{prefix}/map_compressed',
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        else:
            self.map_compressed_pub = None

        # Subscribe to both namespaced and global map topics
        self.map_sub_ns = self.create_subscription(
            OccupancyGrid, f'{prefix}/map', self.map_cb_ns, map_qos)
        self.map_sub_global = self.create_subscription(
            OccupancyGrid, '/map', self.map_cb_global, map_qos)

        self.hb_pub = self.create_publisher(String, f'{prefix}/heartbeat', 10)
        self.create_timer(1.0, self.heartbeat_cb)

        if self.enable_adaptive:
            self._latency_thread = threading.Thread(
                target=self._latency_monitor_loop, daemon=True)
            self._latency_thread.start()

        self.get_logger().info(
            f'Relay active → {self.robot_name} | map throttle {self.map_interval}s | '
            f'compression={self.enable_compression}')

    def map_cb_ns(self, msg):
        self.map_source = f'/{self.robot_name}/map'
        self._handle_map(msg)

    def map_cb_global(self, msg):
        self.map_source = '/map'
        self._handle_map(msg)

    def _handle_map(self, msg):
        """Throttle, delta-detect, and relay map."""
        now = time.time()
        if now - self.last_map_sent < self.map_interval:
            return

        if self.enable_delta and self._is_map_unchanged(msg):
            self.delta_skips += 1
            if self.delta_skips % 10 == 1:
                self.get_logger().info(f'Delta skip #{self.delta_skips}')
            return

        self.map_pub.publish(msg)

        if self.map_compressed_pub is not None:
            self._publish_compressed(msg)

        self.last_map_sent = now
        self.map_count += 1

        self.get_logger().info(
            f'Map relayed #{self.map_count} from {self.map_source} '
            f'({msg.info.width}x{msg.info.height})')

    def _is_map_unchanged(self, msg):
        """Check if map data changed since last publish."""
        data_bytes = bytes(msg.data)
        current_hash = hashlib.md5(data_bytes).digest()

        if self.last_map_hash is None:
            self.last_map_hash = current_hash
            return False

        if current_hash == self.last_map_hash:
            return True

        self.last_map_hash = current_hash
        return False

    def _publish_compressed(self, msg):
        """Compress map data and publish as ByteMultiArray."""
        metadata = {
            'ox': round(msg.info.origin.position.x, 6),
            'oy': round(msg.info.origin.position.y, 6),
            'oz': round(msg.info.origin.position.z, 6),
            'res': round(float(msg.info.resolution), 6),
            'w': int(msg.info.width),
            'h': int(msg.info.height),
            'frame': msg.header.frame_id,
            'sec': msg.header.stamp.sec,
            'nsec': msg.header.stamp.nanosec,
            'qx': round(msg.info.origin.orientation.x, 6),
            'qy': round(msg.info.origin.orientation.y, 6),
            'qz': round(msg.info.origin.orientation.z, 6),
            'qw': round(msg.info.origin.orientation.w, 6),
        }
        meta_json = json.dumps(metadata, separators=(',', ':'))

        raw_data = bytes(msg.data)
        compressed = zlib.compress(raw_data, level=6)

        self.last_raw_size = len(raw_data)
        self.last_compressed_size = len(compressed)
        self.total_bytes_saved += (self.last_raw_size - self.last_compressed_size)

        out_msg = ByteMultiArray()
        out_msg.layout = MultiArrayLayout()
        out_msg.layout.dim = [
            MultiArrayDimension(label=meta_json, size=len(compressed), stride=0)
        ]
        out_msg.data = list(compressed)

        self.map_compressed_pub.publish(out_msg)

    def heartbeat_cb(self):
        """Publish simple heartbeat."""
        hb = String()
        hb.data = f'{self.robot_name} alive | maps={self.map_count}'
        self.hb_pub.publish(hb)

    def _latency_monitor_loop(self):
        """Background thread to monitor WiFi latency and adapt map rate."""
        while True:
            time.sleep(30.0)
            try:
                latency = self._measure_latency()
                self.wifi_latency_ms = latency

                old_interval = self.map_interval
                if latency < 0:
                    self.map_interval = max(30.0, self.map_interval_default)
                elif latency > 300:
                    self.map_interval = 30.0
                elif latency > 150:
                    self.map_interval = 15.0
                elif latency > 50:
                    self.map_interval = max(self.map_interval_default, 10.0)
                else:
                    self.map_interval = self.map_interval_default

                if abs(old_interval - self.map_interval) > 0.1:
                    self.get_logger().info(
                        f'Adaptive throttle: latency={latency:.0f}ms → '
                        f'map_interval {old_interval:.1f}s → {self.map_interval:.1f}s')
            except Exception as e:
                self.get_logger().warning(
                    f'Latency monitor error: {e}',
                    throttle_duration_sec=60.0)

    def _measure_latency(self):
        """Ping base station, return latency in ms or -1 on failure."""
        try:
            result = subprocess.run(
                ['ping', '-c', '1', '-W', '2', self.base_station_ip],
                capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if 'time=' in line:
                        time_str = line.split('time=')[1].split(' ')[0]
                        return float(time_str)
            return -1.0
        except (subprocess.TimeoutExpired, Exception):
            return -1.0


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
