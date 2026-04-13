import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, Pose
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import numpy as np
import struct
import time
import math

class FleetCommNode(Node):
    def __init__(self):
        super().__init__('fleet_comm_node')
        
        self.declare_parameter('namespaces', ['ausra_1', 'ausra_2', 'ausra_3'])
        self.declare_parameter('comm_range', 20.0)
        self.declare_parameter('update_rate', 2.0)
        self.declare_parameter('robot_radius', 0.2)
        
        self.namespaces = self.get_parameter('namespaces').get_parameter_value().string_array_value
        if not self.namespaces:
            # Fallback if unconfigured correctly
            self.namespaces = ['ausra_1', 'ausra_2', 'ausra_3']
            
        self.comm_range = self.get_parameter('comm_range').get_parameter_value().double_value
        update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.robot_maps = {}
        self.map_subs = []
        self.pc_pubs = {}
        self.native_map_pubs = {}
        
        # Maps require Transient Local durability to work with Nav2 Static Layer
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Subscribe and publish for each robot
        for ns in self.namespaces:
            # Subscriber for local map
            sub = self.create_subscription(
                OccupancyGrid,
                f'/{ns}/slam_map_internal',
                lambda msg, ns=ns: self.map_callback(msg, ns),
                map_qos
            )
            self.map_subs.append(sub)
            
            # Publisher for dynamic avoidance pointcloud
            self.pc_pubs[ns] = self.create_publisher(
                PointCloud2,
                f'/{ns}/neighbor_obstacles',
                10
            )
            
            # Publisher to override native map with the stitched map
            self.native_map_pubs[ns] = self.create_publisher(
                OccupancyGrid,
                f'/{ns}/map',
                map_qos
            )
            
        # Global map for Foxglove Fleet Commander
        self.global_map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        
        self.timer = self.create_timer(1.0 / update_rate, self.update_network)
        self.get_logger().info(f"Fleet Comm Node started for: {self.namespaces}")

    def map_callback(self, msg, ns):
        self.robot_maps[ns] = msg

    def get_robot_positions(self):
        positions = {}
        for ns in self.namespaces:
            try:
                # We expect SLAM or odometry to provide map -> ausra_X_robot_footprint
                t = self.tf_buffer.lookup_transform(
                    'map',
                    f'{ns}_robot_footprint',
                    rclpy.time.Time())
                positions[ns] = (t.transform.translation.x, t.transform.translation.y)
            except TransformException as ex:
                # Node just started or robot not spawned yet
                pass
        return positions

    def compute_connected_components(self, positions):
        # Build adjacency
        adj = {ns: set() for ns in positions}
        for ns1, pos1 in positions.items():
            for ns2, pos2 in positions.items():
                if ns1 != ns2:
                    dist = math.hypot(pos1[0] - pos2[0], pos1[1] - pos2[1])
                    if dist <= self.comm_range:
                        adj[ns1].add(ns2)
                        adj[ns2].add(ns1)
                        
        # Find components via BFS
        visited = set()
        components = []
        for ns in positions:
            if ns not in visited:
                comp = set()
                queue = [ns]
                while queue:
                    curr = queue.pop(0)
                    if curr not in visited:
                        visited.add(curr)
                        comp.add(curr)
                        queue.extend(list(adj[curr] - visited))
                components.append(comp)
        return components

    def create_obstacle_pointcloud(self, positions, exclude_ns):
        # Generate a PointCloud2 circle around each position
        points = []
        for ns, pos in positions.items():
            if ns == exclude_ns:
                continue
            # create circle points
            num_points_in_circle = 12
            for i in range(num_points_in_circle):
                angle = (i / num_points_in_circle) * 2 * math.pi
                px = pos[0] + self.robot_radius * math.cos(angle)
                py = pos[1] + self.robot_radius * math.sin(angle)
                pz = 0.2 # 20cm above ground
                points.append((px, py, pz))
                
        # construct PointCloud2
        msg = PointCloud2()
        msg.header = Header()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        
        buffer = []
        for p in points:
            buffer.append(struct.pack('fff', p[0], p[1], p[2]))
        msg.data = b''.join(buffer)
        
        return msg

    def merge_maps(self, map_msgs):
        if not map_msgs:
            return None
            
        # We assume all maps use the same resolution (from our slam_toolbox config)
        res = map_msgs[0].info.resolution
        
        # 1. Find global bounds for the new merged grid
        min_x_list = []
        min_y_list = []
        max_x_list = []
        max_y_list = []
        
        for m in map_msgs:
            try:
                t = self.tf_buffer.lookup_transform('map', m.header.frame_id, rclpy.time.Time())
                offset_x = t.transform.translation.x
                offset_y = t.transform.translation.y
            except TransformException as ex:
                # Fallback
                offset_x = 0.0
                offset_y = 0.0
                
            px = m.info.origin.position.x + offset_x
            py = m.info.origin.position.y + offset_y
            w = m.info.width
            h = m.info.height
            min_x_list.append(px)
            min_y_list.append(py)
            max_x_list.append(px + w * res)
            max_y_list.append(py + h * res)
            
        global_min_x = min(min_x_list)
        global_min_y = min(min_y_list)
        global_max_x = max(max_x_list)
        global_max_y = max(max_y_list)
        
        global_width = int(math.ceil((global_max_x - global_min_x) / res))
        global_height = int(math.ceil((global_max_y - global_min_y) / res))
        
        # Create empty grid initialized to -1 (unknown)
        merged_array = np.full((global_height, global_width), -1, dtype=np.int8)
        
        # 2. Merge each map
        for m in map_msgs:
            try:
                t = self.tf_buffer.lookup_transform('map', m.header.frame_id, rclpy.time.Time())
                offset_x = t.transform.translation.x
                offset_y = t.transform.translation.y
            except TransformException as ex:
                offset_x = 0.0
                offset_y = 0.0
                
            # converting data tuple to numpy array
            arr = np.array(m.data, dtype=np.int8).reshape((m.info.height, m.info.width))
            
            # compute indices in global array using offset
            start_col = int(round((m.info.origin.position.x + offset_x - global_min_x) / res))
            start_row = int(round((m.info.origin.position.y + offset_y - global_min_y) / res))
            
            # end indices
            end_col = start_col + m.info.width
            end_row = start_row + m.info.height
            
            # extract ROI
            roi = merged_array[start_row:end_row, start_col:end_col]
            
            # maximum handles (-1 vs 0 vs 100) correctly!
            # 100 (obstacle) > 0 (free) > -1 (unknown)
            merged_array[start_row:end_row, start_col:end_col] = np.maximum(roi, arr)
            
        merged_msg = OccupancyGrid()
        merged_msg.header.stamp = self.get_clock().now().to_msg()
        merged_msg.header.frame_id = 'map'
        merged_msg.info.resolution = res
        merged_msg.info.width = global_width
        merged_msg.info.height = global_height
        merged_msg.info.origin.position.x = global_min_x
        merged_msg.info.origin.position.y = global_min_y
        merged_msg.info.origin.position.z = 0.0
        # Flatten and convert to list
        merged_msg.data = merged_array.flatten().tolist()
        
        # Log for debugging
        self.get_logger().info(f"Merged {len(map_msgs)} maps into {global_width}x{global_height} grid.")
        
        return merged_msg

    def update_network(self):
        positions = self.get_robot_positions()
        if not positions:
            return
            
        components = self.compute_connected_components(positions)
        
        # For each component, share Poses and Maps
        for comp in components:
            # Poses (Dynamic Avoidance)
            comp_positions = {ns: positions[ns] for ns in comp}
            
            # Gather maps
            comp_maps = [self.robot_maps[ns] for ns in comp if ns in self.robot_maps]
            merged_comp_map = self.merge_maps(comp_maps)
            
            # Publish Data
            for ns in comp:
                # Pointcloud
                pc_msg = self.create_obstacle_pointcloud(comp_positions, exclude_ns=ns)
                self.pc_pubs[ns].publish(pc_msg)
                    
        # Global Map for Foxglove
        all_maps = list(self.robot_maps.values())
        if all_maps:
            self.get_logger().info(f"Generating global map from {len(all_maps)} robots...")
            global_merged = self.merge_maps(all_maps)
            if global_merged:
                self.global_map_pub.publish(global_merged)
                self.get_logger().info("Global map published to /map!")
                
                # Now, reverse the projection and publish to each robot's native /map topic!
                for ns in self.namespaces:
                    local_map = OccupancyGrid()
                    local_map.header.stamp = self.get_clock().now().to_msg()
                    local_map.header.frame_id = f'{ns}_map'
                    
                    local_map.data = global_merged.data
                    
                    local_map.info.resolution = global_merged.info.resolution
                    local_map.info.width = global_merged.info.width
                    local_map.info.height = global_merged.info.height
                    local_map.info.origin.orientation = global_merged.info.origin.orientation
                    
                    # Reverse TF calculate
                    try:
                        t = self.tf_buffer.lookup_transform('map', f'{ns}_map', rclpy.time.Time())
                        offset_x = t.transform.translation.x
                        offset_y = t.transform.translation.y
                    except TransformException as ex:
                        offset_x = 0.0
                        offset_y = 0.0
                        
                    local_map.info.origin.position.x = global_merged.info.origin.position.x - offset_x
                    local_map.info.origin.position.y = global_merged.info.origin.position.y - offset_y
                    local_map.info.origin.position.z = global_merged.info.origin.position.z
                    
                    if ns in self.native_map_pubs:
                        self.native_map_pubs[ns].publish(local_map)
                        
            else:
                self.get_logger().warn("Global map generation returned None.")
        else:
            self.get_logger().info("No maps received yet.")


def main(args=None):
    rclpy.init(args=args)
    node = FleetCommNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
