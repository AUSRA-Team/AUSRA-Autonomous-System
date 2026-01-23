import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import numpy as np

class FrontierExplorer(Node):
    def __init__(self):
        # Name set to 'forntier_explore' as requested
        super().__init__('forntier_explore')

        # --- Parameters ---
        self.declare_parameter('obstacle_distance_threshold', 0.5) # Meters
        self.declare_parameter('scan_force_gain', 1.5)             # Repulsion strength
        self.declare_parameter('goal_force_gain', 1.0)             # Attraction strength
        self.declare_parameter('max_speed', 0.25)
        self.declare_parameter('max_turn', 1.0)

        # --- State Variables ---
        self.map_data = None
        self.map_info = None
        self.robot_pose = None  # [x, y, yaw]
        
        # Laser state
        self.laser_ranges = np.array([])
        self.laser_angles = np.array([])
        
        # Recovery state
        self.previous_pos = None
        self.stuck_timer = 0
        self.is_stuck = False 

        # --- QoS Profiles ---
        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- Subscribers & Publishers ---
        self.sub_map = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, qos_map)
        
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_sensor)
        
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_sensor)

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Control Loop ---
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Node 'forntier_explore' Started. Waiting for data...")

    # --- 1. Odometry: Get Robot Position (Using Numpy) ---
    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        
        # Quaternion to Euler (Yaw) using numpy
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        self.robot_pose = np.array([p.x, p.y, yaw])

    # --- 2. Laser Scan: Pre-processing (Using Numpy) ---
    def scan_callback(self, msg):
        # Create a numpy array of angles corresponding to the ranges
        # This allows us to avoid a for-loop later
        self.laser_angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        
        # Clean data: Replace 0.0 or inf with max_range
        ranges = np.array(msg.ranges)
        ranges[ranges == 0] = msg.range_max
        ranges[np.isinf(ranges)] = msg.range_max
        self.laser_ranges = ranges

    # --- 3. Map: Frontier Detection (Using Numpy) ---
    def map_callback(self, msg):
        self.map_info = msg.info
        raw_data = np.array(msg.data, dtype=np.int8)
        self.map_data = raw_data.reshape((msg.info.height, msg.info.width))

    def get_frontier_target(self):
        """
        Calculates a target point based on the center of mass of unknown areas (-1).
        Prioritizes unknown areas closer to the robot.
        """
        if self.map_data is None or self.robot_pose is None:
            return None

        h, w = self.map_data.shape
        resolution = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y

        # Get indices of all unknown cells (-1)
        # Note: np.argwhere returns [row(y), col(x)]
        unknown_indices = np.argwhere(self.map_data == -1)
        
        if len(unknown_indices) == 0:
            return None # Map fully explored

        # Performance Optimization: Subsample (take every 20th point) to speed up calc
        subsampled = unknown_indices[::20]
        if len(subsampled) == 0:
            return None

        # Convert Grid Indices -> World Coordinates
        ys = subsampled[:, 0] * resolution + origin_y
        xs = subsampled[:, 1] * resolution + origin_x

        # Vectorized Distance Calculation
        # Calculate Euclidean distance from robot to every unknown point
        dx = xs - self.robot_pose[0]
        dy = ys - self.robot_pose[1]
        dists = np.hypot(dx, dy) # np.hypot is faster than sqrt(x**2 + y**2)

        # Weighting: Closer points have higher weight (inverse distance)
        # Add small epsilon 0.1 to prevent division by zero
        weights = 1.0 / (dists + 0.1)
        
        # Calculate Weighted Centroid
        total_weight = np.sum(weights)
        target_x = np.sum(xs * weights) / total_weight
        target_y = np.sum(ys * weights) / total_weight

        return np.array([target_x, target_y])

    # --- 4. Control Loop (Vectorized Potential Field) ---
    def control_loop(self):
        # Safety checks
        if (self.map_data is None or 
            self.robot_pose is None or 
            len(self.laser_ranges) == 0):
            return

        cmd = Twist()
        
        # --- A. Calculate Repulsion (Obstacles) ---
        # We filter laser points that are closer than threshold
        threshold = self.get_parameter('obstacle_distance_threshold').value
        
        # Boolean mask for relevant obstacles
        mask = self.laser_ranges < threshold
        
        repulsion_x = 0.0
        repulsion_y = 0.0

        if np.any(mask):
            valid_ranges = self.laser_ranges[mask]
            valid_angles = self.laser_angles[mask]

            # Calculate force magnitude: Closer = Stronger
            forces = (1.0 / valid_ranges) - (1.0 / threshold)
            
            # Vectorize: Calculate components for all rays at once
            # Negative sign because we want to push AWAY from obstacle
            f_x = -forces * np.cos(valid_angles)
            f_y = -forces * np.sin(valid_angles)
            
            # Sum up all repulsion vectors
            repulsion_x = np.sum(f_x)
            repulsion_y = np.sum(f_y)

        # --- B. Calculate Attraction (Frontier) ---
        target = self.get_frontier_target()
        attraction_x = 0.0
        attraction_y = 0.0

        if target is not None:
            # Vector from robot to target (Global Frame)
            tx, ty = target
            dx = tx - self.robot_pose[0]
            dy = ty - self.robot_pose[1]
            
            # Rotate into Robot Frame (Local)
            # x_local = dx*cos(yaw) + dy*sin(yaw)
            # y_local = -dx*sin(yaw) + dy*cos(yaw)
            yaw = self.robot_pose[2]
            c_yaw = np.cos(yaw)
            s_yaw = np.sin(yaw)
            
            attraction_x = dx * c_yaw + dy * s_yaw
            attraction_y = -dx * s_yaw + dy * c_yaw
            
            # Normalize attraction vector
            mag = np.hypot(attraction_x, attraction_y)
            if mag > 0:
                attraction_x /= mag
                attraction_y /= mag

        # --- C. Combine Forces ---
        k_rep = self.get_parameter('scan_force_gain').value
        k_att = self.get_parameter('goal_force_gain').value
        
        total_x = (k_att * attraction_x) + (k_rep * repulsion_x)
        total_y = (k_att * attraction_y) + (k_rep * repulsion_y)

        # --- D. Stuck Recovery Logic ---
        current_pos = self.robot_pose[0:2]
        if self.previous_pos is not None:
            # Check if moved less than 2cm
            dist_moved = np.linalg.norm(current_pos - self.previous_pos)
            if dist_moved < 0.02: 
                self.stuck_timer += 1
            else:
                self.stuck_timer = 0
                self.is_stuck = False
        self.previous_pos = current_pos

        # If stuck for ~5 seconds (50 * 0.1s)
        if self.stuck_timer > 50:
            self.is_stuck = True

        if self.is_stuck:
            cmd.angular.z = 0.6  # Spin
            # If spinning for ~3 seconds, reset
            if self.stuck_timer > 80:
                self.stuck_timer = 0
                self.is_stuck = False
        else:
            # --- E. Output Velocities ---
            # Linear: Clip between -0.1 (backup) and max speed
            max_speed = self.get_parameter('max_speed').value
            cmd.linear.x = float(np.clip(total_x, -0.1, max_speed))
            
            # Angular: Clip rotation
            max_turn = self.get_parameter('max_turn').value
            # Multiply y by 2.0 to make turning more aggressive
            cmd.angular.z = float(np.clip(total_y * 2.0, -max_turn, max_turn))

            # Safety: If obstacle is dangerously close in front (center 30 deg), back up
            # Assume center of array is front
            mid_idx = len(self.laser_ranges) // 2
            # Check a slice of 40 indices around center
            front_ranges = self.laser_ranges[mid_idx-20 : mid_idx+20]
            if len(front_ranges) > 0 and np.min(front_ranges) < 0.25:
                cmd.linear.x = -0.15

        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist()) # Stop robot
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()