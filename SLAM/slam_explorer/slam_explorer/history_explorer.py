import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math

class HistoryExplorer(Node):
    def __init__(self):
        super().__init__('history_explorer')
        
        # Subscriptions
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- TUNING PARAMETERS ---
        self.max_speed = 0.25
        self.max_turn = 0.5
        self.safety_distance = 0.5  # If closer than this, TRIGGER ESCAPE MODE
        self.scan_cap = 5.0
        
        # Memory Settings
        self.grid_resolution = 0.2
        self.grid_size = 400
        self.robot_radius_grid = 3 
        
        self.memory_grid = np.zeros((self.grid_size, self.grid_size))
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.initialized = False

    def odom_callback(self, odom: Odometry):
        self.robot_x = odom.pose.pose.position.x
        self.robot_y = odom.pose.pose.position.y
        
        q = odom.pose.pose.orientation
        self.robot_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        
        center_gx = int(self.robot_x / self.grid_resolution) + self.grid_size // 2
        center_gy = int(self.robot_y / self.grid_resolution) + self.grid_size // 2
        
        r = self.robot_radius_grid
        for dx in range(-r, r+1):
            for dy in range(-r, r+1):
                nx, ny = center_gx + dx, center_gy + dy
                if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                    self.memory_grid[nx, ny] = min(self.memory_grid[nx, ny] + 1.0, 20.0)
        
        self.initialized = True

    def scan_callback(self, scan: LaserScan):
        if not self.initialized:
            return

        twist = Twist()
        ranges = np.array(scan.ranges)
        angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
        
        if len(angles) > len(ranges): angles = angles[:len(ranges)]

        # Clean data
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=self.scan_cap, neginf=0.0)
        
        # ==========================================
        # PRIORITY 1: SURVIVAL (Escape Mode)
        # ==========================================
        # Find the ABSOLUTE closest obstacle
        closest_idx = np.argmin(ranges)
        min_dist = ranges[closest_idx]
        
        if min_dist < self.safety_distance:
            # We are too close to something!
            # Logic: Turn AWAY from the closest point.
            obstacle_angle = angles[closest_idx]
            
            # The direction we want is Obstacle Angle + 180 degrees (PI)
            escape_heading = obstacle_angle + math.pi
            
            # Normalize angle to -pi to pi
            escape_heading = math.atan2(math.sin(escape_heading), math.cos(escape_heading))
            
            # Action: Backup slowly and spin fast
            twist.linear.x = -0.05 
            # If escape heading is positive, turn left, else right
            twist.angular.z = 0.8 if escape_heading > 0 else -0.8
            
            self.vel_pub.publish(twist)
            return # Skip the rest of the logic

        # ==========================================
        # PRIORITY 2 & 3: EXPLORATION
        # ==========================================
        
        ranges = np.clip(ranges, 0, self.scan_cap)
        weights = []
        
        for i, r in enumerate(ranges):
            global_angle = self.robot_yaw + angles[i]
            tip_x = self.robot_x + r * math.cos(global_angle)
            tip_y = self.robot_y + r * math.sin(global_angle)
            
            gx = int(tip_x / self.grid_resolution) + self.grid_size // 2
            gy = int(tip_y / self.grid_resolution) + self.grid_size // 2
            
            visit_factor = 0.0
            if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                visit_factor = self.memory_grid[gx, gy]

            # Weight Formula
            w = (r ** 2) / (1.0 + (visit_factor ** 3)) 
            weights.append(w)

        weights = np.array(weights)
        x_components = weights * np.cos(angles)
        y_components = weights * np.sin(angles)
        
        total_x = np.sum(x_components)
        total_y = np.sum(y_components)
        magnitude = math.sqrt(total_x**2 + total_y**2)
        
        # Check if we are "Bored/Stuck" (Weak vector sum)
        if magnitude < 5.0:
            # Spin in place to find new frontiers
            twist.linear.x = 0.0
            twist.angular.z = 0.6
        else:
            # Standard Exploration Drive
            target_heading = np.arctan2(total_y, total_x)
            
            # Smooth drive
            twist.linear.x = self.max_speed * max(0.1, np.cos(target_heading))
            twist.angular.z = np.clip(target_heading * 2.5, -self.max_turn, self.max_turn)

        self.vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    explorer = HistoryExplorer()
    rclpy.spin(explorer)
    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()