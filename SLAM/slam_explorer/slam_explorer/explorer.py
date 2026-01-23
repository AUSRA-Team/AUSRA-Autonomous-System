import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        
        self.vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )
        
        self.max_speed = 0.2        # Maximum linear velocity
        self.max_turn = 0.4         # Maximum angular velocity
        self.safety_distance = 0.8  # Minimum distance to obstacles
        self.scan_cap = 4.0         # Max distance to consider
        self.power_weight = 2.0     # Emphasize closer obstacles


    def scan_callback(self, scan: LaserScan):
        ranges = np.array(scan.ranges)
        angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
        
        if len(angles) > len(ranges):
            angles = angles[:len(ranges)]

        ranges = np.nan_to_num(ranges, nan=0.0, posinf=self.scan_cap, neginf=0.0)
        ranges = np.clip(ranges, 0, self.scan_cap)

        weights = ranges ** self.power_weight

        x_components = weights * np.cos(angles)
        y_components = weights * np.sin(angles)
        
        total_x = np.sum(x_components)
        total_y = np.sum(y_components)
        
        target_heading = np.arctan2(total_y, total_x)

        closest_obstacle = np.min(ranges)
        speed_factor = np.clip((closest_obstacle - self.safety_distance), 0.0, 1.0)
        
        twist_msg = Twist()
        twist_msg.linear.x = self.max_speed * speed_factor * max(0, np.cos(target_heading))
        twist_msg.angular.z = np.clip(target_heading * 2.0, -self.max_turn, self.max_turn)

        self.vel_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    explorer = Explorer()
    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        pass
    finally:
        explorer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()