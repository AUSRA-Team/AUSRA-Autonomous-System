#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from collections import deque
import numpy as np

class ReactiveExplorer(Node):

    def __init__(self):
        super().__init__('reactive_explorer')

        # --- ROS interfaces ---
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)

        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        # --- State ---
        self.scan = None
        self.pose_history = deque(maxlen=50)
        self.heading_history = deque(maxlen=30)

        self.mode = 'EXPLORING'
        self.exit_start_pose = None

        # --- Exploration memory ---
        self.visited = set()
        self.grid_res = 0.5

        # --- Parameters ---
        self.safe_dist = 0.25
        self.max_lin = 0.4
        self.max_ang = 1.2

        self.rotate_count = 0

        self.local_visited = set()
        self.room_entry_pose = None


    def scan_cb(self, scan: LaserScan):
        self.scan = scan

    def odom_cb(self, odom: Odometry):
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y

        self.pose_history.append((x, y))
        self.visited.add(self.grid_key(x, y))
        
        key = self.grid_key(x, y)
        self.local_visited.add(key)


    def room_explored(self):
        return len(self.local_visited) > 120

    
    def grid_key(self, x, y):
        return (int(x / self.grid_res), int(y / self.grid_res))
    

    def is_stuck(self):
        if len(self.pose_history) < self.pose_history.maxlen:
            return False

        p0 = np.array(self.pose_history[0])
        p1 = np.array(self.pose_history[-1])

        return np.linalg.norm(p1 - p0) < 0.3
    

    def doorway_detected(self):
        count = 0
        for r in self.scan.ranges:
            if r > 2.0 or np.isinf(r):
                count += 1
                if count > 15:
                    return True
            else:
                count = 0
        return False


    def visited_repulsion(self):
        fx, fy = 0.0, 0.0

        x, y = self.pose_history[-1]
        gx, gy = self.grid_key(x, y)

        for dx in range(-3, 4):
            for dy in range(-3, 4):
                cell = (gx + dx, gy + dy)
                if cell in self.visited:
                    cx = (cell[0] + 0.5) * self.grid_res
                    cy = (cell[1] + 0.5) * self.grid_res

                    d = np.hypot(x - cx, y - cy)
                    if 0.05 < d < 1.5:
                        w = 0.6 / d
                        fx += w * (x - cx)
                        fy += w * (y - cy)

        return fx, fy


    def vector_explore(self, allow_visited_repulsion=True):

        fx, fy = 0.0, 0.0

        angle = self.scan.angle_min
        inc = self.scan.angle_increment

        for r in self.scan.ranges:
            if np.isnan(r):
                angle += inc
                continue

            if np.isinf(r):
                r = self.scan.range_max

            # Obstacle repulsion
            if r < self.safe_dist:
                w = (self.safe_dist - r) / self.safe_dist
                fx -= 1.2 * w * np.cos(angle)
                fy -= 1.2 * w * np.sin(angle)

            # Free-space attraction
            if r > 1.2:
                fx += 0.4 * np.cos(angle)
                fy += 0.4 * np.sin(angle)

            angle += inc

        # Forward bias
        fx += 0.6

        # Visited-space repulsion
        if allow_visited_repulsion:
            vx, vy = self.visited_repulsion()
            fx += vx
            fy += vy

        vec = np.array([fx, fy])
        mag = np.linalg.norm(vec)
        ang = np.arctan2(fy, fx)

        cmd = Twist()
        # --- Linear velocity clamp ---
        cmd.linear.x = np.clip(0.1 + 0.25 * mag, 0.08, self.max_lin)
        # --- Angular velocity ---
        cmd.angular.z = np.clip(ang, -self.max_ang, self.max_ang)
        # --- Angular–linear coupling (CRITICAL) ---
        if abs(cmd.angular.z) > 0.6:
            cmd.linear.x = max(cmd.linear.x, 0.12)

        self.heading_history.append(ang)
        return cmd


    def wall_follow(self):
        ranges = np.array(self.scan.ranges)
        angles = self.scan.angle_min + np.arange(len(ranges)) * self.scan.angle_increment

        idx = np.nanargmin(ranges)
        min_r = ranges[idx]
        min_ang = angles[idx]

        error = self.safe_dist - min_r

        cmd = Twist()
        cmd.linear.x = 0.3
        cmd.angular.z = 2.5 * error - min_ang
        return cmd


    def exit_commit(self):
        mid = len(self.scan.ranges) // 2
        front = self.scan.ranges[mid - 4 : mid + 4]

        cmd = Twist()
        if np.nanmin(front) < 0.4:
            cmd.angular.z = 1.0
            return cmd

        cmd.linear.x = 0.3
        return cmd


    def exit_completed(self):
        p0 = np.array(self.exit_start_pose)
        p1 = np.array(self.pose_history[-1])
        return np.linalg.norm(p1 - p0) > 1.5


    def emergency_brake(self, cmd: Twist):
        if np.nanmin(self.scan.ranges) < 0.3:
            cmd.linear.x = 0.08
            cmd.angular.z = 0.8
        return cmd
    

    def rotation_watchdog(self, cmd: Twist):
        """
        Prevent infinite in-place rotation by enforcing
        forward motion after prolonged high angular velocity.
        """

        if abs(cmd.angular.z) > 0.7 and cmd.linear.x < 0.1:
            self.rotate_count += 1
        else:
            self.rotate_count = 0

        # If rotating too long → force forward escape
        if self.rotate_count > 15:
            cmd.linear.x = max(cmd.linear.x, 0.15)
            self.rotate_count = 0  # reset after intervention

        return cmd
    

    def backtrack(self):
        cmd = self.wall_follow()
        cmd.linear.x = max(cmd.linear.x, 0.22)
        return cmd
    

    def control_loop(self):
        if self.scan is None or len(self.pose_history) < 2:
            return

        if self.mode == 'EXPLORING':
            if self.room_explored():
                self.mode = 'BACKTRACKING'
                return
            if self.is_stuck():
                self.mode = 'WALL_FOLLOW'
                return
            cmd = self.vector_explore(allow_visited_repulsion=True)

        elif self.mode == 'BACKTRACKING':
            if self.doorway_detected():
                self.mode = 'EXPLORING'
                self.local_visited.clear()
                return
            cmd = self.backtrack()

        elif self.mode == 'WALL_FOLLOW':
            if self.doorway_detected():
                self.mode = 'EXIT_COMMIT'
                self.exit_start_pose = self.pose_history[-1]
                return
            cmd = self.wall_follow()

        elif self.mode == 'EXIT_COMMIT':
            if self.exit_completed():
                self.mode = 'EXPLORING'
                self.local_visited.clear()
                self.exit_start_pose = None
                return
            cmd = self.exit_commit()

        cmd = self.emergency_brake(cmd)
        cmd = self.rotation_watchdog(cmd)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    explorer = ReactiveExplorer()
    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        pass
    finally:
        explorer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()