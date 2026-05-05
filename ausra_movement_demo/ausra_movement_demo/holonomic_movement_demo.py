#!/usr/bin/env python3
"""
Holonomic Movement Demo Node for AUSRA Robot

This node demonstrates the robot's movement capabilities:

Phase 1 (Holonomic - omni-directional):
  - Forward 1m, Backward 1m, Right 1m (strafe), Left 1m (strafe)
  - Demonstrates true holonomic movement without rotation

Phase 2 (Differential Drive simulation):
  - Same destination pattern but using only forward movement + rotations
  - Forward 1m → Rotate 180° → Forward 1m → Rotate -90° → Forward 1m → 
    Rotate 180° → Forward 1m → Rotate 90°
  - Simulates how a non-holonomic robot would achieve the same pattern
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from enum import Enum, auto


class MovementState(Enum):
    """State machine states for movement sequence"""
    IDLE = auto()
    # Phase 1: Holonomic (no rotation)
    MOVE_FORWARD = auto()
    MOVE_BACKWARD = auto()
    MOVE_RIGHT = auto()
    MOVE_LEFT = auto()
    # Phase 2: Differential Drive simulation
    P2_FORWARD_1 = auto()       # Move forward 1m
    P2_ROTATE_180_1 = auto()    # Rotate 180° (face backward)
    P2_FORWARD_2 = auto()       # Move forward 1m (going backward direction)
    P2_ROTATE_NEG90 = auto()    # Rotate -90° (face right)
    P2_FORWARD_3 = auto()       # Move forward 1m (going right)
    P2_ROTATE_180_2 = auto()    # Rotate 180° (face left)
    P2_FORWARD_4 = auto()       # Move forward 1m (going left)
    P2_ROTATE_90 = auto()       # Rotate 90° (return to original heading)
    # Complete
    COMPLETE = auto()


class HolonomicMovementDemo(Node):
    """
    ROS 2 node that demonstrates holonomic vs differential-drive movement
    by executing precise 1-meter movements in different directions.
    """

    def __init__(self):
        super().__init__('holonomic_movement_demo')

        # Robot namespace parameter for multi-robot support
        self.declare_parameter('robot_name', '')
        robot_name = self.get_parameter('robot_name').value
        
        # Construct topic names based on robot_name
        if robot_name:
            cmd_vel_topic = f'/{robot_name}/cmd_vel'
            odom_topic = f'/{robot_name}/odom'
            self.get_logger().info(f'Targeting robot: {robot_name}')
        else:
            cmd_vel_topic = '/cmd_vel'
            odom_topic = '/odom'
            self.get_logger().info('No robot_name specified, using global topics')

        # Parameters
        self.declare_parameter('linear_velocity', 0.2)  # m/s
        self.declare_parameter('angular_velocity', 0.3)  # rad/s
        self.declare_parameter('movement_distance', 1.0)  # meters
        self.declare_parameter('position_tolerance', 0.05)  # meters
        self.declare_parameter('angle_tolerance', 0.05)  # radians
        self.declare_parameter('startup_delay_secs', 3.0)  # seconds

        self.linear_vel = self.get_parameter('linear_velocity').value
        self.angular_vel = self.get_parameter('angular_velocity').value
        self.move_distance = self.get_parameter('movement_distance').value
        self.pos_tolerance = self.get_parameter('position_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        startup_delay_secs = self.get_parameter('startup_delay_secs').value

        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)

        # State machine
        self.state = MovementState.IDLE
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_theta = 0.0
        self.target_theta = 0.0  # Target angle for rotation
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.odom_received = False

        # Control timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Startup delay (wait for sensors and systems to stabilize)
        # Convert seconds to ticks at 10Hz (0.1s per tick)
        self.startup_delay = int(startup_delay_secs * 10)  # Convert to ticks
        self.startup_counter = 0

        self.get_logger().info('Holonomic Movement Demo initialized')
        self.get_logger().info(f'Waiting {startup_delay_secs}s for systems to stabilize...')

    def odom_callback(self, msg: Odometry):
        """Extract position and orientation from odometry"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)

        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info(f'Odometry received. Initial pose: '
                                   f'x={self.current_x:.2f}, y={self.current_y:.2f}, '
                                   f'theta={math.degrees(self.current_theta):.1f}°')

    def get_distance_traveled(self) -> float:
        """Calculate distance traveled from start position"""
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        return math.sqrt(dx * dx + dy * dy)

    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def get_angle_error(self) -> float:
        """Calculate signed angle error to target"""
        error = self.target_theta - self.current_theta
        return self.normalize_angle(error)

    def set_start_pose(self):
        """Record current pose as start for next movement"""
        self.start_x = self.current_x
        self.start_y = self.current_y
        self.start_theta = self.current_theta

    def stop_robot(self):
        """Publish zero velocity command"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def move_direction(self, vx: float, vy: float, omega: float = 0.0):
        """Publish velocity command"""
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = omega
        self.cmd_vel_pub.publish(cmd)

    def control_loop(self):
        """Main control loop - state machine execution"""
        # Startup delay
        if self.startup_counter < self.startup_delay:
            self.startup_counter += 1
            if self.startup_counter == self.startup_delay:
                self.get_logger().info('='*60)
                self.get_logger().info('Starting Phase 1: HOLONOMIC movement (strafing)')
                self.get_logger().info('='*60)
                self.state = MovementState.MOVE_FORWARD
                self.set_start_pose()
            return

        if not self.odom_received:
            return

        # State machine
        if self.state == MovementState.IDLE:
            pass

        # ============ PHASE 1: Holonomic (Strafing) ============
        elif self.state == MovementState.MOVE_FORWARD:
            if self.get_distance_traveled() < self.move_distance - self.pos_tolerance:
                self.move_direction(self.linear_vel, 0.0)
            else:
                self.stop_robot()
                self.get_logger().info('✓ Phase 1: Moved 1m FORWARD (holonomic)')
                self.state = MovementState.MOVE_BACKWARD
                self.set_start_pose()

        elif self.state == MovementState.MOVE_BACKWARD:
            if self.get_distance_traveled() < self.move_distance - self.pos_tolerance:
                self.move_direction(-self.linear_vel, 0.0)
            else:
                self.stop_robot()
                self.get_logger().info('✓ Phase 1: Moved 1m BACKWARD (holonomic)')
                self.state = MovementState.MOVE_RIGHT
                self.set_start_pose()

        elif self.state == MovementState.MOVE_RIGHT:
            if self.get_distance_traveled() < self.move_distance - self.pos_tolerance:
                self.move_direction(0.0, -self.linear_vel)
            else:
                self.stop_robot()
                self.get_logger().info('✓ Phase 1: Moved 1m RIGHT (strafe - holonomic)')
                self.state = MovementState.MOVE_LEFT
                self.set_start_pose()

        elif self.state == MovementState.MOVE_LEFT:
            if self.get_distance_traveled() < self.move_distance - self.pos_tolerance:
                self.move_direction(0.0, self.linear_vel)
            else:
                self.stop_robot()
                self.get_logger().info('✓ Phase 1: Moved 1m LEFT (strafe - holonomic)')
                self.get_logger().info('='*60)
                self.get_logger().info('Phase 1 Complete!')
                self.get_logger().info('Starting Phase 2: DIFFERENTIAL DRIVE simulation')
                self.get_logger().info('(Only forward movement + rotations)')
                self.get_logger().info('='*60)
                self.state = MovementState.P2_FORWARD_1
                self.set_start_pose()

        # ============ PHASE 2: Differential Drive Simulation ============
        elif self.state == MovementState.P2_FORWARD_1:
            if self.get_distance_traveled() < self.move_distance - self.pos_tolerance:
                self.move_direction(self.linear_vel, 0.0)
            else:
                self.stop_robot()
                self.get_logger().info('✓ Phase 2: Moved 1m FORWARD')
                # Set target to rotate 180°
                self.target_theta = self.normalize_angle(self.current_theta + math.pi)
                self.state = MovementState.P2_ROTATE_180_1
                self.set_start_pose()

        elif self.state == MovementState.P2_ROTATE_180_1:
            error = self.get_angle_error()
            if abs(error) > self.angle_tolerance:
                # Rotate in direction of error
                omega = self.angular_vel if error > 0 else -self.angular_vel
                self.move_direction(0.0, 0.0, omega)
            else:
                self.stop_robot()
                self.get_logger().info('✓ Phase 2: Rotated 180°')
                self.state = MovementState.P2_FORWARD_2
                self.set_start_pose()

        elif self.state == MovementState.P2_FORWARD_2:
            if self.get_distance_traveled() < self.move_distance - self.pos_tolerance:
                self.move_direction(self.linear_vel, 0.0)
            else:
                self.stop_robot()
                self.get_logger().info('✓ Phase 2: Moved 1m FORWARD (going backward direction)')
                # Set target to rotate -90° (turn right)
                self.target_theta = self.normalize_angle(self.current_theta - math.pi/2)
                self.state = MovementState.P2_ROTATE_NEG90
                self.set_start_pose()

        elif self.state == MovementState.P2_ROTATE_NEG90:
            error = self.get_angle_error()
            if abs(error) > self.angle_tolerance:
                omega = self.angular_vel if error > 0 else -self.angular_vel
                self.move_direction(0.0, 0.0, omega)
            else:
                self.stop_robot()
                self.get_logger().info('✓ Phase 2: Rotated -90° (facing right)')
                self.state = MovementState.P2_FORWARD_3
                self.set_start_pose()

        elif self.state == MovementState.P2_FORWARD_3:
            if self.get_distance_traveled() < self.move_distance - self.pos_tolerance:
                self.move_direction(self.linear_vel, 0.0)
            else:
                self.stop_robot()
                self.get_logger().info('✓ Phase 2: Moved 1m FORWARD (going right direction)')
                # Set target to rotate 180°
                self.target_theta = self.normalize_angle(self.current_theta + math.pi)
                self.state = MovementState.P2_ROTATE_180_2
                self.set_start_pose()

        elif self.state == MovementState.P2_ROTATE_180_2:
            error = self.get_angle_error()
            if abs(error) > self.angle_tolerance:
                omega = self.angular_vel if error > 0 else -self.angular_vel
                self.move_direction(0.0, 0.0, omega)
            else:
                self.stop_robot()
                self.get_logger().info('✓ Phase 2: Rotated 180° (facing left)')
                self.state = MovementState.P2_FORWARD_4
                self.set_start_pose()

        elif self.state == MovementState.P2_FORWARD_4:
            if self.get_distance_traveled() < self.move_distance - self.pos_tolerance:
                self.move_direction(self.linear_vel, 0.0)
            else:
                self.stop_robot()
                self.get_logger().info('✓ Phase 2: Moved 1m FORWARD (going left direction)')
                # Set target to rotate 90° (return to original heading)
                self.target_theta = self.normalize_angle(self.current_theta + math.pi/2)
                self.state = MovementState.P2_ROTATE_90
                self.set_start_pose()

        elif self.state == MovementState.P2_ROTATE_90:
            error = self.get_angle_error()
            if abs(error) > self.angle_tolerance:
                omega = self.angular_vel if error > 0 else -self.angular_vel
                self.move_direction(0.0, 0.0, omega)
            else:
                self.stop_robot()
                self.get_logger().info('✓ Phase 2: Rotated 90° (back to original heading)')
                self.state = MovementState.COMPLETE

        elif self.state == MovementState.COMPLETE:
            self.stop_robot()
            self.get_logger().info('='*60)
            self.get_logger().info('🎉 DEMO COMPLETE!')
            self.get_logger().info('Phase 1 demonstrated HOLONOMIC movement (strafing)')
            self.get_logger().info('Phase 2 demonstrated DIFFERENTIAL DRIVE simulation')
            self.get_logger().info('='*60)
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = HolonomicMovementDemo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Demo interrupted by user')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

