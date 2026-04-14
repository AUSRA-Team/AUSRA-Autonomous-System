#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select
from rclpy.qos import QoSProfile, ReliabilityPolicy

msg = """
AUSRA Latched Teleop
---------------------------
8 : Forward          2 : Backward
4 : Left             6 : Right
7 : Forward-Left     9 : Forward-Right
1 : Backward-Left    3 : Backward-Right

Arrows: Left/Right to Rotate

5 or 0 : STOP (Emergency)

[LATCHED MODE: Press once to start, press 5 to stop]
CTRL-C to quit
"""

# Directions: (x, y, angular_z)
move_bindings = {
    '8': (1.0, 0.0, 0.0),
    '2': (-1.0, 0.0, 0.0),
    '4': (0.0, 1.0, 0.0),
    '6': (0.0, -1.0, 0.0),
    '7': (0.707, 0.707, 0.0),
    '9': (0.707, -0.707, 0.0),
    '1': (-0.707, 0.707, 0.0),
    '3': (-0.707, -0.707, 0.0),
    '\x1b[D': (0.0, 0.0, 1.0),  # Left Arrow
    '\x1b[C': (0.0, 0.0, -1.0), # Right Arrow
}

stop_bindings = ['5', '0']

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    # Very fast check (10ms)
    rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            key += sys.stdin.read(2)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class NumpadTeleop(Node):
    def __init__(self):
        super().__init__('numpad_teleop')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.declare_parameter('robot_name', '')
        robot_name = self.get_parameter('robot_name').value
        topic = f'/{robot_name}/cmd_vel' if robot_name else '/cmd_vel'
            
        self.publisher_ = self.create_publisher(Twist, topic, qos_profile)
        
        # PERSISTENT STATE
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_th = 0.0
        
        # Speeds
        self.linear_speed = 0.4 
        self.angular_speed = 1.0

        # High-frequency timer (50Hz) for instant response
        self.timer = self.create_timer(0.02, self.publish_callback)
        self.get_logger().info(f'Latched Teleop active. Publishing to {topic} at 50Hz')

    def set_command(self, x, y, th):
        self.target_x = x * self.linear_speed
        self.target_y = y * self.linear_speed
        self.target_th = th * self.angular_speed

    def stop_robot(self):
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_th = 0.0

    def publish_callback(self):
        # Continuously sends the CURRENT state
        twist = Twist()
        twist.linear.x = self.target_x
        twist.linear.y = self.target_y
        twist.angular.z = self.target_th
        self.publisher_.publish(twist)

def main(args=None):
    if not sys.stdin.isatty():
        return

    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = NumpadTeleop()

    try:
        print(msg)
        while rclpy.ok():
            key = get_key(settings)
            
            if key in move_bindings:
                x, y, th = move_bindings[key]
                node.set_command(x, y, th)
            elif key in stop_bindings:
                node.stop_robot()
            elif key == '\x03': # CTRL-C
                break
            
            rclpy.spin_once(node, timeout_sec=0)
                
    except Exception as e:
        print(e)
    finally:
        node.stop_robot()
        node.publish_callback()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()