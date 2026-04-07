#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import math
import select

msg = """
AUSRA Numpad & Arrow Teleop
---------------------------
Moving around (Numpad):
   7    8    9
   4    5    6
   1    2    3

8 : Forward
2 : Backward
4 : Left
6 : Right
7 : Forward-Left
9 : Forward-Right
1 : Backward-Left
3 : Backward-Right

Arrows:
Left  : Rotate Clockwise (0.5 rad/s)
Right : Rotate Anti-clockwise (0.5 rad/s)

0 : Do nothing (Stop)
5 : Stop

anything else : Stop

CTRL-C to quit
"""

move_bindings = {
    '8': (1.0, 0.0, 0.0),   # Forward
    '2': (-1.0, 0.0, 0.0),  # Backward
    '4': (0.0, 1.0, 0.0),   # Left
    '6': (0.0, -1.0, 0.0),  # Right
    '7': (0.707, 0.707, 0.0),   # Forward-Left
    '9': (0.707, -0.707, 0.0),  # Forward-Right
    '1': (-0.707, 0.707, 0.0),  # Backward-Left
    '3': (-0.707, -0.707, 0.0), # Backward-Right
    '0': (0.0, 0.0, 0.0),       # Do nothing (Stop)
    '5': (0.0, 0.0, 0.0),       # Stop
    '\x1b[D': (0.0, 0.0, -1.0), # Left Arrow: Clockwise
    '\x1b[C': (0.0, 0.0, 1.0),  # Right Arrow: Anti-clockwise
}

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    # Wait for up to 0.1s for input
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b': # Escape sequence (Arrows)
            key += sys.stdin.read(2)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class NumpadTeleop(Node):
    def __init__(self):
        super().__init__('numpad_teleop')
        
        self.declare_parameter('robot_name', '')
        robot_name = self.get_parameter('robot_name').value
        
        if robot_name:
            topic = f'/{robot_name}/cmd_vel'
        else:
            topic = '/cmd_vel'
            
        self.publisher_ = self.create_publisher(Twist, topic, 10)
        self.get_logger().info(f'Numpad Teleop initialized. Publishing to {topic}')
        
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5 # rad/s

    def publish_twist(self, x, y, z):
        twist = Twist()
        twist.linear.x = x * self.linear_speed
        twist.linear.y = y * self.linear_speed
        twist.angular.z = z * self.angular_speed
        self.publisher_.publish(twist)

def main(args=None):
    if not sys.stdin.isatty():
        print("Error: Numpad Teleop node must be run in a terminal (TTY) to capture keyboard input.")
        print("If you are using 'ros2 launch', try running it directly instead:")
        print("ros2 run ausra_numpad_teleop numpad_teleop --ros-args -p robot_name:=ausrabot")
        return

    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = NumpadTeleop()

    try:
        print(msg)
        while True:
            key = get_key(settings)
            if key in move_bindings.keys():
                x, y, z = move_bindings[key]
                node.publish_twist(x, y, z)
            elif key == '\x03': # CTRL-C
                break
            else:
                node.publish_twist(0.0, 0.0, 0.0)
                
    except Exception as e:
        print(e)

    finally:
        node.publish_twist(0.0, 0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
