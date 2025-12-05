#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import serial
import time

class IMUSerialNode(Node):
    def __init__(self):
        super().__init__('imu_serial_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        
        # Get parameters
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Create publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        
        # Setup serial connection
        try:
            self.serial = serial.Serial(serial_port, baud_rate, timeout=1)
            time.sleep(2)  # Wait for connection to establish
            self.get_logger().info(f'Connected to {serial_port} at {baud_rate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')
            return
        
        # Create timer to read serial data
        self.create_timer(0.01, self.read_and_publish)  # 100 Hz
        
    def read_and_publish(self):
        try:
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode('utf-8').strip()
                
                # Skip calibration messages
                if ',' not in line:
                    return
                
                # Parse CSV data: ax,ay,az,gx,gy,gz,mx,my,mz
                values = line.split(',')
                if len(values) == 9:
                    ax, ay, az = float(values[0]), float(values[1]), float(values[2])
                    gx, gy, gz = float(values[3]), float(values[4]), float(values[5])
                    mx, my, mz = float(values[6]), float(values[7]), float(values[8])
                    
                    # Publish IMU message
                    imu_msg = Imu()
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = self.frame_id
                    
                    # Linear acceleration (m/s^2)
                    imu_msg.linear_acceleration.x = ax * 9.81
                    imu_msg.linear_acceleration.y = ay * 9.81
                    imu_msg.linear_acceleration.z = az * 9.81
                    
                    # Angular velocity (rad/s)
                    imu_msg.angular_velocity.x = gx * 0.017453292519943295  # deg to rad
                    imu_msg.angular_velocity.y = gy * 0.017453292519943295
                    imu_msg.angular_velocity.z = gz * 0.017453292519943295
                    
                    # Set covariance (uncertainty estimates)
                    imu_msg.linear_acceleration_covariance[0] = 0.01
                    imu_msg.linear_acceleration_covariance[4] = 0.01
                    imu_msg.linear_acceleration_covariance[8] = 0.01
                    imu_msg.angular_velocity_covariance[0] = 0.001
                    imu_msg.angular_velocity_covariance[4] = 0.001
                    imu_msg.angular_velocity_covariance[8] = 0.001
                    
                    self.imu_pub.publish(imu_msg)
                    
                    # Publish Magnetometer message
                    mag_msg = MagneticField()
                    mag_msg.header.stamp = imu_msg.header.stamp
                    mag_msg.header.frame_id = self.frame_id
                    mag_msg.magnetic_field.x = mx * 1e-6  # Convert to Tesla
                    mag_msg.magnetic_field.y = my * 1e-6
                    mag_msg.magnetic_field.z = mz * 1e-6
                    
                    self.mag_pub.publish(mag_msg)
                    
        except Exception as e:
            self.get_logger().error(f'Error reading serial: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = IMUSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
