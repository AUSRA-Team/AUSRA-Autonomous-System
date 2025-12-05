from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('imu_esp32_bridge')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    
    return LaunchDescription([
        # IMU Serial Node
        Node(
            package='imu_esp32_bridge',
            executable='imu_serial_node',
            name='imu_serial_node',
            parameters=[{
                'serial_port': '/dev/ttyACM0',  # Change if needed
                'baud_rate': 115200,
                'frame_id': 'imu_link'
            }],
            output='screen'
        ),
        
        # EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_config],
            output='screen'
        ),
    ])
