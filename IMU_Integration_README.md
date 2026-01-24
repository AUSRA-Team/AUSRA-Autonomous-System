# IMU Integration - MPU-9250 with ESP32 Bridge

## Branch: IMU_test
## Repository: AUSRA-Autonomous-System

This branch contains the integration of the MPU-9250 9-axis IMU sensor for the AUSRA autonomous swarm robot system.

## What's Included

### 1. ROS 2 Package (`src/imu_esp32_bridge/`)
- **Node**: `imu_serial_node` - Reads IMU data from ESP32 via serial and publishes to ROS 2
- **Launch file**: `imu_ekf_launch.py` - Launches IMU node with EKF filtering
- **Config**: `ekf.yaml` - Extended Kalman Filter configuration

### 2. Arduino Code (`hardware/arduino/mpu9250_esp32/`)
- ESP32-S3 firmware for reading MPU-9250 sensor data
- Automatic calibration on startup
- Outputs accelerometer, gyroscope, and magnetometer data via serial

## Hardware Setup

### Components
- MPU-9250 9-axis IMU
- ESP32-S3 development board
- USB cable

### Wiring
| MPU-9250 | ESP32-S3 |
|----------|----------|
| VCC      | 3.3V     |
| GND      | GND      |
| SDA      | GPIO 17  |
| SCL      | GPIO 18  |

## Software Dependencies
```bash
# Install required packages
sudo apt install python3-pip python3-serial
sudo apt install ros-humble-robot-localization ros-humble-imu-tools
pip3 install pyserial
```

## Building the Package
```bash
# In your ROS 2 workspace
cd ~/AUSRA-Autonomous-System
colcon build --packages-select imu_esp32_bridge
source install/setup.bash
```

## Usage

### 1. Upload Arduino Code
- Open `hardware/arduino/mpu9250_esp32/mpu9250_esp32.ino` in Arduino IDE
- Select board: ESP32S3 Dev Module
- Upload to ESP32
- **Keep IMU still during calibration (first 2 seconds)**

### 2. Run IMU Node (Basic)
```bash
# Find your ESP32 port
ls /dev/ttyACM* /dev/ttyUSB*

# Give permissions
sudo chmod 666 /dev/ttyACM0  # Replace with your port

# Run the node
ros2 run imu_esp32_bridge imu_serial_node --ros-args -p serial_port:=/dev/ttyACM0
```

### 3. Run with EKF Filter (Recommended)
```bash
ros2 launch imu_esp32_bridge imu_ekf_launch.py
```

## Published Topics

- `/imu/data_raw` (sensor_msgs/Imu) - Raw IMU measurements
- `/imu/mag` (sensor_msgs/MagneticField) - Magnetometer data
- `/odometry/filtered` (nav_msgs/Odometry) - EKF-filtered pose and velocity

## Testing & Verification

### Check if data is publishing
```bash
ros2 topic list
ros2 topic echo /imu/data_raw
```

### Visualize in RViz2
```bash
ros2 run rviz2 rviz2
# Set Fixed Frame to: imu_link
# Add -> By topic -> /imu/data_raw -> Imu
```

### Check filtered output
```bash
ros2 topic echo /odometry/filtered
```

## Integration Notes for AUSRA Team

### For Localization Team
- The EKF node publishes on `/odometry/filtered`
- Frame ID is `imu_link` - you'll need to add TF transform to `base_link`
- Covariance values are configured in `config/ekf.yaml`

### For Simulation Team
- You can replace the serial node with Gazebo IMU plugin for simulation
- Keep the same topic names (`/imu/data_raw`) for consistency
- EKF configuration will work the same

### For Multi-Robot Swarm System
- Add namespace parameter to launch file for multiple robots
- Example: `robot1/imu/data_raw`, `robot2/imu/data_raw`
- Modify launch file to accept robot_id parameter

## Known Issues & Solutions

### "Device or resource busy" error
- **Cause**: Arduino Serial Monitor is open
- **Fix**: Close Serial Monitor or run `sudo killall -9 arduino`

### No data appearing
- **Check**: ESP32 is outputting data (open Serial Monitor at 115200 baud)
- **Check**: Wiring connections
- **Check**: Port permissions

### EKF not converging
- **Tune**: Adjust process noise covariance in `config/ekf.yaml`
- **Check**: IMU is calibrated properly (keep still during startup)

## Testing Status
- [x] Hardware connection verified
- [x] Arduino code tested
- [x] ROS 2 node publishes data
- [x] EKF integration working
- [ ] Tested with full robot system (pending Jetson Orin Nano)
- [ ] Multi-robot namespace testing (pending)
- [ ] Integration with omni-directional driver (pending)

## Next Steps
1. Test integration with odometry from wheel encoders
2. Add TF transforms between `imu_link` and `base_link`
3. Integrate with navigation stack
4. Test on actual Jetson Orin Nano
5. Test with swarm coordination system


## Questions or Issues?
Contact me or open an issue in this repository.
