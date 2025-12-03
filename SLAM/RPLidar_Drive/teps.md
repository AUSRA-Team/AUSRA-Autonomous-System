# Laser Scan Package – Setup & Usage Steps

## **Topic**
- **/scan**
- **Message Type:** `sensor_msgs/LaserScan`

## **Published Topics**
- **scan (`sensor_msgs/LaserScan`)**  
  Publishes scan topic from the laser.

## **Services**
- **stop_motor (`std_srvs/Empty`)**  
  Stops the RPLIDAR motor.
- **start_motor (`std_srvs/Empty`)**  
  Starts the RPLIDAR motor.

---

## **Parameters**
| Parameter         | Type   | Default         | Description |
|------------------|--------|-----------------|-------------|
| serial_port      | string | `/dev/ttyUSB0`  | Serial port name used on your system |
| serial_baudrate  | int    | `115200`        | Serial communication baud rate |
| frame_id         | string | `laser_frame`   | Frame ID for the device |
| inverted         | bool   | `false`         | Whether the LiDAR is mounted upside-down |
| angle_compensate | bool   | `false`         | Enables angle compensation |
| scan_mode        | string | *(empty)*       | Scan mode of the LiDAR |

---

## **Steps**

### **1. Plug the LiDAR**
Connect the RPLIDAR to your machine via USB.

---

### **2. Create udev rules for RPLIDAR**

Option A:
```bash
sudo chmod 777 /dev/ttyUSB0
```

Option B:
```bash
cd src/rplidar_ros/
source scripts/create_udev_rules.sh
```

---

### **3. Run RPLIDAR Node & Visualize in RViz**
```bash
ros2 launch rplidar_ros view_rplidar_a1_launch.py
```

### **4. Echo the Laser Scan Topic**
```bash
ros2 topic echo /scan
```

---

## ✔️ End of Steps
