# AUSRA Autonomous System — Real Hardware ROS 2 Software Stack

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![OS](https://img.shields.io/badge/OS-Ubuntu%2022.04%20LTS-orange.svg)](https://releases.ubuntu.com/22.04/)
[![Compute](https://img.shields.io/badge/Compute-NVIDIA%20Jetson%20Orin%20Nano-green.svg)](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)
[![MCU](https://img.shields.io/badge/MCU-ESP32--S3-red.svg)](https://www.espressif.com/en/products/socs/esp32-s3)
[![Middleware](https://img.shields.io/badge/Middleware-CycloneDDS%20%7C%20Zenoh-purple.svg)](https://zenoh.io/)

This repository contains the high-level ROS 2 Humble software stack, hardware bringup launch files, supervisor state machine, SLAM/Nav2 configs, and Zenoh fleet bridges for the **AUSRA** 3-wheel omnidirectional mobile robot platform.

---

## 🎬 Real-World Results & System Showcase

The **AUSRA Autonomous System** is engineered for real-world autonomous search-and-rescue operations, multi-robot fleet mapping, and precise victim/target localization. Below are the operational video demonstrations highlighting our hardware and simulation results:

### 🤖 1. Autonomous Single-Robot Frontier Exploration
Physical 3-wheel omnidirectional AUSRA robot dynamically mapping unknown terrain using RPLIDAR A1, `slam_toolbox`, and autonomous frontier exploration (`explore_lite`).

<p align="center">
  <video src="docs/single_robot_exploration.mp4" width="85%" controls loop muted></video>
</p>

---

### 🌐 2. Multi-Robot Fleet Swarm & Distributed Map Fusion
Multi-agent fleet coordination and real-time distributed map merging (`ausra_map_merge_HW`) over a low-latency Zenoh cross-WiFi network bridge.

<p align="center">
  <video src="docs/multi_robot_simualtion.mp4" width="85%" controls loop muted></video>
</p>

---

### 🎯 Real-Time Victim & Target Localization Pipeline
Real-world hardware camera feed synchronized with real-time RViz spatial mapping on the global occupancy grid:

<table width="100%">
  <tr>
    <td width="50%" align="center" valign="top">
      <h3>Real Hardware Camera Feed</h3>
      <video src="docs/victim_localization_real.mp4" width="100%" controls loop muted></video>
      <p><em>On-board hardware vision and sensor pipeline detecting and localizing targets/victims in real physical environments.</em></p>
    </td>
    <td width="50%" align="center" valign="top">
      <h3>Real-Time RViz Global Map</h3>
      <video src="docs/victim_localization_map.mp4" width="100%" controls loop muted></video>
      <p><em>Real-time victim spatial mapping and target coordinate estimation rendered directly on the global RViz occupancy grid.</em></p>
    </td>
  </tr>
</table>

---

## 1. System Architecture & Topology

### Control Hierarchy

- **High-Level "Brain" (NVIDIA Jetson Orin Nano):** ROS 2 Humble on Ubuntu 22.04. Runs SLAM (`slam_toolbox`), EKF (`robot_localization`), holonomic planning (`Nav2`), health supervisor (`ausra_supervisor`), and fleet comms (`Zenoh`).
- **Low-Level "Spine" (ESP32-S3 Microcontroller):** Runs micro-ROS over USB-serial (`/dev/ttyACM0`). Executes 33 Hz PID motor control, quadrature encoder sampling, and inverse kinematics.

<p align="center">
  <img src="docs/figures/Chapter 5/Hardware_architecture.png" width="500" alt="Hardware Architecture"/>
  <br/>
  <em>Fig 1.1: System Hardware Architecture Block Diagram.</em>
</p>

### Electrical System Topology

<p align="center">
  <img src="docs/figures/Chapter 4/System_topology.png" width="500" alt="System Topology"/>
  <br/>
  <em>Fig 1.2: Power distribution topology (20V battery $\rightarrow$ buck converters $\rightarrow$ Jetson Orin Nano & ESP32-S3).</em>
</p>

---

## 2. Physical Robot Assembly

| Locomotion Base Plate | Control Hub Layer |
| :---: | :---: |
| <img src="docs/figures/Chapter 3/Locomotion_and_Power_Drive_Base_Plate_exploded_view.jpg" width="280" alt="Locomotion Base Plate"/> | <img src="docs/figures/Chapter 3/Integrated_Power_Control_and_Signal_Integrity.jpg" width="280" alt="Control Hub Layer"/> |
| *Bottom Tier: 3x 120° omni-wheels & JGY-370 motors.* | *Middle Tier: ESP32-S3 MCU, power buck converters & battery hub.* |

---

## 3. Hardware Components & Coordinate Frames

### Component Selection Gallery

| Main SBC | Microcontroller | 360° LiDAR | 6-Axis IMU | Motor Driver | Power Source |
| :---: | :---: | :---: | :---: | :---: | :---: |
| <img src="docs/figures/Chapter 5/Jetson_orin_nano.jpg" width="90"/> | <img src="docs/figures/Chapter 4/Esp32-s3.png" width="90"/> | <img src="docs/figures/Chapter 5/rplidar.jpg" width="90"/> | <img src="docs/figures/Chapter 5/IMU_image.png" width="90"/> | <img src="docs/figures/Chapter 4/MDD3A_Motor_Driver_2_Channel.png" width="90"/> | <img src="docs/figures/Chapter 4/Total_p20s_battery.png" width="90"/> |
| **Jetson Orin Nano** | **ESP32-S3** | **RPLIDAR A1** | **MPU6050** | **Cytron MDD3A** | **TOTAL 20V** |

### TF Coordinate Frame Hierarchy

Launching with `robot_name:=<name>` dynamically prefixes all TF frames:

<p align="center">
  <img src="docs/figures/Chapter 5/Frames_axis_rviz2.jpeg" width="450" alt="TF Frames in RViz2"/>
  <br/>
  <em>Fig 3.1: Active TF Frame Tree in RViz2.</em>
</p>

| Frame Name | Description | Parent Frame | Source Publisher |
| :--- | :--- | :--- | :--- |
| `map` | Fixed global reference frame | Ground truth | SLAM / Map Server |
| `<robot_name>_map` | Local robot map frame | `map` | `static_transform_publisher` |
| `<robot_name>_odom` | Odometry drift frame | `<robot_name>_map` | `robot_localization` (EKF) |
| `<robot_name>_robot_footprint` | Robot base footprint | `<robot_name>_odom` | `omnidirectional_driver` / EKF |
| `<robot_name>_lidar` | RPLIDAR optical frame | `<robot_name>_robot_footprint` | `robot_state_publisher` (URDF) |
| `<robot_name>_imu_link` | MPU6050 IMU frame | `<robot_name>_robot_footprint` | `robot_state_publisher` (URDF) |

---

## 4. IMU Schematics & Custom PCB

| IMU Circuit Schematic | Custom Interface PCB |
| :---: | :---: |
| <img src="docs/figures/Chapter 5/IMU_circuit_schematic.png" width="260"/> | <img src="docs/figures/Chapter 5/IMU_pcb_2.png" width="260"/> |
| *I2C connection schematic (3.3V, SCL, SDA @ 400 kHz).* | *Assembled PCB with screw terminals & bypass capacitors.* |

---

## 5. FreeRTOS & Micro-ROS Architecture

### Real-Time FreeRTOS Dual-Core Allocation

<p align="center">
  <img src="docs/figures/Chapter 4/RTOS_dual_core.png" width="450" alt="FreeRTOS Dual-Core"/>
  <br/>
  <em>Fig 5.1: Dual-core task split (Core 0: Micro-ROS @ 50 Hz; Core 1: PID Control @ 33 Hz).</em>
</p>

### Micro-ROS XRCE-DDS Transport Layer

<p align="center">
  <img src="docs/figures/Chapter 4/microros_architecture_image.jpeg" width="420" alt="Micro-ROS Architecture"/>
  <br/>
  <em>Fig 5.2: Micro-ROS Client-Agent bridge over USB-CDC serial (115200 baud).</em>
</p>

### DDS Setup (`jetson_ros_env.sh`)

Source this file in every terminal:

```bash
source ~/ausra_NM_ws/src/AUSRA-Autonomous-System/jetson_ros_env.sh
```

```bash
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>500</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
```

> [!IMPORTANT]
> **Do NOT set `ROS_LOCALHOST_ONLY=1`**. `micro_ros_agent` uses FastDDS on WiFi. Setting `ROS_LOCALHOST_ONLY=1` restricts CycloneDDS to loopback, making micro-ROS topics invisible to ROS 2 nodes.

---

## 6. Repository Structure & Supervisor States

```
AUSRA-Autonomous-System/
├── ausra_bringup/                     # Central production hardware launch & systemd
│   ├── launch/hardware_full_stack.launch.py  # Primary multi-stage single/multi-robot bringup
│   └── systemd/ausra-supervisor.service     # Systemd autostart service
├── ausra_supervisor/                  # State machine supervisor node
├── ausra_comms/                       # Zenoh network & fleet bridge
│   └── launch/hardware_with_comms.launch.py  # Full hardware + Zenoh bridge bringup
├── lidar_slam_pkg/                    # Hardware SLAM & Nav2 configs
│   └── launch/slam.launch.py          # Standalone SLAM mapping launch
├── ausra_movement_demo/               # Motion validation suite
│   └── launch/holonomic_demo.launch.py# Distance motion test launch
├── ausra_numpad_teleop/               # Numpad teleop tool
│   └── launch/numpad_teleop.launch.py # Teleoperation launch
├── ausra_map_merge_HW/                # Multi-robot real hardware map merger
│   └── launch/map_merge_hw.launch.py  # Networked map merge launch
├── jetson_ros_env.sh                  # DDS environment setup
├── start_micro_ros_agent.sh           # Micro-ROS launcher with DTR reset
└── start__drivers_for_tune_ekf.sh     # EKF diagnostic tool
```

### Supervisor State Machine (`ausra_supervisor`)

| State ID | Enum Name | Description | Priority / Behavior |
| :---: | :--- | :--- | :--- |
| `0` | **`STATE_IDLE`** | Robot idle, ready for tasks. | Normal status broadcast. |
| `1` | **`STATE_NAVIGATING`** | Autonomous Nav2 goal active. | Rejects new task goals. |
| `2` | **`STATE_DEGRADED`** | Sensor watchdog timeout. | Triggers safety speed limit. |
| `3` | **`STATE_ESTOP`** | Emergency Stop active. | Cancels Nav2, halts motors. |
| `4` | **`STATE_LOST`** | Robot unlocalized (fleet-side). | Reported to fleet manager. |
| `5` | **`STATE_EXPLORING`** | Autonomous exploration active. | Runs managed `explore_lite` process. |

---

## 7. Launch Files Reference

The repository provides modular ROS 2 launch files tailored for specific deployment stages:

| Launch File | Package | Key Arguments | Purpose / Description |
| :--- | :--- | :--- | :--- |
| **`hardware_full_stack.launch.py`** | `ausra_bringup` | `robot_name:=ausra_1`<br/>`use_sim_time:=false`<br/>`nudge_robot:=false`<br/>`x:=0.0 y:=0.0 yaw:=0.0` | **Primary Unified Hardware Bringup**: Executes multi-stage launch (Stage 0: Drivers $\rightarrow$ Stage 1: EKF+SLAM $\rightarrow$ Stage 2: Nav2 stack). |
| **`hardware_with_comms.launch.py`** | `ausra_comms` | `robot_name:=ausra_1`<br/>`use_zenoh:=true`<br/>`enable_compression:=true` | **Full Hardware + Fleet Comms**: Wraps `hardware_full_stack.launch.py` and launches `relay_node` + `zenoh-bridge-ros2dds`. |
| **`slam.launch.py`** | `lidar_slam_pkg` | `serial_port:=/dev/ttyUSB0`<br/>`use_rviz:=false` | **Standalone SLAM Mapping**: Launches RSP, Omni driver, RPLIDAR A1, and `slam_toolbox` for manual map building. |
| **`holonomic_demo.launch.py`** | `ausra_movement_demo` | `movement_distance:=1.0`<br/>`linear_velocity:=0.2`<br/>`angular_velocity:=0.3` | **Motion Validation**: Executes automated omnidirectional strafe and rotation sequences to test physical odometry accuracy. |
| **`numpad_teleop.launch.py`** | `ausra_numpad_teleop` | N/A | **Keyboard Teleop**: Launches keyboard numpad node for driving the robot during mapping. |
| **`map_merge_hw.launch.py`** | `ausra_map_merge_HW` | `robot_config:="ausra_1:0.0:0.0 ausra_2:1.5:0.0"` | **Multi-Robot Map Merger**: Runs on laptop/base-station to merge individual robot maps into `/map_merged`. |

---

## 8. Multi-Stage Launch Architecture (`hardware_full_stack.launch.py`)

The production launch script (`hardware_full_stack.launch.py`) uses strict time-delayed stages (`TimerAction` + `GroupAction` + `PushRosNamespace`) to prevent CPU spikes and guarantee stable node startup:

```
[t = 0.0s] Stage 0: Core Hardware & Description
   ├── robot_state_publisher (Xacro URDF)
   ├── omnidirectional_driver (Kinematics & Wheel Odom)
   ├── sllidar_ros2 Driver (/dev/ttyUSB0 @ 115200)
   ├── mpu6050driver IMU (I2C /dev/i2c-1)
   └── static_transform_publisher (map -> ausra_1_map)
         │
[t = 5.0s] Stage 1: Localization & SLAM
   ├── robot_localization (EKF IMU + Wheel Odom Fusion)
   └── slam_toolbox (Async SLAM Mapping)
         │
[t = 15.0s] Stage 2: Nav2 Navigation Stack
   ├── controller_server (DWB Holonomic Local Planner)
   ├── planner_server (A* Global Planner)
   ├── behavior_server (Recovery Behaviors)
   ├── bt_navigator (Behavior Tree Engine)
   ├── velocity_smoother & waypoint_follower
   └── nav2_lifecycle_manager (Autostarts Nav2 nodes)
         │
[t = 35.0s] Stage 4: Optional Initial Nudge (nudge_robot:=true)
   └── Temporary rotational pulse via /cmd_vel to seed SLAM scan alignment
```

---

## 9. How to Run the Full Pipeline Correctly (Step-by-Step)

Follow this sequence to launch the physical robot from power-on to full autonomous navigation and multi-robot fleet operation.

### Step 0: Environment Setup (All Terminals)
Open a terminal on the Jetson Orin Nano and source the workspace environment:
```bash
source ~/ausra_NM_ws/src/AUSRA-Autonomous-System/jetson_ros_env.sh
```

---

### Step 1: Start Micro-ROS Agent & MCU Namespace
Connect the ESP32-S3 via USB (`/dev/ttyACM0`) and run the launcher script:
```bash
./start_micro_ros_agent.sh ausra_1 /dev/ttyACM0
```
*Verification:* Wait until `Session established!` appears in terminal output.

---

### Step 2: Launch the Hardware Stack

- **Option A: Single Standalone Robot (No Fleet Bridge)**
  ```bash
  ros2 launch ausra_bringup hardware_full_stack.launch.py robot_name:=ausra_1 nudge_robot:=false
  ```

- **Option B: Fleet-Connected Robot (With Zenoh Cross-WiFi Bridge)**
  ```bash
  ros2 launch ausra_comms hardware_with_comms.launch.py robot_name:=ausra_1 use_zenoh:=true
  ```

---

### Step 3: Create & Save an Initial Environment Map

1. In a new terminal on the Jetson, start teleoperation:
   ```bash
   ros2 run ausra_numpad_teleop numpad_teleop
   ```
2. Drive the robot around the room to construct the occupancy grid in `slam_toolbox`.
3. Save the map:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/maps/my_lab_map
   ```

| Physical Laser Measurement Benchmark | RViz SLAM Map Verification |
| :---: | :---: |
| <img src="docs/figures/Chapter 6/hw_tape_measure_photo.jpeg" width="280"/> | <img src="docs/figures/Chapter 6/hw_tape_measure_rviz.jpeg" width="280"/> |
| *Physical distance measurement in lab.* | *Corresponding `slam_toolbox` map.* |

---

### Step 4: Motion Validation & Autonomous Nav2 Goal Execution

#### A. Validate Holonomic Motion Accuracy
Run the automated distance test script:
```bash
ros2 launch ausra_movement_demo holonomic_demo.launch.py movement_distance:=0.5 linear_velocity:=0.2
```

#### B. Autonomous Navigation via Nav2
1. Open RViz on your laptop/host connected to the ROS network.
2. Set initial pose using the **2D Pose Estimate** button.
3. Set destination using the **Nav2 Goal** button. The DWB local planner will output holonomic velocity vectors ($v_x, v_y, \omega$) to navigate around obstacles.

---

### Step 5: Trigger Autonomous Frontier Exploration

To start autonomous exploration without teleoperation, publish an exploration command to `ausra_supervisor`:
```bash
ros2 topic pub --once /ausra_1/supervisor/explore_cmd std_msgs/msg/Bool "{data: true}"
```
*The supervisor node dynamically transitions state to `STATE_EXPLORING (5)` and launches a managed `explore_lite` subprocess until all frontiers are cleared.*

---

### Step 6: Multi-Robot Fleet Map Merging & GUI (Laptop Side)

For multi-robot deployments (`ausra_1`, `ausra_2`):

1. **Launch Map Merger on Base Station Laptop:**
   ```bash
   ros2 launch ausra_map_merge_HW map_merge_hw.launch.py robot_config:="ausra_1:0.0:0.0 ausra_2:1.5:0.0"
   ```
2. **Launch Central Fleet Commander GUI:**
   ```bash
   ros2 run ausra_comms fleet_gui
   ```

| Real-World Multi-Robot Deployment | Real-Time Merged Map (`ausra_map_merge_HW`) |
| :---: | :---: |
| <img src="docs/figures/Chapter 6/hw_final_real_world_final_poses.jpeg" width="280"/> | <img src="docs/figures/Chapter 6/hw_final_rviz_map.jpeg" width="280"/> |
| *Physical multi-robot deployment.* | *Merged global map via Zenoh.* |

<p align="center">
  <img src="docs/figures/Chapter 6/gui_tab_fleet_commander.png" width="500" alt="Fleet Commander GUI"/>
  <br/>
  <em>Fig 9.1: Centralized Fleet Commander GUI Interface.</em>
</p>

---

## 10. Production Autostart via Systemd

```bash
sudo cp ~/ausra_NM_ws/src/AUSRA-Autonomous-System/ausra_bringup/systemd/ausra-supervisor.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now ausra-supervisor.service
```

---

## 11. Hardware Troubleshooting Matrix

| Issue / Symptom | Root Cause | Resolution |
| :--- | :--- | :--- |
| **`Permission denied: '/dev/ttyUSB0'` / `ttyACM0`** | Missing udev/group permissions | Run `sudo usermod -a -G dialout $USER && sudo chmod 666 /dev/ttyUSB0 /dev/ttyACM0`. Log out and log back in. |
| **`Transform [ausrabot_odom] not available`** | Driver or micro-ROS agent disconnected | Verify ESP32 power and `/dev/ttyACM0`. Relaunch `./start_micro_ros_agent.sh`. |
| **Micro-ROS agent connection timeout** | ESP32 serial DTR reset race | Install `esptool` (`pip install esptool`) and use `./start_micro_ros_agent.sh`. |
| **Incorrect odometry distance** | Physical parameter mismatch | Verify `robot_radius` & `wheel_radius` in `ausrabot_description/config/hardware_params.yaml`. |
| **Missing topics on `ros2 topic list`** | CycloneDDS participant limit hit | Source `jetson_ros_env.sh` (`MaxAutoParticipantIndex=500`). |
| **Robot spins or drifts during translation** | Unbalanced PID gains or encoder polarity | Verify low-level ESP32 PID gains ($K_p=10, K_i=10, K_d=0.01$) and encoder directions. |
| **Nav2 fails to plan / goals rejected** | Frame transform mismatch | Check TF tree with `ros2 run tf2_tools view_frames`. Verify: `map` $\rightarrow$ `ausra_1_odom` $\rightarrow$ `ausra_1_robot_footprint`. |

---

## 12. License & Team

Developed by the **AUSRA** Team. Released under the MIT License.