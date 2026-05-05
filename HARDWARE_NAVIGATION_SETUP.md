# AUSRA Real Hardware Navigation Setup

This guide explains how to use the newly integrated Navigation packages with real AUSRA robot hardware.

## Branch Information

- **Branch**: `hardware_with_nav2`
- **Base**: Combines working SLAM from `hardware` branch with Nav2 packages from `latest` branch
- **Target Robot**: AUSRA omnidirectional robot with RPLIDAR A1 lidar

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    REAL HARDWARE AUSRA                      │
│  - 3-wheel omnidirectional robot                            │
│  - RPLIDAR A1 mounted on top (/dev/ttyUSB0, 115200 baud)   │
│  - Odometry from motor encoders                             │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                    ROS 2 Drivers Layer                      │
│  - omnidirectional_driver: Robot motion control            │
│  - sllidar_ros2: RPLIDAR A1 driver                         │
│  - robot_state_publisher: Transform publishing             │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                  SLAM & Localization                        │
│  - slam_toolbox (async mode): Creates map                  │
│  - AMCL (optional): Localization with existing map         │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                    Nav2 Stack                               │
│  - nav2_planner: Path planning                             │
│  - nav2_controller: Motion control                         │
│  - nav2_behavior_tree: Decision tree execution             │
│  - nav2_costmap_2d: Obstacle detection                    │
└─────────────────────────────────────────────────────────────┘
```

## Frame Names (Critical for Integration)

The following frame names must be consistent across all components:

| Frame | Purpose | Source |
|-------|---------|--------|
| `map` | Global fixed frame | SLAM/Map server |
| `ausrabot_odom` | Odometry frame | omnidirectional_driver |
| `ausrabot_robot_footprint` | Robot base frame | URDF root |
| `ausrabot_lidar` | LiDAR sensor | sensors.xacro |

**All configurations must use these exact names.**

## Key Packages

### 1. **lidar_slam_pkg** (Hardware-specific SLAM)
- **Location**: `/lidar_slam_pkg/`
- **Launch**: `lidar_slam_pkg/launch/slam.launch.py`
- **Purpose**: Real-time SLAM using RPLIDAR A1
- **Config**: `lidar_slam_pkg/config/slam_toolbox_config.yaml`
- **Features**:
  - Tuned slam_toolbox parameters for RPLIDAR A1 (max range 10m, resolution 0.05m)
  - Async mode (non-blocking mapping)
  - Loop closure enabled
  - CeresSolver backend

### 2. **Navigation/** (Nav2 Stack)
- **Location**: `/Navigation/`
- **Contains**: 17 ROS 2 packages for autonomous navigation
- **Key packages**:
  - `nav2_bringup`: Main launch files
  - `nav2_planner`: Path planning (uses SMAC Planner)
  - `nav2_controller`: Motion control
  - `nav2_costmap_2d`: Obstacle representation
  - `nav2_amcl`: Particle filter localization

### 3. **ausra_movement_demo** (Autonomous Movement)
- **Location**: `/ausra_movement_demo/`
- **Node**: `holonomic_movement_demo.py`
- **Purpose**: Autonomous distance-based movements for testing
- **Launch**: `ausra_movement_demo/launch/holonomic_demo.launch.py`
- **Features**:
  - Odometry-based distance control (0.05m tolerance)
  - Holonomic (omnidirectional) strafing in Phase 1
  - Differential-drive comparison in Phase 2
  - Real hardware optimized (use_sim_time: False)

## Launch Files

### Option 1: SLAM Only (Map Creation)
```bash
ros2 launch lidar_slam_pkg slam.launch.py
```
**Output**: 
- Creates `/map` topic and tf transformation
- Running slam_toolbox in async mode
- LiDAR data visualization in RViz

**Use case**: Manually Drive robot to create initial map

### Option 2: SLAM + Autonomous Movement Demo
```bash
ros2 launch lidar_slam_pkg slam.launch.py &
sleep 5  # Wait for SLAM to initialize
ros2 launch ausra_movement_demo holonomic_demo.launch.py movement_distance:=2.0
```
**Parameters**:
- `movement_distance`: Distance per movement (default 1.0m)
- `linear_velocity`: Forward/strafe speed (default 0.2 m/s)
- `angular_velocity`: Rotation speed (default 0.3 rad/s)
- `startup_delay_secs`: Startup stabilization time (default 1.0s for hardware)

### Option 3: Full Integrated Navigation (Hardware + SLAM + Nav2)
```bash
ros2 launch lidar_slam_pkg integrated_hardware_navigation.launch.py
```
**Features**:
- Automatically starts SLAM, Drivers, and Nav2.
- Uses custom parameters from `lidar_slam_pkg/config/nav2_params.yaml`.
- Includes pre-configured RViz.

## Configuration Files

### 1. **slam_toolbox_config.yaml** (SLAM Parameters)
`lidar_slam_pkg/config/slam_toolbox_config.yaml`

**Critical Parameters for Real Hardware**:
```yaml
# Frame mapping
odom_frame: ausrabot_odom
base_frame: ausrabot_robot_footprint
map_frame: map
scan_topic: /scan

# Hardware-specific tuning
map_limit_saturation: 0.05  # 50mm resolution for RPLIDAR A1
max_laser_range: 10.0       # RPLIDAR A1 max range
map_update_interval: 2.0    # Update frequency
mode: localization           # Set to mapping for first run

# Performance tuning
minimum_travel_distance: 0.1   # Scan matching threshold
minimum_travel_heading: 0.1    # Angular threshold for new scan
```

### 2. **hardware_params.yaml** (Robot Driver Parameters)
`ausrabot_description/config/hardware_params.yaml`

**Robot Configuration**:
```yaml
omnidirectional_driver:
  robot_radius: 0.124           # Meters
  wheel_radius: 0.0325          # Meters
  wheel_angles_deg: [270, 30, 150]  # 3-wheel angles
  odom_frame_id: ausrabot_odom        # MUST MATCH
  base_frame_id: ausrabot_robot_footprint  # MUST MATCH
  use_sim_time: false           # CRITICAL for real hardware
```

### 3. **nav2_params.yaml** (Navigation Parameters)
`lidar_slam_pkg/config/nav2_params.yaml`

**Already configured for AUSRA**:
- Robot model: `OmniMotionModel` (omnidirectional)
- Controller: DWB Local Planner tuned for omni-wheels.
- Costmap: Voxel and Obstacle layers for real-world sensing.

## Step-by-Step Real Hardware Testing

### Phase 1: Verify Hardware Integration
```bash
# Check LIDAR connection
ls -la /dev/ttyUSB0    # Should exist and be readable

# Check motors respond
ros2 input cmd_vel
# Publish: {"linear": {"x": 0.1, "y": 0, "z": 0}, "angular": {"z": 0}}
# Robot should move forward slowly
```

### Phase 2: Create Initial Map
```bash
# Terminal 1: Start SLAM
ros2 launch lidar_slam_pkg slam.launch.py

# Terminal 2: Publish initial pose (very important!)
ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped \
  "{header: {stamp: now, frame_id: map}, pose: {pose: {position: {x: 0, y: 0, z: 0}}}}"

# Terminal 3: Manual teleoperation (from ausra_numpad_teleop)
ros2 run ausra_numpad_teleop numpad_teleop

# Drive robot around room to create map (5-10 minutes)
# Check RViz for map quality
```
**Expected**: `/map` topic appearing, LiDAR scans aligning

### Phase 3: Test Autonomous Movement
```bash
# Start SLAM in existing map mode (if using previously saved map)
ros2 launch lidar_slam_pkg slam.launch.py

# Run autonomous movement demo
ros2 launch ausra_movement_demo holonomic_demo.launch.py movement_distance:=0.5

# Robot will execute:
# - Phase 1: Move forward 0.5m, backward 0.5m, strafe right 0.5m, left 0.5m
# - Phase 2: Same pattern using only forward + rotation (differential drive)
```
**Expected**: Clean odometry tracking, accurate distance measurements

### Phase 4: Full Navigation Stack
```bash
# Start full stack
ros2 launch lidar_slam_pkg slam.launch.py &
sleep 2
ros2 launch ausra_nav2_bringup bringup_launch.py

# In RViz:
# 1. Set initial pose with "2D Pose Estimate" button
# 2. Click "Navigation2" button  3. Set goal with "Nav2 Goal" button
# Robot should navigate autonomously!
```

## Troubleshooting

### Issue: "Transform [ausrabot_odom] is not available"
- **Cause**: omnidirectional_driver not running or crashed
- **Solution**: 
  ```bash
  ros2 node list | grep omni
  ros2 launch lidar_slam_pkg slam.launch.py  # Restarts everything
  ```

### Issue: Robot moves but doesn't match odometry
- **Cause**: Incorrect robot parameters (wheel radius, angles)
- **Solution**: Check `hardware_params.yaml` - these must match physical robot
  ```bash
  # Test with known distance movement
  ros2 input cmd_vel  # Publish 0.1 m/s for 5 seconds
  # Should move exactly 0.5m if calibrated correctly
  ```

### Issue: Map quality poor, LiDAR scans don't align
- **Cause**: slam_toolbox parameters, or LIDAR moved/damaged
- **Solution**:
  ```bash
  # Check LIDAR data quality
  ros2 topic echo /scan | head -20
  # Look for: ranges should have many values < 10m, some > 8m
  # Check for sudden gaps or jumps
  ```

### Issue: Nav2 won't accept goals or navigation fails
- **Cause**: Frame mismatch between nav2_config and actual system
- **Solution**: 
  ```bash
  # Check tf tree
  ros2 run tf2_tools view_frames
  # Verify: map → ausrabot_odom → ausrabot_robot_footprint → ausrabot_lidar
  
  # Check frame names in config
  grep -r "ausrabot" Navigation/nav2_bringup/params/
  ```

## Performance Tuning

### For Slow Mapping (Drifting)
Increase SLAM constraints in `slam_toolbox_config.yaml`:
```yaml
do_loop_closing: true          # Enable to fix drift
loop_match_minimum_chain_size: 10
minimum_travel_distance: 0.15  # Require more movement
```

### For Jerky Movement
Reduce velocity in `holonomic_demo.launch.py`:
```bash
ros2 launch ausra_movement_demo holonomic_demo.launch.py \
  linear_velocity:=0.1 angular_velocity:=0.2
```

### For Slow Navigation
Increase Nav2 velocity limits in `nav2_ausra_configuration.yaml`:
```yaml
# Increase from defaults (careful with motor heating)
max_vel_x: 0.3     # Up from 0.26
max_vel_y: 0.3     # Up from 0.26
max_vel_theta: 0.5 # Up from 0.44
```

## Important Notes

1. **Always verify frame names match** - This is the most common integration issue
2. **Hardware params must be exact** - Wrong wheel radius/angles causes odometry drift
3. **use_sim_time must be False** - Simulation time will break realtime systems
4. **LIDAR calibration critical** - Must be in `sensors.xacro` with exact transform
5. **Test incrementally** - Don't jump straight to autonomous navigation

## Related Branches

- **`latest`**: Full research stack with simulation
- **`hardware`**: Minimal working real-hardware setup (SLAM only)
- **`hardware_with_nav2`** ← **YOU ARE HERE**: Hardware + Navigation integration

## Next Steps

1. Test SLAM generation with manual driving
2. Validate autonomous movement accuracy
3. Run full navigation stack in open space
4. Tune parameters for your specific environment
5. Add obstacle avoidance behaviors
6. Consider adding frontier exploration for autonomous mapping

## Contact & Support

For issues or questions about real hardware integration, check:
- SLAM tuning: `/lidar_slam_pkg/config/slam_toolbox_config.yaml`
- Hardware params: `/ausrabot_description/config/hardware_params.yaml`
- Frame transforms: `/ausrabot_description/urdf/sensors.xacro`
