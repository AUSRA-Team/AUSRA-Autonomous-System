# ==============================================================================
# AUSRA Multi-Robot Navigation System
# ==============================================================================
#
# This package provides a complete multi-robot navigation system for the AUSRA
# omni-directional robot platform with proper namespace isolation.
#
# ==============================================================================
# QUICK START - 3 Terminal Workflow
# ==============================================================================
#
# Terminal 1: Launch Gazebo World
# --------------------------------
# ros2 launch ausra_simulation world.launch.py world:=room_1.sdf
#
# Terminal 2: Spawn Robot 1 with Full Navigation Stack
# ----------------------------------------------------
# ros2 launch ausra_spawner robot_bringup.launch.py robot_id:=1 x:=0.0 y:=0.0
#
# Terminal 3: Spawn Robot 2 with Full Navigation Stack
# ----------------------------------------------------
# ros2 launch ausra_spawner robot_bringup.launch.py robot_id:=2 x:=2.0 y:=0.0
#
# Terminal 4 (Optional): Launch RViz for Visualization
# ----------------------------------------------------
# ros2 launch ausra_spawner rviz_multirobot.launch.py
#
# ==============================================================================
# ARCHITECTURE OVERVIEW
# ==============================================================================
#
# Multi-Robot Namespace Isolation:
# --------------------------------
# Each robot operates in its own namespace (e.g., /ausra_1, /ausra_2) with:
#   - Isolated topics: /ausra_1/scan, /ausra_1/cmd_vel, /ausra_1/odom
#   - Isolated TF frames: ausra_1_odom, ausra_1_robot_footprint
#   - Isolated navigation stack: /ausra_1/navigate_to_pose action
#   - Shared map frame: All robots share the global 'map' frame
#
# TF Tree Structure:
# -----------------
#   map (global, shared)
#    ├── ausra_1_odom (robot 1's odometry frame)
#    │   └── ausra_1_robot_footprint (robot 1's base frame)
#    │       └── ausra_1_base_link
#    │           └── ... (robot 1's links)
#    └── ausra_2_odom (robot 2's odometry frame)
#        └── ausra_2_robot_footprint (robot 2's base frame)
#            └── ... (robot 2's links)
#
# ==============================================================================
# LAUNCH FILES
# ==============================================================================
#
# robot_bringup.launch.py (MAIN LAUNCH FILE)
# ------------------------------------------
# Spawns a single robot with the complete navigation stack.
#
# Arguments:
#   robot_id:        Unique integer ID (creates namespace ausra_<id>)
#   x, y, yaw:       Spawn position in world coordinates
#   use_slam:        Enable SLAM Toolbox (default: true)
#   use_nav2:        Enable Nav2 navigation stack (default: true)
#   use_exploration: Enable frontier exploration (default: true)
#
# Examples:
#   # Robot with full stack (SLAM + Nav2 + Exploration)
#   ros2 launch ausra_spawner robot_bringup.launch.py robot_id:=1 x:=0 y:=0
#
#   # Robot without exploration (just SLAM + Nav2)
#   ros2 launch ausra_spawner robot_bringup.launch.py robot_id:=1 x:=0 y:=0 use_exploration:=false
#
#   # Robot without SLAM (assumes map is provided)
#   ros2 launch ausra_spawner robot_bringup.launch.py robot_id:=1 x:=0 y:=0 use_slam:=false
#
#
# rviz_multirobot.launch.py
# -------------------------
# Launches RViz2 with multi-robot configuration showing both robots.
#
# ==============================================================================
# CONFIGURATION FILES
# ==============================================================================
#
# config/nav2_multirobot.yaml
# ---------------------------
# Nav2 parameters with <robot_namespace> placeholders for dynamic replacement.
# Includes:
#   - AMCL (for post-SLAM localization)
#   - BT Navigator
#   - Controller Server (DWB for omni-directional motion)
#   - Planner Server
#   - Behavior Server
#   - Local/Global Costmaps
#
# config/slam_multirobot.yaml
# ---------------------------
# SLAM Toolbox configuration for multi-robot mapping.
# Each robot runs its own SLAM instance but publishes to the shared /map topic.
#
# config/ekf_multirobot.yaml
# --------------------------
# Extended Kalman Filter configuration for sensor fusion.
# Fuses odometry data with optional IMU data.
#
# config/exploration_multirobot.yaml
# ----------------------------------
# Frontier exploration parameters including:
#   - Coverage threshold (95% default)
#   - Return-to-home behavior
#   - Frontier detection settings
#
# ==============================================================================
# EXPLORATION WORKFLOW
# ==============================================================================
#
# 1. Robot spawns and starts SLAM
# 2. Exploration server finds frontiers (unexplored areas)
# 3. Robot navigates to closest frontier using Nav2
# 4. Process repeats until coverage_threshold (95%) is reached
# 5. Robot returns to start position
# 6. Robot waits for external goals
#
# Status topics:
#   /ausra_1/exploration/status - Current exploration state
#   /ausra_1/exploration/frontiers - Frontier visualization markers
#
# ==============================================================================
# USEFUL COMMANDS
# ==============================================================================
#
# Check robot topics:
#   ros2 topic list | grep ausra_1
#
# Check TF tree:
#   ros2 run tf2_tools view_frames
#
# Send navigation goal to robot 1:
#   ros2 action send_goal /ausra_1/navigate_to_pose nav2_msgs/action/NavigateToPose \
#     "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"
#
# Check exploration status:
#   ros2 topic echo /ausra_1/exploration/status
#
# Save map:
#   ros2 run nav2_map_server map_saver_cli -f my_map --ros-args -p use_sim_time:=true
#
# ==============================================================================
# TROUBLESHOOTING
# ==============================================================================
#
# Problem: TF frames not found
# Solution: Check that robot_state_publisher is running and frame_prefix is correct
#   ros2 topic echo /tf --once | grep ausra_1
#
# Problem: Navigation not working
# Solution: Check lifecycle manager status
#   ros2 lifecycle list /ausra_1/lifecycle_manager_navigation
#
# Problem: Costmap not updating
# Solution: Verify scan topic is publishing
#   ros2 topic hz /ausra_1/scan
#
# Problem: Exploration stops prematurely
# Solution: Lower the coverage_threshold or check for blocked frontiers
#
# ==============================================================================
