-- ============================================================================
-- Cartographer Configuration for Omni-Wheel Robot in Gazebo Simulation
-- ============================================================================
-- Robot: Omni-wheel robot with RPLidar A1
-- Environment: Gazebo simulation
-- Driver: Custom omni-wheel driver (publishes /odom and odom->base_link TF)
-- ============================================================================

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  
  -- ========================================================================
  -- FRAME CONFIGURATION (Critical for omni-wheel robots!)
  -- ========================================================================
  map_frame = "map",
  tracking_frame = "base_link",      -- Frame at robot center (tracked by Cartographer)  //robot_footprint // was base_link
  published_frame = "odom",          -- Cartographer publishes: map -> odom //odom //robot_footprint at the old driver but the new one use odom
  odom_frame = "odom",               -- Odometry frame name
  
  -- CRITICAL: Set to FALSE because your omni-wheel driver publishes odom->base_link
  provide_odom_frame = false,  --it was false at first 
  
  -- ========================================================================
  -- GENERAL OPTIONS
  -- ========================================================================
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  
  -- ========================================================================
  -- SENSOR CONFIGURATION
  -- ========================================================================
  use_odometry = true,               -- Use your omni-wheel odometry! --it wa true //worked with false as lidar only
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,               -- Single 2D LiDAR
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  
  -- ========================================================================
  -- TIMING & PUBLISHING
  -- ========================================================================
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  publish_to_tf = true,              -- Enable TF publishing (map -> odom)
  publish_tracked_pose = false,      -- Optional: set true for debugging
  
  -- ========================================================================
  -- SAMPLING RATIOS
  -- ========================================================================
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- ============================================================================
-- MAP BUILDER CONFIGURATION
-- ============================================================================
MAP_BUILDER.use_trajectory_builder_2d = true

MAP_BUILDER.num_background_threads = 2 --added for the lag 


-- ============================================================================
-- 2D TRAJECTORY BUILDER CONFIGURATION
-- ============================================================================

-- Process every scan for real-time performance in Gazebo
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1  --can increas it to combine the laser scans into super one to make lines more accurate 

-- ========================================================================
-- LIDAR RANGE CONFIGURATION (RPLidar A1)
-- ========================================================================
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 6.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 6.5 --3 due to rplidar take inf for above range values

-- ========================================================================
-- IMU CONFIGURATION
-- ========================================================================
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- ========================================================================
-- SCAN MATCHING CONFIGURATION
-- ========================================================================
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- Real-time correlative scan matcher
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1 --0.1 --Increase to 0.15-0.2 if you have odometry drift issues
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Ceres scan matcher
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20 --was 20 decrease it to decrease compuation  --use 10 for lag 
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 1

-- ========================================================================
-- MOTION FILTER CONFIGURATION
-- ========================================================================
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)

-- ========================================================================
-- SUBMAP CONFIGURATION
-- ========================================================================
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05 --decrease less than 5cm decrease error

-- ============================================================================
-- POSE GRAPH OPTIMIZATION CONFIGURATION
-- ============================================================================
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 20  -- Optimize more frequently for smaller environments it was 35 
--for large enviroments 90 

-- ========================================================================
-- LOOP CLOSURE CONFIGURATION
-- ========================================================================
POSE_GRAPH.constraint_builder.min_score = 0.65  -- 0.55 large --0.50 for long hallways 
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.

-- Fast correlative scan matcher for loop closure
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

-- Ceres scan matcher for loop closure --double wight for less error
POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 1.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 1

-- ============================================================================
-- OPTIMIZATION WEIGHTS (High trust in omni-wheel odometry)
-- ============================================================================
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e1  --// 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e1      --// 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5

return options