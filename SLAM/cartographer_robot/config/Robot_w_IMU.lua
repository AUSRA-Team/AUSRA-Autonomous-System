-- --- version test ---
-- -- ============================================================================
-- -- Cartographer Configuration for Omni-Wheel Robot in Gazebo Simulation
-- -- ============================================================================
-- -- Robot: Omni-wheel robot with RPLidar A1
-- -- Environment: Gazebo simulation
-- -- Driver: Custom omni-wheel driver (publishes /odom and odom->base_link TF)
-- -- ============================================================================

-- include "map_builder.lua"
-- include "trajectory_builder.lua"

-- options = {
--   map_builder = MAP_BUILDER,
--   trajectory_builder = TRAJECTORY_BUILDER,
  
--   -- ========================================================================
--   -- FRAME CONFIGURATION (Critical for omni-wheel robots!)
--   -- ========================================================================
--   map_frame = "map",
--   tracking_frame = "robot_footprint",      -- Frame at robot center (tracked by Cartographer)  //robot_footprint // was base_link
--   published_frame = "odom",          -- Cartographer publishes: map -> odom //odom //robot_footprint at the old driver but the new one use odom
--   odom_frame = "odom",               -- Odometry frame name
  
--   -- CRITICAL: Set to FALSE because your omni-wheel driver publishes odom->base_link
--   provide_odom_frame = false,  --it was false at first 
  
--   -- ========================================================================
--   -- GENERAL OPTIONS
--   -- ========================================================================
--   publish_frame_projected_to_2d = false,
--   use_pose_extrapolator = true,
  
--   -- ========================================================================
--   -- SENSOR CONFIGURATION
--   -- ========================================================================
--   use_odometry = true,               -- Use your omni-wheel odometry! --it wa true //worked with false as lidar only
--   use_nav_sat = false,
--   use_landmarks = false,
--   num_laser_scans = 1,               -- Single 2D LiDAR
--   num_multi_echo_laser_scans = 0,
--   num_subdivisions_per_laser_scan = 1,
--   num_point_clouds = 0,
  
--   -- ========================================================================
--   -- TIMING & PUBLISHING
--   -- ========================================================================
--   lookup_transform_timeout_sec = 0.2,
--   submap_publish_period_sec = 0.3,
--   pose_publish_period_sec = 5e-3,
--   trajectory_publish_period_sec = 30e-3,
--   publish_to_tf = true,              -- Enable TF publishing (map -> odom)
--   publish_tracked_pose = false,      -- Optional: set true for debugging
  
--   -- ========================================================================
--   -- SAMPLING RATIOS
--   -- ========================================================================
--   rangefinder_sampling_ratio = 1.,
--   odometry_sampling_ratio = 1.,
--   fixed_frame_pose_sampling_ratio = 1.,
--   imu_sampling_ratio = 1.,
--   landmarks_sampling_ratio = 1.,
-- }

-- -- ============================================================================
-- -- MAP BUILDER CONFIGURATION
-- -- ============================================================================
-- MAP_BUILDER.use_trajectory_builder_2d = true

-- MAP_BUILDER.num_background_threads = 2 --added for the lag 


-- -- ============================================================================
-- -- 2D TRAJECTORY BUILDER CONFIGURATION
-- -- ============================================================================

-- -- Process every scan for real-time performance in Gazebo
-- TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1  -- increase it to combine the laser scans into super one

-- -- ========================================================================
-- -- LIDAR RANGE CONFIGURATION (RPLidar A1)
-- -- ========================================================================
-- TRAJECTORY_BUILDER_2D.min_range = 0.15
-- TRAJECTORY_BUILDER_2D.max_range = 6.0
-- TRAJECTORY_BUILDER_2D.missing_data_ray_length = 6.5 --3 due to rplidar take inf for above range values

-- -- ========================================================================
-- -- IMU CONFIGURATION
-- -- ========================================================================
-- TRAJECTORY_BUILDER_2D.use_imu_data = false

-- -- ========================================================================
-- -- SCAN MATCHING CONFIGURATION
-- -- ========================================================================
-- TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- -- Real-time correlative scan matcher
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1 --0.1 --Increase to 0.15-0.2 if you have odometry drift issues trust lidar wall 
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(40.)  -- good at 40 #test make it 40 to wide the search og 20 increase to catch slip rotation
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.  -- decrease to add more penalty  og was10
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- -- Ceres scan matcher
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20. --  good at 20 test  to enable lidar trust more
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.  --can decrease to trust lidar more  increase to 
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.    -- test trust lidar more in rotation orginal was 40. make it 4e2

-- --used if there is lag can decrease it -- 
-- -- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
-- -- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20 
-- -- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 1



-- -- Motion Filter Configuration

-- TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.
-- TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1  --## decrease to be more sensitive
-- TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.) 

-- -- Submap configuration

-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35  --  sub map takes 35 laser scan and mesh together if there slip in these it will cause error decrease
-- TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05 --decrease less than 5cm decrease error



-- -- POSE GRAPH OPTIMIZATION CONFIGURATION

-- POSE_GRAPH.optimization_problem.huber_scale = 1e2 --Lowering this (e.g., 1e1) tells the optimizer: "Ignore the noise/outliers. Only fit the perfect data." This can sometimes sharpen maps by ignoring bad sensor readings
-- POSE_GRAPH.optimize_every_n_nodes = 20  -- Optimize og 20 more frequently for smaller environments it was 35  glopbal map correction decrease correct more 
-- --for large enviroments 90 

-- -- -- ========================================================================
-- -- -- LOOP CLOSURE CONFIGURATION
-- -- -- ========================================================================
-- -- POSE_GRAPH.constraint_builder.min_score = 0.65  -- 0.55 large --0.50 for long hallways 
-- -- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
-- -- POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
-- -- POSE_GRAPH.constraint_builder.max_constraint_distance = 15.

-- -- -- Fast correlative scan matcher for loop closure
-- -- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.
-- -- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
-- -- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

-- -- -- Ceres scan matcher for loop closure --double wight for less error
-- -- POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20. --## test Was 20. -> Change to 10.
-- -- POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10.
-- -- POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 1.
-- -- POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
-- -- POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
-- -- POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 1

-- -- OPTIMIZATION WEIGHTS use when making ekf 

-- -- POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5  
-- -- POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5      
-- -- POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e6
-- -- POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e6

-- return options