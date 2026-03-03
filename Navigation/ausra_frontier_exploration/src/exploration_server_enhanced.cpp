// Copyright 2024 AUSRA Team
// Licensed under Apache-2.0
//
// Enhanced Frontier Exploration Server with Return-to-Home functionality
// Designed for multi-robot operation with proper namespace isolation

#include <chrono>
#include <memory>
#include <set>
#include <string>
#include <cmath>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "slam_toolbox/srv/serialize_pose_graph.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "ausra_frontier_exploration/frontier_search.hpp"

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

namespace ausra_frontier_exploration
{

enum class ExplorationState
{
  IDLE,
  WAITING_FOR_MAP,
  EXPLORING,
  NAVIGATING_TO_FRONTIER,
  RETURNING_HOME,
  WAITING_AT_HOME,
  COMPLETED
};

class ExplorationServerEnhanced : public rclcpp::Node
{
public:
  ExplorationServerEnhanced()
  : Node("exploration_server"),
    state_(ExplorationState::WAITING_FOR_MAP),
    last_blacklist_clear_(0, 0, RCL_ROS_TIME),  // Initialize with ROS time source
    nav_start_time_(0, 0, RCL_ROS_TIME)         // Initialize nav timeout timer
  {
    // Parameters
    declare_parameter("robot_radius", 0.15);
    declare_parameter("inflation_radius", 0.35);
    declare_parameter("min_frontier_size", 4);
    declare_parameter("safety_ratio", 0.98);
    declare_parameter("blacklist_timeout", 30.0);
    declare_parameter("coverage_threshold", 0.95);
    declare_parameter("map_topic", "map");
    declare_parameter("robot_base_frame", "base_link");
    declare_parameter("global_frame", "map");
    declare_parameter("exploration_loop_rate", 2.0);
    declare_parameter("return_to_start_on_complete", true);
    declare_parameter("start_position_tolerance", 0.3);
    declare_parameter("visualize_frontiers", true);
    // Minimum frontier distance - must be greater than Nav2 goal tolerance
    declare_parameter("min_frontier_distance", 0.5);
    
    // Start position (set by launch file)
    declare_parameter("start_x", 0.0);
    declare_parameter("start_y", 0.0);
    declare_parameter("start_yaw", 0.0);
    
    // Map saving parameters
    declare_parameter("save_map_on_complete", true);
    declare_parameter("map_save_path", "/tmp/exploration_map");

    // Start position will be captured from TF when exploration begins
    // This ensures we use map-frame coordinates, not world coordinates
    start_x_ = 0.0;
    start_y_ = 0.0;
    start_yaw_ = 0.0;
    start_position_captured_ = false;
    min_frontier_distance_ = get_parameter("min_frontier_distance").as_double();
    save_map_on_complete_ = get_parameter("save_map_on_complete").as_bool();
    map_save_path_ = get_parameter("map_save_path").as_string();
    
    // Configure frontier search
    frontier_search_.configure(
      get_parameter("robot_radius").as_double(),
      get_parameter("inflation_radius").as_double(),
      get_parameter("min_frontier_size").as_int(),
      get_parameter("safety_ratio").as_double()
    );

    blacklist_timeout_ = get_parameter("blacklist_timeout").as_double();
    coverage_threshold_ = get_parameter("coverage_threshold").as_double();
    return_to_start_ = get_parameter("return_to_start_on_complete").as_bool();
    start_tolerance_ = get_parameter("start_position_tolerance").as_double();
    visualize_ = get_parameter("visualize_frontiers").as_bool();

    // TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribers
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      get_parameter("map_topic").as_string(), 10,
      std::bind(&ExplorationServerEnhanced::mapCallback, this, std::placeholders::_1));

    // Publishers
    frontier_viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "exploration/frontiers", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "exploration/status", 10);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", 10);

    // Nav2 action client
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Timer for exploration loop
    double loop_rate = get_parameter("exploration_loop_rate").as_double();
    exploration_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / loop_rate)),
      std::bind(&ExplorationServerEnhanced::explorationLoop, this));

    // Service client for map saving (slam_toolbox)
    save_map_client_ = create_client<slam_toolbox::srv::SerializePoseGraph>("slam_toolbox/serialize_map");

    RCLCPP_INFO(get_logger(), "Exploration server initialized");
    RCLCPP_INFO(get_logger(), "Start position: (%.2f, %.2f, %.2f)", start_x_, start_y_, start_yaw_);
    RCLCPP_INFO(get_logger(), "Coverage threshold: %.1f%%", coverage_threshold_ * 100.0);
    RCLCPP_INFO(get_logger(), "Map saving: %s (path: %s)", 
      save_map_on_complete_ ? "enabled" : "disabled", map_save_path_.c_str());
    RCLCPP_INFO(get_logger(), "Waiting for map and TF transforms before starting exploration...");
    
    publishStatus("WAITING_FOR_MAP");
    // State is already WAITING_FOR_MAP from constructor
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    latest_map_ = msg;
    
    // Calculate coverage
    int total = msg->data.size();
    int known = 0;
    int free = 0;
    int occupied = 0;
    
    for (const auto & cell : msg->data) {
      if (cell != -1) {
        known++;
        if (cell == 0) free++;
        else if (cell >= 50) occupied++;
      }
    }
    
    current_coverage_ = static_cast<double>(known) / total;
    
    // Log coverage periodically using message count instead of time
    // (avoids time source mismatch between system and sim time)
    static int log_counter = 0;
    log_counter++;
    if (log_counter >= 10) {  // Log every 10th map message
      RCLCPP_INFO(get_logger(), "Map coverage: %.1f%% (known: %d, free: %d, obstacles: %d)",
        current_coverage_ * 100.0, known, free, occupied);
      log_counter = 0;
    }
  }

  void explorationLoop()
  {
    switch (state_) {
      case ExplorationState::IDLE:
        // Do nothing
        break;
      
      case ExplorationState::WAITING_FOR_MAP:
        handleWaitingForMap();
        break;
        
      case ExplorationState::EXPLORING:
        handleExploring();
        break;
        
      case ExplorationState::NAVIGATING_TO_FRONTIER:
        // Check for navigation timeout
        handleNavigationTimeout();
        break;
        
      case ExplorationState::RETURNING_HOME:
        // Waiting for navigation to complete
        break;
        
      case ExplorationState::WAITING_AT_HOME:
        handleWaitingAtHome();
        break;
        
      case ExplorationState::COMPLETED:
        // Done, just idle
        break;
    }
  }
  
  void handleWaitingForMap()
  {
    // Wait for map to be received
    if (!latest_map_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "Waiting for map from SLAM...");
      return;
    }
    
    // Wait for TF transform to be available
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!getRobotPose(robot_pose)) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "Waiting for TF transform (map -> %s)...",
        get_parameter("robot_base_frame").as_string().c_str());
      return;
    }
    
    // Wait for Nav2 action server to be available
    if (!nav_client_->wait_for_action_server(1s)) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "Waiting for Nav2 action server...");
      return;
    }
    
    // Capture start position from TF (in map frame) for return-to-home
    // This is the robot's actual position in the SLAM map, should be ~(0,0)
    if (!start_position_captured_) {
      start_x_ = robot_pose.pose.position.x;
      start_y_ = robot_pose.pose.position.y;
      // Extract yaw from quaternion
      tf2::Quaternion q(
        robot_pose.pose.orientation.x,
        robot_pose.pose.orientation.y,
        robot_pose.pose.orientation.z,
        robot_pose.pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      start_yaw_ = yaw;
      start_position_captured_ = true;
      RCLCPP_INFO(get_logger(), "Captured start position in MAP frame: (%.2f, %.2f, yaw=%.2f)",
        start_x_, start_y_, start_yaw_);
    }
    
    // All prerequisites met - start exploring!
    RCLCPP_INFO(get_logger(), "==== STARTING EXPLORATION ====");
    RCLCPP_INFO(get_logger(), "Map received, TF available, Nav2 ready");
    RCLCPP_INFO(get_logger(), "Home position (map frame): (%.2f, %.2f)",
      start_x_, start_y_);
    
    publishStatus("EXPLORING");
    state_ = ExplorationState::EXPLORING;
  }

  void handleExploring()
  {
    // Ensure we still have a valid map
    if (!latest_map_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Lost map, waiting...");
      return;
    }
    
    // Check if exploration complete
    if (current_coverage_ >= coverage_threshold_) {
      RCLCPP_INFO(get_logger(), "==== EXPLORATION COMPLETE ====");
      RCLCPP_INFO(get_logger(), "Coverage achieved: %.1f%% (threshold: %.1f%%)",
        current_coverage_ * 100.0, coverage_threshold_ * 100.0);
      
      // Save map before returning home
      if (save_map_on_complete_) {
        saveMap();
      }
      
      publishStatus("EXPLORATION_COMPLETE");
      
      if (return_to_start_) {
        RCLCPP_INFO(get_logger(), "Returning to start position...");
        navigateToPosition(start_x_, start_y_, start_yaw_, true);
        state_ = ExplorationState::RETURNING_HOME;
      } else {
        state_ = ExplorationState::COMPLETED;
        stopRobot();
      }
      return;
    }

    // Clear blacklist periodically
    auto now = get_clock()->now();
    if ((now - last_blacklist_clear_).seconds() > blacklist_timeout_) {
      if (!blacklist_.empty()) {
        RCLCPP_INFO(get_logger(), "Clearing %zu blacklisted frontiers", blacklist_.size());
        blacklist_.clear();
      }
      last_blacklist_clear_ = now;
    }

    // Get robot pose
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!getRobotPose(robot_pose)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Could not get robot pose");
      return;
    }

    // Find frontiers
    auto frontiers = frontier_search_.searchFromMap(
      *latest_map_, robot_pose.pose.position.x, robot_pose.pose.position.y);
    
    // Log frontier discovery details
    if (!frontiers.empty()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
        "Found %zu frontiers. Closest at (%.2f, %.2f), dist=%.2fm. Farthest dist=%.2fm",
        frontiers.size(),
        frontiers.front().centroid.x, frontiers.front().centroid.y,
        frontiers.front().min_distance,
        frontiers.back().min_distance);
    }

    // Visualize
    if (visualize_) {
      publishFrontierMarkers(frontiers);
    }

    if (frontiers.empty()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "No frontiers found. Coverage: %.1f%%", current_coverage_ * 100.0);
      
      // If no frontiers but below threshold, might be stuck
      if (current_coverage_ < coverage_threshold_) {
        // Try clearing blacklist to find more frontiers
        if (!blacklist_.empty()) {
          blacklist_.clear();
          RCLCPP_INFO(get_logger(), "Cleared blacklist to find more frontiers");
        }
      }
      return;
    }

    // Select best frontier (with blacklist and minimum distance filter)
    // IMPORTANT: During initial exploration (coverage < 5%), accept any frontier
    // to bootstrap map expansion. Otherwise use min_frontier_distance filter.
    double effective_min_dist = min_frontier_distance_;
    if (current_coverage_ < 0.05) {
      // Initial exploration phase - accept frontiers as close as 0.1m
      effective_min_dist = 0.1;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "Initial exploration phase - using reduced min frontier distance (0.1m)");
    }
    
    const Frontier * target = frontier_search_.selectBestFrontier(
      frontiers, blacklist_, effective_min_dist);

    if (!target) {
      RCLCPP_WARN(get_logger(), "All %zu frontiers blacklisted or too close (min dist: %.2f), clearing blacklist", 
        frontiers.size(), effective_min_dist);
      blacklist_.clear();
      target = frontier_search_.selectBestFrontier(frontiers, blacklist_, effective_min_dist);
    }
    
    // Last resort: if still no target and we have frontiers, just pick the first one
    if (!target && !frontiers.empty() && current_coverage_ < 0.10) {
      RCLCPP_WARN(get_logger(), "Bootstrap mode: selecting closest frontier regardless of distance");
      target = &frontiers.front();
    }

    if (target) {
      // Check if this is a useless micro-goal (too close to current position)
      geometry_msgs::msg::PoseStamped robot_pose;
      if (getRobotPose(robot_pose)) {
        double dx = target->centroid.x - robot_pose.pose.position.x;
        double dy = target->centroid.y - robot_pose.pose.position.y;
        double dist = std::sqrt(dx*dx + dy*dy);
        
        // If frontier is within goal tolerance, it's useless - we need to move further
        if (dist < 0.25) {  // xy_goal_tolerance is 0.20
          same_frontier_count_++;
          RCLCPP_WARN(get_logger(), "Frontier too close (%.2fm), count=%d", dist, same_frontier_count_);
          
          if (same_frontier_count_ >= 3) {
            // Force exploration in a direction to expand the map
            double angle = static_cast<double>(rand()) / RAND_MAX * 2.0 * M_PI;
            double force_dist = 1.0;  // Move 1 meter to expand map
            double force_x = robot_pose.pose.position.x + force_dist * std::cos(angle);
            double force_y = robot_pose.pose.position.y + force_dist * std::sin(angle);
            
            RCLCPP_INFO(get_logger(), "Forcing exploration movement to (%.2f, %.2f) to expand map", 
              force_x, force_y);
            navigateToPosition(force_x, force_y, 0.0, false);
            state_ = ExplorationState::NAVIGATING_TO_FRONTIER;
            same_frontier_count_ = 0;
            return;
          }
          // Skip this frontier and wait for map to update
          return;
        }
        same_frontier_count_ = 0;
      }
      
      navigateToFrontier(*target);
      state_ = ExplorationState::NAVIGATING_TO_FRONTIER;
    } else {
      // All frontiers are too close - this means the map is very small
      // Log this to help debug
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No valid frontiers found (all too close). Waiting for map to expand...");
    }
  }

  void handleWaitingAtHome()
  {
    // Verify robot is actually at home position
    geometry_msgs::msg::PoseStamped robot_pose;
    if (getRobotPose(robot_pose)) {
      double dx = robot_pose.pose.position.x - start_x_;
      double dy = robot_pose.pose.position.y - start_y_;
      double dist_to_home = std::sqrt(dx * dx + dy * dy);
      
      if (dist_to_home > start_tolerance_) {
        // Not yet at home, re-navigate
        RCLCPP_WARN(get_logger(), "Robot not at home (%.2fm away), re-navigating to start", dist_to_home);
        navigateToPosition(start_x_, start_y_, start_yaw_, true);
        state_ = ExplorationState::RETURNING_HOME;
        return;
      }
    }
    
    // Robot is at home position - STOP and complete
    publishStatus("COMPLETED");
    stopRobot();
    
    // Transition to COMPLETED state (only log once)
    if (state_ != ExplorationState::COMPLETED) {
      RCLCPP_INFO(get_logger(), "==== EXPLORATION FULLY COMPLETE ====");
      RCLCPP_INFO(get_logger(), "Robot at home position (%.2f, %.2f). Stopping.", start_x_, start_y_);
      state_ = ExplorationState::COMPLETED;
    }
  }

  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose)
  {
    try {
      auto transform = tf_buffer_->lookupTransform(
        get_parameter("global_frame").as_string(),
        get_parameter("robot_base_frame").as_string(),
        tf2::TimePointZero);

      pose.header = transform.header;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
      return true;
    } catch (const tf2::TransformException & ex) {
      return false;
    }
  }

  void navigateToFrontier(const Frontier & frontier)
  {
    current_frontier_ = frontier;
    
    // Get robot pose to compute safety offset
    geometry_msgs::msg::PoseStamped robot_pose;
    double target_x = frontier.centroid.x;
    double target_y = frontier.centroid.y;
    
    if (getRobotPose(robot_pose)) {
      // Pull the target point slightly towards the robot to avoid navigating
      // directly onto obstacle boundaries. This creates a safety buffer.
      double dx = frontier.centroid.x - robot_pose.pose.position.x;
      double dy = frontier.centroid.y - robot_pose.pose.position.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      
      if (dist > 0.5) {  // Only apply offset if frontier is far enough
        // Pull back 0.3m towards the robot (safety_offset)
        double safety_offset = 0.3;
        double ratio = (dist - safety_offset) / dist;
        target_x = robot_pose.pose.position.x + dx * ratio;
        target_y = robot_pose.pose.position.y + dy * ratio;
        
        RCLCPP_DEBUG(get_logger(), "Applied safety offset: (%.2f, %.2f) -> (%.2f, %.2f)",
          frontier.centroid.x, frontier.centroid.y, target_x, target_y);
      }
    }
    
    navigateToPosition(target_x, target_y, 0.0, false);
  }

  void handleNavigationTimeout()
  {
    // Check if navigation has been running too long
    auto elapsed = (get_clock()->now() - nav_start_time_).seconds();
    
    if (elapsed > 60.0) {  // 60 second timeout for navigation
      RCLCPP_WARN(get_logger(), "Navigation timeout after %.1f seconds, canceling and retrying", elapsed);
      nav_client_->async_cancel_all_goals();
      blacklist_.insert({current_frontier_.centroid.x, current_frontier_.centroid.y});
      state_ = ExplorationState::EXPLORING;
    } else if (elapsed > 5.0 && !goal_accepted_) {
      // Goal wasn't accepted within 5 seconds - Nav2 probably wasn't ready
      RCLCPP_WARN(get_logger(), "Goal not accepted after %.1f seconds, Nav2 may not be ready. Retrying...", elapsed);
      state_ = ExplorationState::EXPLORING;
    }
  }

  void navigateToPosition(double x, double y, double yaw, bool is_return_home)
  {
    // Wait longer for Nav2 to be fully ready
    if (!nav_client_->wait_for_action_server(5s)) {
      RCLCPP_WARN(get_logger(), "Nav2 action server not available after 5s, retrying later");
      state_ = ExplorationState::EXPLORING;  // Retry
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = get_parameter("global_frame").as_string();
    goal_msg.pose.header.stamp = get_clock()->now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.position.z = 0.0;
    
    // Convert yaw to quaternion
    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
    goal_msg.pose.pose.orientation.w = std::cos(yaw / 2.0);

    if (is_return_home) {
      RCLCPP_INFO(get_logger(), "Navigating HOME to (%.2f, %.2f)", x, y);
      publishStatus("RETURNING_HOME");
    } else {
      RCLCPP_INFO(get_logger(), "Navigating to frontier at (%.2f, %.2f)", x, y);
      publishStatus("NAVIGATING_TO_FRONTIER");
    }

    // Track navigation start time for timeout
    nav_start_time_ = get_clock()->now();
    goal_accepted_ = false;
    is_returning_home_ = is_return_home;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    
    // Goal response callback - fires when goal is accepted or rejected
    send_goal_options.goal_response_callback =
      [this](const GoalHandleNav::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Navigation goal was REJECTED by Nav2!");
          state_ = ExplorationState::EXPLORING;
        } else {
          RCLCPP_INFO(get_logger(), "Navigation goal ACCEPTED by Nav2");
          goal_accepted_ = true;
        }
      };
    
    if (is_return_home) {
      send_goal_options.result_callback =
        std::bind(&ExplorationServerEnhanced::returnHomeResultCallback, this, std::placeholders::_1);
    } else {
      send_goal_options.result_callback =
        std::bind(&ExplorationServerEnhanced::navResultCallback, this, std::placeholders::_1);
    }

    nav_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void navResultCallback(const GoalHandleNav::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Reached frontier successfully");
        nav_failure_count_ = 0;  // Reset failure counter
        break;
      case rclcpp_action::ResultCode::ABORTED:
      case rclcpp_action::ResultCode::CANCELED:
        nav_failure_count_++;
        RCLCPP_WARN(get_logger(), "Navigation to frontier (%.2f, %.2f) failed (%d times), blacklisting",
          current_frontier_.centroid.x, current_frontier_.centroid.y, nav_failure_count_);
        blacklist_.insert({current_frontier_.centroid.x, current_frontier_.centroid.y});
        
        // If too many failures in a row, try recovery movement
        if (nav_failure_count_ >= 2) {
          RCLCPP_WARN(get_logger(), "Multiple navigation failures, attempting recovery movement");
          
          // Move robot away from last frontier (back towards center/start)
          geometry_msgs::msg::PoseStamped robot_pose;
          if (getRobotPose(robot_pose)) {
            // Compute direction towards start position (safe area)
            double dx = start_x_ - robot_pose.pose.position.x;
            double dy = start_y_ - robot_pose.pose.position.y;
            double dist = std::sqrt(dx * dx + dy * dy);
            
            if (dist > 0.3) {
              // Move 0.5m towards start
              double recovery_dist = 0.5;
              double ratio = recovery_dist / dist;
              double recovery_x = robot_pose.pose.position.x + dx * ratio;
              double recovery_y = robot_pose.pose.position.y + dy * ratio;
              
              RCLCPP_INFO(get_logger(), "Recovery: moving 0.5m towards start (%.2f, %.2f)", 
                recovery_x, recovery_y);
              
              // Use cmd_vel to back up slightly first
              geometry_msgs::msg::Twist cmd;
              cmd.linear.x = -0.1;  // Gentle reverse
              cmd.linear.y = 0.0;
              cmd.angular.z = 0.0;
              cmd_vel_pub_->publish(cmd);
            }
          }
          
          blacklist_.clear();
          nav_failure_count_ = 0;
        }
        break;
      default:
        break;
    }
    
    // Continue exploring
    state_ = ExplorationState::EXPLORING;
  }

  void returnHomeResultCallback(const GoalHandleNav::WrappedResult & result)
  {
    // Get current position to verify
    geometry_msgs::msg::PoseStamped robot_pose;
    double dist_to_home = 999.0;
    if (getRobotPose(robot_pose)) {
      double dx = robot_pose.pose.position.x - start_x_;
      double dy = robot_pose.pose.position.y - start_y_;
      dist_to_home = std::sqrt(dx * dx + dy * dy);
    }
    
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "==== RETURNED HOME SUCCESSFULLY ====");
        RCLCPP_INFO(get_logger(), "Distance to exact start: %.3fm", dist_to_home);
        
        // If we're close enough, complete. Otherwise, do a precise navigation
        if (dist_to_home <= start_tolerance_) {
          state_ = ExplorationState::WAITING_AT_HOME;
        } else {
          RCLCPP_INFO(get_logger(), "Fine-tuning position to reach exact start...");
          navigateToPosition(start_x_, start_y_, start_yaw_, true);
        }
        break;
      case rclcpp_action::ResultCode::ABORTED:
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(get_logger(), "Failed to return home (dist=%.2fm), retrying...", dist_to_home);
        // If we're close enough despite failure, accept it
        if (dist_to_home <= start_tolerance_ * 2.0) {
          RCLCPP_INFO(get_logger(), "Close enough to home, accepting position");
          state_ = ExplorationState::WAITING_AT_HOME;
        } else {
          // Retry return home
          navigateToPosition(start_x_, start_y_, start_yaw_, true);
        }
        break;
      default:
        state_ = ExplorationState::WAITING_AT_HOME;
        break;
    }
  }

  void saveMap()
  {
    RCLCPP_INFO(get_logger(), "Saving map to: %s", map_save_path_.c_str());
    
    if (!save_map_client_->wait_for_service(5s)) {
      RCLCPP_WARN(get_logger(), "slam_toolbox/serialize_map service not available, cannot save map");
      return;
    }
    
    auto request = std::make_shared<slam_toolbox::srv::SerializePoseGraph::Request>();
    request->filename = map_save_path_;
    
    auto future = save_map_client_->async_send_request(request);
    
    // Wait for result with timeout
    auto status = future.wait_for(10s);
    if (status == std::future_status::ready) {
      auto result = future.get();
      if (result->result == 0) {
        RCLCPP_INFO(get_logger(), "==== MAP SAVED SUCCESSFULLY ====");
        RCLCPP_INFO(get_logger(), "Map saved to: %s.pgm and %s.yaml", 
          map_save_path_.c_str(), map_save_path_.c_str());
        RCLCPP_INFO(get_logger(), "Pose graph saved to: %s.posegraph", map_save_path_.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "Failed to save map (error code: %d)", result->result);
      }
    } else {
      RCLCPP_WARN(get_logger(), "Map save service call timed out");
    }
  }

  void stopRobot()
  {
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.linear.y = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_cmd);
  }

  void publishStatus(const std::string & status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
  }

  void publishFrontierMarkers(const std::vector<Frontier> & frontiers)
  {
    visualization_msgs::msg::MarkerArray markers;

    // Clear previous
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    int id = 0;
    for (const auto & f : frontiers) {
      // Skip frontiers with invalid coordinates
      if (std::isnan(f.centroid.x) || std::isnan(f.centroid.y) ||
          std::isinf(f.centroid.x) || std::isinf(f.centroid.y)) {
        continue;
      }
      
      // Skip frontiers too far from robot (likely invalid)
      if (f.min_distance > 15.0) {
        continue;
      }
      
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = get_parameter("global_frame").as_string();
      marker.header.stamp = get_clock()->now();
      marker.ns = "frontiers";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position = f.centroid;
      marker.pose.orientation.w = 1.0;
      
      // Size based on frontier size
      double scale = std::min(0.5, std::max(0.2, f.size / 10.0));
      marker.scale.x = marker.scale.y = marker.scale.z = scale;
      
      // Color: green for closest, yellow for others
      if (id == 1) {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
      } else {
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
      }
      marker.color.a = 0.8;
      marker.lifetime = rclcpp::Duration(1s);
      markers.markers.push_back(marker);
    }

    // Add home marker only if within reasonable bounds (map is centered around robot start)
    // Only show home marker if we have valid coordinates
    if (!std::isnan(start_x_) && !std::isnan(start_y_) &&
        std::abs(start_x_) < 100.0 && std::abs(start_y_) < 100.0) {
      visualization_msgs::msg::Marker home_marker;
      home_marker.header.frame_id = get_parameter("global_frame").as_string();
      home_marker.header.stamp = get_clock()->now();
      home_marker.ns = "home";
      home_marker.id = 0;
      home_marker.type = visualization_msgs::msg::Marker::CYLINDER;
      home_marker.action = visualization_msgs::msg::Marker::ADD;
      home_marker.pose.position.x = start_x_;
      home_marker.pose.position.y = start_y_;
      home_marker.pose.position.z = 0.1;
      home_marker.pose.orientation.w = 1.0;
      home_marker.scale.x = 0.3;
      home_marker.scale.y = 0.3;
      home_marker.scale.z = 0.2;
      home_marker.color.r = 0.0;
      home_marker.color.g = 0.0;
      home_marker.color.b = 1.0;
      home_marker.color.a = 0.8;
      markers.markers.push_back(home_marker);
    }

    frontier_viz_pub_->publish(markers);
  }

  // Members
  FrontierSearch frontier_search_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_viz_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::TimerBase::SharedPtr exploration_timer_;
  rclcpp::Client<slam_toolbox::srv::SerializePoseGraph>::SharedPtr save_map_client_;

  ExplorationState state_;
  double current_coverage_{0.0};
  double coverage_threshold_;
  double blacklist_timeout_;
  bool return_to_start_;
  double start_tolerance_;
  bool visualize_;
  double min_frontier_distance_;
  int nav_failure_count_{0};
  bool save_map_on_complete_;
  std::string map_save_path_;
  
  // Navigation timeout tracking
  rclcpp::Time nav_start_time_;
  bool goal_accepted_{false};
  bool is_returning_home_{false};
  int same_frontier_count_{0};  // Count of consecutive too-close frontiers
  
  // Start position for return-to-home (captured from TF in map frame)
  double start_x_;
  double start_y_;
  double start_yaw_;
  bool start_position_captured_;
  
  std::set<std::pair<double, double>> blacklist_;
  rclcpp::Time last_blacklist_clear_;
  Frontier current_frontier_;
};

}  // namespace ausra_frontier_exploration

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ausra_frontier_exploration::ExplorationServerEnhanced>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
