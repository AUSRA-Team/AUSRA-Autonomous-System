// Copyright 2024 AUSRA Team
// Licensed under Apache-2.0

#include <chrono>
#include <memory>
#include <set>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "ausra_frontier_exploration/frontier_search.hpp"

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

namespace ausra_frontier_exploration
{

class ExplorationServer : public rclcpp::Node
{
public:
  ExplorationServer()
  : Node("exploration_server"),
    exploring_(false)
  {
    // Parameters
    declare_parameter("robot_radius", 0.13);
    declare_parameter("inflation_radius", 0.35);
    declare_parameter("min_frontier_size", 4);
    declare_parameter("safety_ratio", 0.98);
    declare_parameter("blacklist_timeout", 5.0);
    declare_parameter("coverage_threshold", 0.99);
    declare_parameter("map_topic", "map");
    declare_parameter("robot_base_frame", "base_link");
    declare_parameter("global_frame", "map");

    // Configure frontier search
    frontier_search_.configure(
      get_parameter("robot_radius").as_double(),
      get_parameter("inflation_radius").as_double(),
      get_parameter("min_frontier_size").as_int(),
      get_parameter("safety_ratio").as_double()
    );

    blacklist_timeout_ = get_parameter("blacklist_timeout").as_double();
    coverage_threshold_ = get_parameter("coverage_threshold").as_double();

    // TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribers
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      get_parameter("map_topic").as_string(), 10,
      std::bind(&ExplorationServer::mapCallback, this, std::placeholders::_1));

    // Publishers
    frontier_viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "exploration/frontiers", 10);

    // Nav2 action client
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Timer for exploration loop
    exploration_timer_ = create_wall_timer(
      500ms, std::bind(&ExplorationServer::explorationLoop, this));

    RCLCPP_INFO(get_logger(), "Exploration server initialized");
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    latest_map_ = msg;
    
    // Calculate coverage
    int total = msg->data.size();
    int known = 0;
    for (const auto & cell : msg->data) {
      if (cell != -1) {
        known++;
      }
    }
    current_coverage_ = static_cast<double>(known) / total;
  }

  void explorationLoop()
  {
    if (!latest_map_) {
      return;
    }

    // Check if exploration complete
    if (current_coverage_ >= coverage_threshold_) {
      RCLCPP_INFO(get_logger(), "Exploration complete! Coverage: %.1f%%", 
        current_coverage_ * 100.0);
      exploring_ = false;
      return;
    }

    // Skip if currently navigating
    if (exploring_) {
      return;
    }

    // Clear blacklist periodically
    auto now = get_clock()->now();
    if ((now - last_blacklist_clear_).seconds() > blacklist_timeout_) {
      blacklist_.clear();
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

    // Visualize
    publishFrontierMarkers(frontiers);

    if (frontiers.empty()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "No frontiers found");
      return;
    }

    // Select best frontier (with blacklist)
    const Frontier * target = frontier_search_.selectBestFrontier(frontiers, blacklist_);

    if (!target) {
      RCLCPP_WARN(get_logger(), "All frontiers blacklisted, clearing blacklist");
      blacklist_.clear();
      target = frontier_search_.selectBestFrontier(frontiers, blacklist_);
    }

    if (target) {
      navigateToFrontier(*target);
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
    if (!nav_client_->wait_for_action_server(1s)) {
      RCLCPP_WARN(get_logger(), "Nav2 action server not available");
      return;
    }

    exploring_ = true;
    current_frontier_ = frontier;

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = get_parameter("global_frame").as_string();
    goal_msg.pose.header.stamp = get_clock()->now();
    goal_msg.pose.pose.position = frontier.centroid;
    goal_msg.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(get_logger(), "Navigating to frontier at (%.2f, %.2f)",
      frontier.centroid.x, frontier.centroid.y);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = 
      std::bind(&ExplorationServer::navResultCallback, this, std::placeholders::_1);
    
    nav_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void navResultCallback(const GoalHandleNav::WrappedResult & result)
  {
    exploring_ = false;

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Reached frontier successfully");
        break;
      case rclcpp_action::ResultCode::ABORTED:
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(get_logger(), "Navigation failed, blacklisting frontier");
        blacklist_.insert({current_frontier_.centroid.x, current_frontier_.centroid.y});
        break;
      default:
        break;
    }
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
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = get_parameter("global_frame").as_string();
      marker.header.stamp = get_clock()->now();
      marker.ns = "frontiers";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position = f.centroid;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = marker.scale.y = marker.scale.z = 0.3;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 0.8;
      marker.lifetime = rclcpp::Duration(1s);
      markers.markers.push_back(marker);
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
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::TimerBase::SharedPtr exploration_timer_;

  bool exploring_;
  double current_coverage_{0.0};
  double coverage_threshold_;
  double blacklist_timeout_;
  std::set<std::pair<double, double>> blacklist_;
  rclcpp::Time last_blacklist_clear_;
  Frontier current_frontier_;
};

}  // namespace ausra_frontier_exploration

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ausra_frontier_exploration::ExplorationServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
