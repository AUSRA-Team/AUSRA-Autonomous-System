// Copyright 2024 AUSRA Team
// Licensed under Apache-2.0

#ifndef AUSRA_FRONTIER_EXPLORATION__FRONTIER_SEARCH_HPP_
#define AUSRA_FRONTIER_EXPLORATION__FRONTIER_SEARCH_HPP_

#include <vector>
#include <set>
#include <queue>
#include <cmath>
#include <algorithm>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace ausra_frontier_exploration
{

struct Frontier
{
  geometry_msgs::msg::Point centroid;
  std::vector<std::pair<int, int>> cells;
  double size;
  double min_distance;
  double cost;  // Cost = potential_scale * distance - gain_scale * size (lower is better)
};

class FrontierSearch
{
public:
  FrontierSearch() = default;

  /**
   * @brief Configure with map parameters
   * @param robot_radius Robot's physical radius (m)
   * @param inflation_radius Costmap inflation radius (m)
   * @param min_frontier_size Minimum cells for valid frontier cluster
   * @param safety_ratio Required ratio of safe cells in 1m radius (0.0-1.0)
   */
  void configure(
    double robot_radius = 0.13,
    double inflation_radius = 0.35,
    int min_frontier_size = 4,
    double safety_ratio = 0.98);

  /**
   * @brief Find all frontiers in the occupancy grid
   * @param costmap Occupancy grid message
   * @return Vector of frontier structs sorted by distance
   */
  std::vector<Frontier> searchFromMap(
    const nav_msgs::msg::OccupancyGrid & costmap,
    double robot_x, double robot_y);

  /**
   * @brief Select best frontier (closest reachable)
   * @param frontiers Vector of frontiers
   * @param blacklist Set of blacklisted centroid positions
   * @param min_frontier_distance Minimum distance to consider a frontier valid
   * @return Best frontier or nullptr if none available
   */
  const Frontier * selectBestFrontier(
    const std::vector<Frontier> & frontiers,
    const std::set<std::pair<double, double>> & blacklist,
    double min_frontier_distance = 0.5);

private:
  // Parameters
  double robot_radius_{0.13};
  double inflation_radius_{0.35};
  int min_frontier_size_{4};
  double safety_ratio_{0.98};

  // Map info (cached for performance)
  double resolution_{0.05};
  int width_{0};
  int height_{0};
  double origin_x_{0.0};
  double origin_y_{0.0};

  // Internal methods
  bool isFrontierCell(const std::vector<int8_t> & data, int x, int y) const;
  bool isSafeArea(const std::vector<int8_t> & data, int x, int y) const;
  std::vector<Frontier> clusterFrontiers(
    const std::vector<std::pair<int, int>> & frontier_cells,
    double robot_x, double robot_y) const;
  
  inline int toIndex(int x, int y) const { return y * width_ + x; }
  inline bool isValid(int x, int y) const { return x >= 0 && x < width_ && y >= 0 && y < height_; }
  geometry_msgs::msg::Point gridToWorld(int x, int y) const;
};

}  // namespace ausra_frontier_exploration

#endif  // AUSRA_FRONTIER_EXPLORATION__FRONTIER_SEARCH_HPP_
