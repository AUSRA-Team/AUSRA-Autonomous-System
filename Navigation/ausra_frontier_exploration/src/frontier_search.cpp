// Copyright 2024 AUSRA Team
// Licensed under Apache-2.0

#include "ausra_frontier_exploration/frontier_search.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_set>

namespace ausra_frontier_exploration
{

void FrontierSearch::configure(
  double robot_radius,
  double inflation_radius,
  int min_frontier_size,
  double safety_ratio)
{
  robot_radius_ = robot_radius;
  inflation_radius_ = inflation_radius;
  min_frontier_size_ = min_frontier_size;
  safety_ratio_ = safety_ratio;
}

std::vector<Frontier> FrontierSearch::searchFromMap(
  const nav_msgs::msg::OccupancyGrid & costmap,
  double robot_x, double robot_y)
{
  // Cache map info
  resolution_ = costmap.info.resolution;
  width_ = static_cast<int>(costmap.info.width);
  height_ = static_cast<int>(costmap.info.height);
  origin_x_ = costmap.info.origin.position.x;
  origin_y_ = costmap.info.origin.position.y;

  const auto & data = costmap.data;
  std::vector<std::pair<int, int>> frontier_cells;
  frontier_cells.reserve(1000);  // Pre-allocate for performance

  // Scan for frontier cells (free cells adjacent to unknown)
  // Skip border cells for safety
  for (int y = 1; y < height_ - 1; ++y) {
    for (int x = 1; x < width_ - 1; ++x) {
      if (isFrontierCell(data, x, y)) {
        // Check safety: count obstacle neighbors
        int obs_count = 0;
        const int dx8[] = {1, -1, 0, 0, 1, -1, -1, 1};
        const int dy8[] = {0, 0, 1, -1, 1, -1, 1, -1};
        
        for (int i = 0; i < 8; ++i) {
          int nx = x + dx8[i];
          int ny = y + dy8[i];
          if (data[toIndex(nx, ny)] == 100) {
            obs_count++;
          }
        }
        
        // Filter: max 1 obstacle neighbor (from Python simulation)
        if (obs_count <= 1) {
          // Check 1m radius safety (98% safe)
          if (isSafeArea(data, x, y)) {
            frontier_cells.emplace_back(x, y);
          }
        }
      }
    }
  }

  // Cluster and return
  return clusterFrontiers(frontier_cells, robot_x, robot_y);
}

bool FrontierSearch::isFrontierCell(const std::vector<int8_t> & data, int x, int y) const
{
  // Must be free (0)
  if (data[toIndex(x, y)] != 0) {
    return false;
  }

  // Check 4-neighbors for unknown (-1)
  const int dx4[] = {1, -1, 0, 0};
  const int dy4[] = {0, 0, 1, -1};

  for (int i = 0; i < 4; ++i) {
    int nx = x + dx4[i];
    int ny = y + dy4[i];
    if (isValid(nx, ny) && data[toIndex(nx, ny)] == -1) {
      return true;
    }
  }
  return false;
}

bool FrontierSearch::isSafeArea(const std::vector<int8_t> & data, int x, int y) const
{
  // Check 1m radius for 98% safe cells
  int radius_cells = static_cast<int>(1.0 / resolution_);
  
  int x_min = std::max(0, x - radius_cells);
  int x_max = std::min(width_, x + radius_cells + 1);
  int y_min = std::max(0, y - radius_cells);
  int y_max = std::min(height_, y + radius_cells + 1);
  
  int total = 0;
  int safe = 0;
  
  for (int cy = y_min; cy < y_max; ++cy) {
    for (int cx = x_min; cx < x_max; ++cx) {
      total++;
      int8_t val = data[toIndex(cx, cy)];
      if (val == 0 || val == -1) {  // Free or unknown
        safe++;
      }
    }
  }
  
  return (static_cast<double>(safe) / total) >= safety_ratio_;
}

std::vector<Frontier> FrontierSearch::clusterFrontiers(
  const std::vector<std::pair<int, int>> & frontier_cells,
  double robot_x, double robot_y) const
{
  if (frontier_cells.empty()) {
    return {};
  }

  // Convert to set for O(1) lookup
  std::set<std::pair<int, int>> cell_set(frontier_cells.begin(), frontier_cells.end());
  std::set<std::pair<int, int>> visited;
  std::vector<Frontier> frontiers;

  const int dx8[] = {1, -1, 0, 0, 1, -1, -1, 1};
  const int dy8[] = {0, 0, 1, -1, 1, -1, 1, -1};

  for (const auto & start : frontier_cells) {
    if (visited.count(start)) {
      continue;
    }

    // BFS to cluster connected cells
    std::vector<std::pair<int, int>> cluster;
    std::queue<std::pair<int, int>> queue;
    queue.push(start);
    visited.insert(start);

    while (!queue.empty()) {
      auto curr = queue.front();
      queue.pop();
      cluster.push_back(curr);

      for (int i = 0; i < 8; ++i) {
        std::pair<int, int> neighbor = {curr.first + dx8[i], curr.second + dy8[i]};
        if (cell_set.count(neighbor) && !visited.count(neighbor)) {
          visited.insert(neighbor);
          queue.push(neighbor);
        }
      }
    }

    // Filter small clusters
    if (static_cast<int>(cluster.size()) >= min_frontier_size_) {
      Frontier f;
      f.cells = cluster;
      f.size = static_cast<double>(cluster.size());

      // For A* planning: compute centroid and then PULL IT BACK into known free space
      // This ensures the goal point is reachable by A* which needs connected paths
      double sum_x = 0.0, sum_y = 0.0;
      for (const auto & cell : cluster) {
        sum_x += cell.first;
        sum_y += cell.second;
      }
      double centroid_grid_x = sum_x / cluster.size();
      double centroid_grid_y = sum_y / cluster.size();
      
      // Direction from robot to centroid
      int robot_grid_x = static_cast<int>((robot_x - origin_x_) / resolution_);
      int robot_grid_y = static_cast<int>((robot_y - origin_y_) / resolution_);
      
      double dir_x = centroid_grid_x - robot_grid_x;
      double dir_y = centroid_grid_y - robot_grid_y;
      double dist = std::sqrt(dir_x * dir_x + dir_y * dir_y);
      
      if (dist > 0.1) {
        dir_x /= dist;
        dir_y /= dist;
      }
      
      // Pull the goal back 0.5m (10 cells at 0.05m resolution) into known space
      // This gives A* a target in FREE space that it can definitely reach
      int pullback_cells = static_cast<int>(0.5 / resolution_);
      int goal_grid_x = static_cast<int>(centroid_grid_x - dir_x * pullback_cells);
      int goal_grid_y = static_cast<int>(centroid_grid_y - dir_y * pullback_cells);
      
      // Convert to world coordinates
      f.centroid.x = origin_x_ + (goal_grid_x + 0.5) * resolution_;
      f.centroid.y = origin_y_ + (goal_grid_y + 0.5) * resolution_;
      f.centroid.z = 0.0;
      
      // Compute distance to robot (to the adjusted goal point)
      double dx = f.centroid.x - robot_x;
      double dy = f.centroid.y - robot_y;
      f.min_distance = std::sqrt(dx * dx + dy * dy);
      
      // Compute cost: prefer closer and larger frontiers
      const double potential_scale = 2.0;   // Weight for distance (higher = prefer closer)
      const double gain_scale = 0.1;        // Weight for size (higher = prefer larger frontiers)
      f.cost = potential_scale * f.min_distance - gain_scale * f.size;

      frontiers.push_back(f);
    }
  }

  // Sort by COST (lowest cost first) - balances distance and frontier size
  std::sort(frontiers.begin(), frontiers.end(),
    [](const Frontier & a, const Frontier & b) {
      return a.cost < b.cost;
    });

  return frontiers;
}

const Frontier * FrontierSearch::selectBestFrontier(
  const std::vector<Frontier> & frontiers,
  const std::set<std::pair<double, double>> & blacklist,
  double min_frontier_distance)
{
  // Maximum reasonable distance for a frontier (prevents selecting outliers)
  const double max_frontier_distance = 15.0;  // 15 meters max
  
  for (const auto & f : frontiers) {
    // Skip frontiers that are too close to the robot
    // This prevents selecting frontiers within Nav2's goal tolerance
    if (f.min_distance < min_frontier_distance) {
      continue;
    }
    
    // Skip frontiers that are too far (likely invalid or outside reasonable bounds)
    if (f.min_distance > max_frontier_distance) {
      continue;
    }
    
    // Skip frontiers with NaN or infinite coordinates
    if (std::isnan(f.centroid.x) || std::isnan(f.centroid.y) ||
        std::isinf(f.centroid.x) || std::isinf(f.centroid.y)) {
      continue;
    }
    
    std::pair<double, double> key = {f.centroid.x, f.centroid.y};
    if (blacklist.find(key) == blacklist.end()) {
      return &f;
    }
  }
  return nullptr;
}

geometry_msgs::msg::Point FrontierSearch::gridToWorld(int x, int y) const
{
  geometry_msgs::msg::Point p;
  p.x = origin_x_ + (x + 0.5) * resolution_;
  p.y = origin_y_ + (y + 0.5) * resolution_;
  p.z = 0.0;
  return p;
}

}  // namespace ausra_frontier_exploration
