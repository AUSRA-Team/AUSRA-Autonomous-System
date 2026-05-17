cmake_minimum_required(VERSION 3.8)
project(ausra_frontier_exploration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(slam_toolbox REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  nav_msgs
  geometry_msgs
  visualization_msgs
  nav2_msgs
  nav2_util
  nav2_costmap_2d
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  std_srvs
  slam_toolbox
)

include_directories(include)

# Library
add_library(frontier_search SHARED
  src/frontier_search.cpp
)
ament_target_dependencies(frontier_search ${dependencies})

# Exploration server node (original)
add_executable(exploration_server
  src/exploration_server.cpp
  src/frontier_search.cpp
)
ament_target_dependencies(exploration_server ${dependencies})

# Enhanced exploration server with return-to-home
add_executable(exploration_server_enhanced
  src/exploration_server_enhanced.cpp
  src/frontier_search.cpp
)
ament_target_dependencies(exploration_server_enhanced ${dependencies})
target_link_libraries(exploration_server_enhanced
  ${std_msgs_TARGETS}
)

# Install
install(TARGETS
  frontier_search
  exploration_server
  exploration_server_enhanced
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(frontier_search)
ament_export_dependencies(${dependencies})
ament_package()
