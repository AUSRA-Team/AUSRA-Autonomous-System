cmake_minimum_required(VERSION 3.8)
project(ausra_map_merge_HW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

# Build the map expansion node executable
add_executable(map_expansion_node src/map_expansion_node.cpp)

ament_target_dependencies(map_expansion_node
  rclcpp
  nav_msgs
)

# Install targets
install(TARGETS
  map_expansion_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install docs
install(DIRECTORY docs/
  DESTINATION share/${PROJECT_NAME}/docs
)

ament_package()
