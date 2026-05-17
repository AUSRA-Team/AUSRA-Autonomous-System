cmake_minimum_required(VERSION 3.8)
project(ausra_map_merge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Find dependencies ──────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ── map_expansion_node executable ─────────────────────────────────────────
add_executable(map_expansion_node src/map_expansion_node.cpp)
ament_target_dependencies(map_expansion_node
  rclcpp
  nav_msgs
  geometry_msgs
)

# ── Install ────────────────────────────────────────────────────────────────
install(TARGETS
  map_expansion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
