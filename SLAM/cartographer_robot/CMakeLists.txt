cmake_minimum_required(VERSION 3.8)
project(cartographer_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ========================================================================
# INSTALLATION RULES
# ========================================================================

# Install the launch directory to share/cartographer_robot/launch
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the config directory to share/cartographer_robot/config
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# ========================================================================
# EXPORT
# ========================================================================
ament_package()