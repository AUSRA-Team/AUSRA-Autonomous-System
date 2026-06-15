cmake_minimum_required(VERSION 3.8)
project(ausra_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config scripts systemd zenoh
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/ns_resolver.py
  scripts/send_namespace_to_esp32.sh
  scripts/gen_zenoh_config.sh
  scripts/start_micro_ros.sh
  scripts/install_jetson.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
