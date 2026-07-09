cmake_minimum_required(VERSION 3.8)
project(state_machine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(planning_state_machine_node
  src/global_path_readiness.cpp
  src/lap_tracking.cpp
  src/planning_state_debug.cpp
  src/planning_state_machine_main.cpp
  src/planning_state_machine_node.cpp
  src/planning_state_outputs.cpp
)
ament_target_dependencies(planning_state_machine_node
  eufs_msgs
  nav_msgs
  rclcpp
  std_msgs
)

install(TARGETS
  planning_state_machine_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  docs
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
