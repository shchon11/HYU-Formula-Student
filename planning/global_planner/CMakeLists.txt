cmake_minimum_required(VERSION 3.8)
project(global_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(global_planner_trajectory_publisher_node
  src/global_planner_trajectory_publisher_node.cpp
)
target_include_directories(global_planner_trajectory_publisher_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(global_planner_trajectory_publisher_node
  rclcpp
  eufs_msgs
)

add_executable(global_planner_debug_visualizer_node
  src/global_planner_debug_visualizer_node.cpp
)
target_include_directories(global_planner_debug_visualizer_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(global_planner_debug_visualizer_node
  rclcpp
  geometry_msgs
  visualization_msgs
)

add_executable(wpnt_publisher_node
  src/wpnt_publisher_node.cpp
)
target_include_directories(wpnt_publisher_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(wpnt_publisher_node
  rclcpp
  eufs_msgs
  nav_msgs
  geometry_msgs
)

install(TARGETS
  global_planner_trajectory_publisher_node
  global_planner_debug_visualizer_node
  wpnt_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
