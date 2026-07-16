cmake_minimum_required(VERSION 3.8)
project(global_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(hyu_formular_control_msgs REQUIRED)

add_library(global_planner_slam_geometry
  src/planner_geometry.cpp
  src/slam_boundary_ordering.cpp
  src/slam_centerline_builder.cpp
)
target_include_directories(global_planner_slam_geometry PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(global_planner_slam_geometry
  eufs_msgs
  nav_msgs
)

add_executable(global_planner_trajectory_publisher_node
  src/global_planner_trajectory_publisher_node.cpp
  src/trajectory_csv.cpp
)
target_include_directories(global_planner_trajectory_publisher_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(global_planner_trajectory_publisher_node
  rclcpp
  eufs_msgs
  std_msgs
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

add_executable(planner_node
  src/planner_node.cpp
)
target_include_directories(planner_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(planner_node global_planner_slam_geometry)
ament_target_dependencies(planner_node
  rclcpp
  eufs_msgs
  std_msgs
  nav_msgs
)

add_library(global_planner_tmpc_trajectory_builder STATIC
  src/tmpc_trajectory_builder.cpp
)
target_include_directories(global_planner_tmpc_trajectory_builder PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(global_planner_tmpc_trajectory_builder
  eufs_msgs
  std_msgs
  nav_msgs
  hyu_formular_control_msgs
)

add_library(global_planner_wpnt_publisher STATIC
  src/global_path_validity_gate.cpp
  src/path_snapshot.cpp
  src/wpnt_publisher_node.cpp
)
target_include_directories(global_planner_wpnt_publisher PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(global_planner_wpnt_publisher
  global_planner_tmpc_trajectory_builder
)
ament_target_dependencies(global_planner_wpnt_publisher
  rclcpp
  eufs_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  ament_index_cpp
  hyu_formular_control_msgs
)

add_executable(wpnt_publisher_node
  src/wpnt_publisher_main.cpp
)
target_link_libraries(wpnt_publisher_node
  global_planner_wpnt_publisher
)
ament_target_dependencies(wpnt_publisher_node
  rclcpp
  eufs_msgs
  std_msgs
  nav_msgs
  hyu_formular_control_msgs
)

install(TARGETS
  global_planner_trajectory_publisher_node
  global_planner_debug_visualizer_node
  planner_node
  wpnt_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  get_filename_component(GLOBAL_PLANNER_SRC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

  foreach(test_target
      slam_boundary_ordering_test
      slam_centerline_builder_test
      map_geometry_fail_closed_test
      bad_map_duplicate_test
      invalid_width_test
      branch_jump_test
      raceline_smoothing_test)
    ament_add_gtest(${test_target} test/${test_target}.cpp)
    target_link_libraries(${test_target} global_planner_slam_geometry)
    target_compile_definitions(${test_target} PRIVATE
      GLOBAL_PLANNER_SRC_ROOT="${GLOBAL_PLANNER_SRC_ROOT}")
    ament_target_dependencies(${test_target}
      eufs_msgs
      nav_msgs
    )
  endforeach()

  ament_add_gtest(tmpc_trajectory_builder_test
    test/tmpc_trajectory_builder_test.cpp
  )
  target_link_libraries(tmpc_trajectory_builder_test
    global_planner_tmpc_trajectory_builder
  )
  ament_target_dependencies(tmpc_trajectory_builder_test
    eufs_msgs
    std_msgs
    nav_msgs
    hyu_formular_control_msgs
  )

  ament_add_gtest(global_path_validity_gate_test
    test/global_path_validity_gate_test.cpp
  )
  target_link_libraries(global_path_validity_gate_test
    global_planner_wpnt_publisher
  )
  ament_target_dependencies(global_path_validity_gate_test
    rclcpp
    eufs_msgs
    std_msgs
    nav_msgs
    hyu_formular_control_msgs
  )

  ament_add_gtest(wpnt_publisher_tmpc_test
    test/wpnt_publisher_tmpc_test.cpp
  )
  target_link_libraries(wpnt_publisher_tmpc_test
    global_planner_wpnt_publisher
  )
  ament_target_dependencies(wpnt_publisher_tmpc_test
    rclcpp
    eufs_msgs
    std_msgs
    nav_msgs
    geometry_msgs
    ament_index_cpp
    hyu_formular_control_msgs
  )
endif()

ament_package()
