cmake_minimum_required(VERSION 3.8)
project(local_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(local_planner_core
  src/input_policy.cpp
  src/local_path_classify.cpp
  src/local_path_delaunay.cpp
  src/local_path_geometry.cpp
  src/local_path_intersections.cpp
  src/local_path_builder.cpp
  src/local_path_sampling.cpp
)
target_compile_features(local_planner_core PUBLIC cxx_std_17)
target_include_directories(local_planner_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_executable(local_planner_node
  src/local_planner_main.cpp
  src/local_planner_node.cpp
  src/ros_inputs.cpp
  src/ros_outputs.cpp
)
target_compile_features(local_planner_node PUBLIC cxx_std_17)
target_include_directories(local_planner_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(local_planner_node local_planner_core)
ament_target_dependencies(local_planner_node
  rclcpp
  message_filters
  eufs_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)

install(TARGETS local_planner_core local_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(local_planner_test test/local_planner_test.cpp)
  target_link_libraries(local_planner_test local_planner_core)

  ament_add_gtest(local_planner_input_policy_test test/local_planner_input_policy_test.cpp)
  target_link_libraries(local_planner_input_policy_test local_planner_core)

  ament_add_gtest(local_planner_robustness_test test/local_planner_robustness_test.cpp)
  target_link_libraries(local_planner_robustness_test local_planner_core)

  ament_add_gtest(local_planner_chicane_fold_test test/chicane_fold_test.cpp)
  target_link_libraries(local_planner_chicane_fold_test local_planner_core)
  target_compile_definitions(local_planner_chicane_fold_test PRIVATE
    BOA_TRACK_CSV="${CMAKE_CURRENT_SOURCE_DIR}/../../eufs_sim/eufs_tracks/csv/boa_constrictor.csv")

  ament_add_gtest(local_planner_validity_test
    test/local_planner_validity_test.cpp
    src/local_planner_node.cpp
    src/ros_inputs.cpp
    src/ros_outputs.cpp)
  target_link_libraries(local_planner_validity_test local_planner_core)
  target_include_directories(local_planner_validity_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(local_planner_validity_test
    rclcpp
    message_filters
    eufs_msgs
    nav_msgs
    geometry_msgs
    std_msgs)

  ament_add_gtest(local_planner_stale_recovery_test
    test/local_planner_stale_recovery_test.cpp
    src/local_planner_node.cpp
    src/ros_inputs.cpp
    src/ros_outputs.cpp)
  target_link_libraries(local_planner_stale_recovery_test local_planner_core)
  target_include_directories(local_planner_stale_recovery_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(local_planner_stale_recovery_test
    rclcpp
    message_filters
    eufs_msgs
    nav_msgs
    geometry_msgs
    std_msgs)

  ament_add_gtest(local_planner_slam_validity_test
    test/local_planner_slam_validity_test.cpp
    src/local_planner_node.cpp
    src/ros_inputs.cpp
    src/ros_outputs.cpp)
  target_link_libraries(local_planner_slam_validity_test local_planner_core)
  target_include_directories(local_planner_slam_validity_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(local_planner_slam_validity_test
    rclcpp
    message_filters
    eufs_msgs
    nav_msgs
    geometry_msgs
    std_msgs)
endif()

ament_export_include_directories(include)
ament_export_libraries(local_planner_core)
ament_package()
