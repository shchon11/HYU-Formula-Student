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

  # Deterministic synthetic-scenario fixtures vendored from ksae_local
  # (test/test_utils + test/vendor). Test-only; never installed.
  add_library(local_planner_test_scenarios STATIC
    test/test_utils/synthetic_scenario.cpp
    test/test_utils/scenario_builder.cpp
    test/test_utils/fault_injector.cpp
    test/test_utils/scenario_catalog.cpp
    test/vendor/geometry.cpp)
  target_compile_features(local_planner_test_scenarios PUBLIC cxx_std_17)
  target_include_directories(local_planner_test_scenarios PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/test
    ${CMAKE_CURRENT_SOURCE_DIR}/test/vendor)

  ament_add_gtest(local_planner_test test/local_planner_test.cpp)
  target_link_libraries(local_planner_test local_planner_core)

  ament_add_gtest(local_planner_scenario_adapter_test test/scenario_adapter_test.cpp)
  target_link_libraries(local_planner_scenario_adapter_test
    local_planner_core local_planner_test_scenarios)

  ament_add_gtest(local_planner_orange_gate_test test/orange_gate_test.cpp)
  target_link_libraries(local_planner_orange_gate_test local_planner_core)

  ament_add_gtest(local_planner_boa_golden_test test/boa_golden_test.cpp)
  target_link_libraries(local_planner_boa_golden_test local_planner_core)
  target_compile_definitions(local_planner_boa_golden_test PRIVATE
    BOA_TRACK_CSV="${CMAKE_CURRENT_SOURCE_DIR}/../../eufs_sim/eufs_tracks/csv/boa_constrictor.csv"
    BOA_GOLDEN_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/data/boa_golden.txt")

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
