cmake_minimum_required(VERSION 3.8)
project(hyu_path_selector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hyu_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(hyu_path_selector_core
  src/continuity_check.cpp
  src/continuity_geometry.cpp
  src/selection_policy.cpp
  src/transition_blend.cpp
)
target_include_directories(hyu_path_selector_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(hyu_path_selector_core
  hyu_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
)

add_executable(hyu_path_selector_node
  src/hyu_path_selector_main.cpp
  src/hyu_path_selector_node.cpp
)
target_include_directories(hyu_path_selector_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(hyu_path_selector_node hyu_path_selector_core)
ament_target_dependencies(hyu_path_selector_node
  hyu_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
)

install(
  TARGETS hyu_path_selector_core hyu_path_selector_node
  EXPORT export_hyu_path_selector_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_selection_policy test/test_selection_policy.cpp)
  target_link_libraries(test_selection_policy hyu_path_selector_core)

  ament_add_gtest(test_continuity_check test/test_continuity_check.cpp)
  target_link_libraries(test_continuity_check hyu_path_selector_core)

  ament_add_gtest(test_transition_blend test/test_transition_blend.cpp)
  target_link_libraries(test_transition_blend hyu_path_selector_core)
  add_test(
    NAME empty_global_path
    COMMAND $<TARGET_FILE:test_continuity_check>
      --gtest_filter=ContinuityCheckTest.AttributesMissingGlobalPathAsGlobalCandidateFailure
  )
  add_test(
    NAME common_length
    COMMAND $<TARGET_FILE:test_continuity_check>
      --gtest_filter=ContinuityCheckTest.FailureReasonsAreDistinctForClosedHandoffCases
  )
  add_test(
    NAME heading_mismatch
    COMMAND $<TARGET_FILE:test_continuity_check>
      --gtest_filter=ContinuityCheckTest.FailureReasonsAreDistinctForClosedHandoffCases
  )

  add_test(
    NAME hyu_path_selector_launch_starts
    COMMAND ${CMAKE_COMMAND} -E env
      "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}"
      bash ${CMAKE_CURRENT_SOURCE_DIR}/test/test_hyu_path_selector_launch.sh
  )
  set_tests_properties(hyu_path_selector_launch_starts PROPERTIES TIMEOUT 15)
endif()

ament_export_targets(export_hyu_path_selector_core HAS_LIBRARY_TARGET)
ament_export_dependencies(hyu_msgs geometry_msgs nav_msgs rclcpp std_msgs)

ament_package()
