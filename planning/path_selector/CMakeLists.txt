cmake_minimum_required(VERSION 3.8)
project(path_selector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(path_selector_core
  src/continuity_check.cpp
  src/continuity_geometry.cpp
  src/selection_policy.cpp
)
target_include_directories(path_selector_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(path_selector_core
  eufs_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
)

add_executable(path_selector_node
  src/path_selector_main.cpp
  src/path_selector_node.cpp
)
target_include_directories(path_selector_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(path_selector_node path_selector_core)
ament_target_dependencies(path_selector_node
  eufs_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
)

install(
  TARGETS path_selector_core path_selector_node
  EXPORT export_path_selector_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_selection_policy test/test_selection_policy.cpp)
  target_link_libraries(test_selection_policy path_selector_core)

  ament_add_gtest(test_continuity_check test/test_continuity_check.cpp)
  target_link_libraries(test_continuity_check path_selector_core)

  add_test(
    NAME path_selector_launch_starts
    COMMAND ${CMAKE_COMMAND} -E env
      "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}"
      bash ${CMAKE_CURRENT_SOURCE_DIR}/test/test_path_selector_launch.sh
  )
  set_tests_properties(path_selector_launch_starts PROPERTIES TIMEOUT 15)
endif()

ament_export_targets(export_path_selector_core HAS_LIBRARY_TARGET)
ament_export_dependencies(eufs_msgs geometry_msgs nav_msgs rclcpp std_msgs)

ament_package()
