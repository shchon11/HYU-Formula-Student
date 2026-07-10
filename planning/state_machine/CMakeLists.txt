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

add_library(state_machine_core
  src/global_path_readiness.cpp
  src/lap_tracking.cpp
  src/planning_state_debug.cpp
  src/planning_state_machine_node.cpp
  src/planning_state_outputs.cpp
)
target_include_directories(state_machine_core PUBLIC include)
ament_target_dependencies(state_machine_core
  eufs_msgs
  nav_msgs
  rclcpp
  std_msgs
)

add_executable(planning_state_machine_node
  src/planning_state_machine_main.cpp
)
target_link_libraries(planning_state_machine_node state_machine_core)
ament_target_dependencies(planning_state_machine_node rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(planning_state_machine_policy_test
    test/planning_state_machine_policy_test.cpp
  )
  target_link_libraries(planning_state_machine_policy_test state_machine_core)
  ament_target_dependencies(planning_state_machine_policy_test
    eufs_msgs
    rclcpp
  )
endif()

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
