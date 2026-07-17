cmake_minimum_required(VERSION 3.8)
project(hyu_pure_pursuit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(hyu_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(hyu_pure_pursuit_core
  src/controller.cpp
  src/steering_lookup.cpp
)
target_include_directories(hyu_pure_pursuit_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(hyu_pure_pursuit_node
  src/hyu_pure_pursuit_node.cpp
)
target_link_libraries(hyu_pure_pursuit_node
  hyu_pure_pursuit_core
)
ament_target_dependencies(hyu_pure_pursuit_node
  rclcpp
  ackermann_msgs
  hyu_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
)

install(TARGETS
  hyu_pure_pursuit_core
  hyu_pure_pursuit_node
  EXPORT hyu_pure_pursuit_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_controller test/test_controller.cpp)
  target_link_libraries(test_controller hyu_pure_pursuit_core)

  ament_add_gtest(test_steering_lookup test/test_steering_lookup.cpp)
  target_link_libraries(test_steering_lookup hyu_pure_pursuit_core)
  target_compile_definitions(test_steering_lookup PRIVATE
    "SIM_LINEAR_TABLE_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/test/data/sim_linear_lookup_table.csv\"")
endif()

ament_export_include_directories(include)
ament_export_targets(hyu_pure_pursuit_targets HAS_LIBRARY_TARGET)
ament_package()
