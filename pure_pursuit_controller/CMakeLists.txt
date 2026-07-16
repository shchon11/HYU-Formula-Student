cmake_minimum_required(VERSION 3.8)
project(pure_pursuit_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(pure_pursuit_controller_core
  src/controller.cpp
)
target_include_directories(pure_pursuit_controller_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(pure_pursuit_controller_node
  src/pure_pursuit_controller_node.cpp
)
target_link_libraries(pure_pursuit_controller_node
  pure_pursuit_controller_core
)
ament_target_dependencies(pure_pursuit_controller_node
  rclcpp
  ackermann_msgs
  eufs_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
)

install(TARGETS
  pure_pursuit_controller_core
  pure_pursuit_controller_node
  EXPORT pure_pursuit_controller_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_controller test/test_controller.cpp)
  target_link_libraries(test_controller pure_pursuit_controller_core)
endif()

ament_export_include_directories(include)
ament_export_targets(pure_pursuit_controller_targets HAS_LIBRARY_TARGET)
ament_package()
