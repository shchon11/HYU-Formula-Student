cmake_minimum_required(VERSION 3.8)
project(hyu_cmd_selector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}_policy
  src/selector_policy.cpp
)
target_include_directories(${PROJECT_NAME}_policy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(${PROJECT_NAME}_node
  src/hyu_cmd_selector_node.cpp
)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_policy)
ament_target_dependencies(${PROJECT_NAME}_node
  ackermann_msgs
  rclcpp
  std_msgs
)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_node)
ament_target_dependencies(${PROJECT_NAME} rclcpp)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_policy
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_selector_policy test/test_selector_policy.cpp)
  if(TARGET test_selector_policy)
    target_link_libraries(test_selector_policy ${PROJECT_NAME}_policy)
  endif()

  ament_add_gtest(test_hyu_cmd_selector_node test/test_hyu_cmd_selector_node.cpp)
  if(TARGET test_hyu_cmd_selector_node)
    target_link_libraries(test_hyu_cmd_selector_node ${PROJECT_NAME}_node)
    ament_target_dependencies(test_hyu_cmd_selector_node
      ackermann_msgs
      rclcpp
      std_msgs
    )
  endif()
endif()

ament_package()
