cmake_minimum_required(VERSION 3.5)
project(tum_mpc_output_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(hyu_formular_control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}_conversion
  src/command_conversion.cpp
)
target_include_directories(${PROJECT_NAME}_conversion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(${PROJECT_NAME}
  src/tum_mpc_output_bridge_node.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_conversion
)
ament_target_dependencies(${PROJECT_NAME}
  ackermann_msgs
  hyu_formular_control_msgs
  rclcpp
  std_msgs
)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_conversion
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_command_conversion
    test/test_command_conversion.cpp
  )
  if(TARGET test_command_conversion)
    target_link_libraries(test_command_conversion
      ${PROJECT_NAME}_conversion
    )
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
