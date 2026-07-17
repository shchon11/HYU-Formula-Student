cmake_minimum_required(VERSION 3.5)
project(tum_vehicle_state_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(hyu_formular_control_msgs REQUIRED)

add_library(${PROJECT_NAME}_heading_conversion
  src/heading_conversion.cpp
)
target_include_directories(${PROJECT_NAME}_heading_conversion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(${PROJECT_NAME}
  src/tum_vehicle_state_bridge_node.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_heading_conversion
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav_msgs
  geometry_msgs
  eufs_msgs
  tf2
  hyu_formular_control_msgs
)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_heading_conversion
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
  ament_add_gtest(test_heading_conversion
    test/test_heading_conversion.cpp
  )
  if(TARGET test_heading_conversion)
    target_link_libraries(test_heading_conversion
      ${PROJECT_NAME}_heading_conversion
    )
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
