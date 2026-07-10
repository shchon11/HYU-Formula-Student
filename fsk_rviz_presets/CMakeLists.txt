cmake_minimum_required(VERSION 3.8)
project(fsk_rviz_presets)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES include/fsk_rviz_presets/stack_displays.hpp)

add_library(${PROJECT_NAME} SHARED
  src/stack_displays.cpp
  ${MOC_FILES}
)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  rviz_common
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_package()
