cmake_minimum_required(VERSION 3.8)
project(control_harness)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(eufs_models REQUIRED)
find_package(pure_pursuit_controller REQUIRED)
find_package(local_planner REQUIRED)

add_library(control_harness_core
  src/track.cpp
  src/closed_loop.cpp
)
target_include_directories(control_harness_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(control_harness_core
  pure_pursuit_controller::pure_pursuit_controller_core
)
ament_target_dependencies(control_harness_core
  eufs_models
  local_planner
)

add_executable(map_harness src/map_harness_main.cpp)
target_link_libraries(map_harness control_harness_core)

install(TARGETS
  control_harness_core
  map_harness
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS scripts/sweep_map.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(track_test test/track_test.cpp)
  target_link_libraries(track_test control_harness_core)

  ament_add_gtest(closed_loop_test test/closed_loop_test.cpp TIMEOUT 180)
  target_link_libraries(closed_loop_test control_harness_core)
  target_compile_definitions(closed_loop_test PRIVATE
    "PLANT_CONFIG_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/../eufs_sim/eufs_racecar/robots/eufs/configDry.yaml\"")
endif()

ament_package()
