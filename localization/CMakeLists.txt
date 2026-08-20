cmake_minimum_required(VERSION 3.8)
project(hyu_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module QUIET)
find_package(Eigen3 REQUIRED)
find_package(hyu_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sbg_driver REQUIRED)
# Optional: RViz HUD overlay for the raw EKF node, and the bag eval tool.
find_package(rviz_2d_overlay_msgs QUIET)
find_package(rosbag2_cpp QUIET)
find_package(rosbag2_storage QUIET)

find_package(g2o QUIET CONFIG)
if(NOT g2o_FOUND)
  set(G2O_VENDOR_SOURCE_DIR "" CACHE PATH "Path to a g2o source tree")
  if(NOT G2O_VENDOR_SOURCE_DIR)
    get_filename_component(_workspace_parent "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
    set(_g2o_candidate "${_workspace_parent}/g2o")
    if(EXISTS "${_g2o_candidate}/CMakeLists.txt")
      set(G2O_VENDOR_SOURCE_DIR "${_g2o_candidate}" CACHE PATH "Path to a g2o source tree" FORCE)
    endif()
  endif()

  if(NOT EXISTS "${G2O_VENDOR_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
      "Could not find g2o. Install a g2o CMake package or configure with "
      "-DG2O_VENDOR_SOURCE_DIR=/path/to/g2o. In this workspace, the expected "
      "sibling location is ../g2o from eufs_simulator.")
  endif()

  set(G2O_BUILD_APPS OFF CACHE BOOL "" FORCE)
  set(G2O_BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
  set(G2O_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(G2O_BUILD_SLAM2D_TYPES ON CACHE BOOL "" FORCE)
  set(G2O_BUILD_SLAM2D_ADDON_TYPES OFF CACHE BOOL "" FORCE)
  set(G2O_BUILD_DATA_TYPES OFF CACHE BOOL "" FORCE)
  set(G2O_BUILD_SCLAM2D_TYPES OFF CACHE BOOL "" FORCE)
  set(G2O_BUILD_SLAM3D_TYPES OFF CACHE BOOL "" FORCE)
  set(G2O_BUILD_SBA_TYPES OFF CACHE BOOL "" FORCE)
  set(G2O_BUILD_ICP_TYPES OFF CACHE BOOL "" FORCE)
  set(G2O_BUILD_SIM3_TYPES OFF CACHE BOOL "" FORCE)
  set(G2O_USE_CHOLMOD OFF CACHE BOOL "" FORCE)
  set(G2O_USE_CSPARSE OFF CACHE BOOL "" FORCE)
  set(G2O_USE_LGPL_LIBS OFF CACHE BOOL "" FORCE)
  set(G2O_USE_OPENGL OFF CACHE BOOL "" FORCE)
  set(G2O_USE_OPENMP OFF CACHE BOOL "" FORCE)

  add_subdirectory("${G2O_VENDOR_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/g2o_vendor" EXCLUDE_FROM_ALL)
endif()

foreach(_g2o_target core stuff solver_eigen types_slam2d)
  if(TARGET g2o::${_g2o_target})
    list(APPEND EUFS_GRAPH_SLAM_G2O_TARGETS g2o::${_g2o_target})
  elseif(TARGET ${_g2o_target})
    list(APPEND EUFS_GRAPH_SLAM_G2O_TARGETS ${_g2o_target})
  else()
    message(FATAL_ERROR "Required g2o target '${_g2o_target}' was not found")
  endif()
endforeach()

add_executable(graph_slam_node
  src/graph_slam_node.cpp
  src/main.cpp)

target_include_directories(graph_slam_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

ament_target_dependencies(graph_slam_node
  hyu_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  std_srvs
  tf2_ros
  visualization_msgs)

target_link_libraries(graph_slam_node
  Eigen3::Eigen
  ${EUFS_GRAPH_SLAM_G2O_TARGETS})

install(TARGETS graph_slam_node
  DESTINATION lib/${PROJECT_NAME})

# --- raw GNSS/IMU odometry EKF (replaces sbg_odometry_bridge.py) -----------
# Core filter as a library so the node, the bag eval tool and the gtest share
# one implementation (port of the offline reference my_ekf.py).
add_library(raw_gnss_ekf STATIC src/raw_gnss_ekf.cpp)
target_include_directories(raw_gnss_ekf PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(raw_gnss_ekf PUBLIC Eigen3::Eigen)

add_executable(sbg_raw_ekf
  src/sbg_raw_ekf_node.cpp
  src/sbg_raw_ekf_main.cpp)
ament_target_dependencies(sbg_raw_ekf
  diagnostic_msgs
  hyu_msgs
  nav_msgs
  rclcpp
  sbg_driver)
target_link_libraries(sbg_raw_ekf raw_gnss_ekf)
if(rviz_2d_overlay_msgs_FOUND)
  ament_target_dependencies(sbg_raw_ekf rviz_2d_overlay_msgs)
  target_compile_definitions(sbg_raw_ekf PRIVATE HYU_HAVE_RVIZ_OVERLAY=1)
endif()
install(TARGETS sbg_raw_ekf
  DESTINATION lib/${PROJECT_NAME})

if(rosbag2_cpp_FOUND AND rosbag2_storage_FOUND)
  add_executable(sbg_raw_ekf_bag_eval src/sbg_raw_ekf_bag_eval.cpp)
  ament_target_dependencies(sbg_raw_ekf_bag_eval
    rclcpp
    rosbag2_cpp
    rosbag2_storage
    sbg_driver)
  target_link_libraries(sbg_raw_ekf_bag_eval raw_gnss_ekf)
  install(TARGETS sbg_raw_ekf_bag_eval
    DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "rosbag2_cpp not found: sbg_raw_ekf_bag_eval is not built")
endif()

install(PROGRAMS scripts/slam_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME slam_gui)

install(PROGRAMS scripts/ate_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ate_monitor)

install(PROGRAMS scripts/wheel_odometry.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wheel_odometry)

install(PROGRAMS scripts/status_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME status_monitor)

install(DIRECTORY config launch map
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # The python nodes under scripts/ had no test runner at all, so the INS
  # coupling in wheel_odometry (body-frame sign conventions, slip direction)
  # was unguarded. Mirrors eufs_sensors' pytest wiring.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(wheel_odometry_tests test/test_wheel_odometry.py
    TIMEOUT 120
  )

  ament_add_gtest(raw_gnss_ekf_test
    test/raw_gnss_ekf_test.cpp)
  target_link_libraries(raw_gnss_ekf_test raw_gnss_ekf)
  ament_target_dependencies(raw_gnss_ekf_test sbg_driver)

  ament_add_gtest(loop_confirmation_test
    test/loop_confirmation_test.cpp)
  target_include_directories(loop_confirmation_test PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

  ament_add_gtest(mapping_stop_classifier_test
    test/mapping_stop_classifier_test.cpp)
  target_include_directories(mapping_stop_classifier_test PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

  ament_add_gtest(lap_finish_gate_test
    test/lap_finish_gate_test.cpp)
  target_include_directories(lap_finish_gate_test PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

  ament_add_gtest(gate_anchor_test
    test/gate_anchor_test.cpp)
  target_include_directories(gate_anchor_test PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

  ament_add_gtest(local_submap_test
    test/local_submap_test.cpp)
  target_include_directories(local_submap_test PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

  ament_add_gtest(tentative_track_frontend_test
    test/tentative_track_frontend_test.cpp)
  target_include_directories(tentative_track_frontend_test PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  target_link_libraries(tentative_track_frontend_test Eigen3::Eigen)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
