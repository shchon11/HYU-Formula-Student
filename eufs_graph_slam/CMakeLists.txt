cmake_minimum_required(VERSION 3.8)
project(eufs_graph_slam)

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
find_package(eufs_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(g2o QUIET CONFIG)
if(NOT g2o_FOUND)
  set(G2O_VENDOR_SOURCE_DIR "" CACHE PATH "Path to a g2o source tree")
  if(NOT G2O_VENDOR_SOURCE_DIR)
    get_filename_component(_package_parent "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
    get_filename_component(_workspace_parent "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
    foreach(_g2o_candidate
        "${_package_parent}/g2o"
        "${_workspace_parent}/g2o")
      if(EXISTS "${_g2o_candidate}/CMakeLists.txt")
        set(G2O_VENDOR_SOURCE_DIR "${_g2o_candidate}" CACHE PATH "Path to a g2o source tree" FORCE)
        break()
      endif()
    endforeach()
  endif()

  if(NOT EXISTS "${G2O_VENDOR_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
      "Could not find g2o. Install a g2o CMake package or configure with "
      "-DG2O_VENDOR_SOURCE_DIR=/path/to/g2o. Expected source locations include "
      "a g2o directory next to eufs_graph_slam or next to this repository.")
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

foreach(_g2o_target core stuff solver_dense types_slam2d)
  if(TARGET g2o::${_g2o_target})
    list(APPEND EUFS_GRAPH_SLAM_G2O_TARGETS g2o::${_g2o_target})
  elseif(TARGET ${_g2o_target})
    list(APPEND EUFS_GRAPH_SLAM_G2O_TARGETS ${_g2o_target})
  else()
    message(FATAL_ERROR "Required g2o target '${_g2o_target}' was not found")
  endif()
endforeach()

add_library(graph_slam_time_alignment
  src/observation_time_alignment.cpp)

target_include_directories(graph_slam_time_alignment PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

target_link_libraries(graph_slam_time_alignment
  Eigen3::Eigen)

add_executable(graph_slam_node
  src/graph_slam_node.cpp
  src/main.cpp)

target_include_directories(graph_slam_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

ament_target_dependencies(graph_slam_node
  eufs_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  std_srvs
  tf2_ros
  visualization_msgs)

target_link_libraries(graph_slam_node
  Eigen3::Eigen
  graph_slam_time_alignment
  ${EUFS_GRAPH_SLAM_G2O_TARGETS})

install(TARGETS graph_slam_node graph_slam_time_alignment
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_observation_time_alignment
    test/test_observation_time_alignment.cpp)
  target_link_libraries(test_observation_time_alignment
    graph_slam_time_alignment)

  set(ament_cmake_cppcheck_ADDITIONAL_INCLUDE_DIRS ${GTEST_INCLUDE_DIRS})
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
