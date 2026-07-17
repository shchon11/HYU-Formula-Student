cmake_minimum_required(VERSION 3.8)
project(hyu_frenet_conversion)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- ament / ROS dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hyu_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ---- External dependencies required by the CommonRoad-CLCS C++ core ----
find_package(Eigen3 3.3 REQUIRED)
find_package(Boost REQUIRED)        # Boost.Geometry is header-only (Boost::headers)
find_package(spdlog REQUIRED)
find_package(OpenMP REQUIRED)       # crccosy calls omp_set_* directly -> mandatory
find_package(Threads REQUIRED)

# ---- Locate the CommonRoad-CLCS source checkout ----
# Resolution order (no hard-coded absolute path baked into the target):
#   1. -DCOMMONROAD_CLCS_DIR=... passed to cmake / colcon --cmake-args
#   2. COMMONROAD_CLCS_DIR environment variable
#   3. $HOME/commonroad-clcs (default checkout location)
if(NOT COMMONROAD_CLCS_DIR)
  if(DEFINED ENV{COMMONROAD_CLCS_DIR})
    set(COMMONROAD_CLCS_DIR "$ENV{COMMONROAD_CLCS_DIR}")
  else()
    set(COMMONROAD_CLCS_DIR "$ENV{HOME}/commonroad-clcs")
  endif()
endif()
set(COMMONROAD_CLCS_DIR "${COMMONROAD_CLCS_DIR}" CACHE PATH
  "Path to the CommonRoad-CLCS checkout (must contain cpp/include and cpp/src)")

set(CLCS_INCLUDE_DIR "${COMMONROAD_CLCS_DIR}/cpp/include")
set(CLCS_SRC_DIR "${COMMONROAD_CLCS_DIR}/cpp/src")

if(NOT EXISTS "${CLCS_INCLUDE_DIR}/geometry/curvilinear_coordinate_system.h")
  message(FATAL_ERROR
    "CommonRoad-CLCS headers were not found under '${CLCS_INCLUDE_DIR}'.\n"
    "Set COMMONROAD_CLCS_DIR to the commonroad-clcs checkout, e.g.\n"
    "  colcon build --packages-select hyu_frenet_conversion \\\n"
    "    --cmake-args -DCOMMONROAD_CLCS_DIR=/path/to/commonroad-clcs\n"
    "or export COMMONROAD_CLCS_DIR before building.")
endif()

message(STATUS "hyu_frenet_conversion: using CommonRoad-CLCS at ${COMMONROAD_CLCS_DIR}")

# ---- Build the CLCS core as a static library (mirrors upstream cpp/CMakeLists.txt) ----
set(CLCS_CORE_SOURCES
  ${CLCS_SRC_DIR}/segment.cc
  ${CLCS_SRC_DIR}/curvilinear_coordinate_system.cc
  ${CLCS_SRC_DIR}/projection_domain.cc
  ${CLCS_SRC_DIR}/path_segments.cc
  ${CLCS_SRC_DIR}/util.cc
  ${CLCS_SRC_DIR}/util_sweep_line.cc
  ${CLCS_SRC_DIR}/util_projection_domain.cc
  # Serialization TUs are compiled but become empty under ENABLE_SERIALIZER=0.
  ${CLCS_SRC_DIR}/serialize/serialize_curvil_cs.cc
  ${CLCS_SRC_DIR}/serialize/public/serialize_public.cc
)

add_library(crccosy STATIC ${CLCS_CORE_SOURCES})
target_compile_features(crccosy PUBLIC cxx_std_17)
# Serialization/pickling disabled: the frenet node never (de)serializes a CLCS,
# so we avoid the extra libs11n / Boost.Serialization dependency.
target_compile_definitions(crccosy PRIVATE ENABLE_SERIALIZER=0)
# SYSTEM so warnings from the third-party CLCS headers do not pollute our build
# (also propagates to consumers that include geometry/*.h).
target_include_directories(crccosy SYSTEM PUBLIC "${CLCS_INCLUDE_DIR}")
set_target_properties(crccosy PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(crccosy
  PUBLIC
    Eigen3::Eigen
    Boost::headers
    spdlog::spdlog
  PRIVATE
    OpenMP::OpenMP_CXX
    Threads::Threads
)

# ---- frenet_odom_node ----
add_executable(frenet_odom_node
  src/frenet_odom_node.cpp
  src/clcs_frenet_converter.cpp
  src/clcs_build_diagnostics.cpp
  src/frenet_odom_messages.cpp
  src/frenet_odom_parameters.cpp
  src/global_path_validity_state.cpp
)
target_include_directories(frenet_odom_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(frenet_odom_node crccosy)
ament_target_dependencies(frenet_odom_node
  rclcpp
  hyu_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(frenet_odom_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS frenet_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
