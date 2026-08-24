cmake_minimum_required(VERSION 3.8)
project(planner_replay)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(message_filters REQUIRED)
find_package(hyu_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
get_filename_component(LP "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
add_executable(planner_replay replay_planner.cpp
  ${LP}/src/input_policy.cpp ${LP}/src/local_path_classify.cpp ${LP}/src/local_path_delaunay.cpp
  ${LP}/src/local_path_geometry.cpp ${LP}/src/local_path_intersections.cpp ${LP}/src/local_path_builder.cpp ${LP}/src/local_path_live_extension.cpp
  ${LP}/src/local_path_sampling.cpp ${LP}/src/ros_inputs.cpp)
target_include_directories(planner_replay PRIVATE ${LP}/include ${LP}/src)
ament_target_dependencies(planner_replay rclcpp rosbag2_cpp rosbag2_storage message_filters hyu_msgs nav_msgs geometry_msgs std_msgs)
