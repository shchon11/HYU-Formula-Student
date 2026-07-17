cmake_minimum_required(VERSION 3.5)

add_library(gazebo_race_car_model SHARED
  src/gazebo_ros_race_car_model.cpp
  src/state_machine.cpp
  src/terrain_field.cpp)

find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(eufs_models REQUIRED)
find_package(ament_index_cpp REQUIRED)

ament_target_dependencies(gazebo_race_car_model
  "gazebo_dev"
  "gazebo_ros"
  "ament_index_cpp"
  "rclcpp"
  "eufs_msgs"
  "nav_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "tf2"
  "ackermann_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "std_srvs"
  "std_msgs"
  "eufs_models")

target_include_directories(gazebo_race_car_model PUBLIC include)
target_link_libraries(gazebo_race_car_model eufs_models::eufs_models)
ament_export_libraries(gazebo_race_car_model)

install(TARGETS gazebo_race_car_model
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # terrain_field.cpp is compiled into the test rather than linked from the
  # plugin: the field is plain geometry with no Gazebo in it, and keeping it
  # that way means it can be tested without standing up a simulator.
  ament_add_gtest(terrain_field_test
    test/terrain_field_test.cpp
    src/terrain_field.cpp)
  target_include_directories(terrain_field_test PRIVATE include)
endif()
