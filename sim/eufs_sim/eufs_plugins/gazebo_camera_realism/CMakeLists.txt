# In-plugin camera realism (rolling shutter + motion blur inside the stock
# gazebo_ros_camera publish path). std::clamp needs C++17.
set(CMAKE_CXX_STANDARD 17)

find_package(gazebo_plugins REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_library(gazebo_camera_realism SHARED
  src/gazebo_camera_realism.cpp)

ament_target_dependencies(gazebo_camera_realism
  "gazebo_dev"
  "gazebo_ros"
  "gazebo_plugins"
  "rclcpp")

target_link_libraries(gazebo_camera_realism ${OpenCV_LIBS})
target_include_directories(gazebo_camera_realism PUBLIC ${OpenCV_INCLUDE_DIRS})

ament_export_libraries(gazebo_camera_realism)

install(TARGETS gazebo_camera_realism
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib)
