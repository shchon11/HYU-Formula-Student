cmake_minimum_required(VERSION 3.5)
project(hyu_formular_control_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TumTrajectory.msg"
  "msg/TumVehicleState.msg"
  "msg/TumMpcOutput.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
