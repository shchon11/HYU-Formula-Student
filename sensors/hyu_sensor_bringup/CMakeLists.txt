cmake_minimum_required(VERSION 3.8)
project(hyu_sensor_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/sync_capture.py
  scripts/solve_mount.py
  scripts/stationary_wheels.py
  scripts/stationary_ins.py
  scripts/sbg_wheels.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
