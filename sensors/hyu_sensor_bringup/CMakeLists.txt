cmake_minimum_required(VERSION 3.8)
project(hyu_sensor_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/sync_capture.py DESTINATION lib/${PROJECT_NAME})

ament_package()
