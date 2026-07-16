cmake_minimum_required(VERSION 3.8)
project(planning_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/stack_hud.py
  scripts/skidpad_director.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
