cmake_minimum_required(VERSION 3.8)
project(hyu_planning_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/stack_hud.py
  scripts/skidpad_director.py
  scripts/dssi_state.py
  scripts/inspection_mission.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
