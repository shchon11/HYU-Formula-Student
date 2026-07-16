cmake_minimum_required(VERSION 3.5)
project(eufs_sensors)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(ament_cmake)

install(DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

# Simulated SBG Ellipse-D GNSS/INS (ground truth -> /sbg/ekf_nav + ekf_euler).
install(PROGRAMS scripts/sim_ellipse_d.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sim_ellipse_d
)

ament_package()
