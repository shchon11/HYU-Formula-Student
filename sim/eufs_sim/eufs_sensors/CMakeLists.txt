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

  # The python tests under test/ (xacro well-formedness, INS error-model
  # behaviour, lidar realism math) only exist if something RUNS them; lint
  # alone never did, so test_xacro_wellformed.py sat here unexecuted.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(sensor_behaviour_tests test
    TIMEOUT 300
  )
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

# LiDAR realism bridge: ideal gazebo scan -> real velodyne driver layout,
# motion distortion, far-grazing-ground dropout.
install(PROGRAMS scripts/lidar_realism.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lidar_realism
)

# Parity harness: same statistics from a live sim or a replayed real bag,
# so every "estimate" parameter in the sensor sim can be pinned by data.
install(PROGRAMS scripts/sensor_parity_report.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sensor_parity_report
)

ament_package()
