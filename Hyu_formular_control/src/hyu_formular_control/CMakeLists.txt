cmake_minimum_required(VERSION 3.5)
project(hyu_formular_control)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hyu_formular_control_msgs REQUIRED)

set(TUM_CODEGEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vendor/tum_codegen)
set(MVDC_PATH_MATCHING_DIR ${TUM_CODEGEN_DIR}/mvdc_path_matching_ert_rtw)
set(MVDC_MPC_DIR ${TUM_CODEGEN_DIR}/mvdc_mpc_ert_rtw)
set(OSQP_DIR ${TUM_CODEGEN_DIR}/osqp)
set(MATLAB_RTW_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vendor/matlab_rtw/include)

set(TUM_CODEGEN_SOURCES
  src/rtw_memory_error.cpp
  ${MVDC_PATH_MATCHING_DIR}/mvdc_path_matching.cpp
  ${MVDC_PATH_MATCHING_DIR}/mvdc_path_matching_data.cpp
  ${MVDC_PATH_MATCHING_DIR}/mvdc_path_matching_capi.cpp
  ${MVDC_MPC_DIR}/mvdc_mpc.cpp
  ${MVDC_MPC_DIR}/mvdc_mpc_data.cpp
  ${MVDC_MPC_DIR}/mvdc_mpc_capi.cpp
  ${MVDC_MPC_DIR}/rtGetInf.cpp
  ${MVDC_MPC_DIR}/rtGetNaN.cpp
  ${MVDC_MPC_DIR}/rt_nonfinite.cpp
  ${OSQP_DIR}/osqp_wrapper.c
)

add_library(hyu_tum_codegen STATIC ${TUM_CODEGEN_SOURCES})
target_include_directories(hyu_tum_codegen PUBLIC
  ${MVDC_PATH_MATCHING_DIR}
  ${MVDC_MPC_DIR}
  ${OSQP_DIR}
  ${OSQP_DIR}/header
  ${MATLAB_RTW_INCLUDE_DIR}
)
target_link_libraries(hyu_tum_codegen PUBLIC
  ${OSQP_DIR}/lib/libosqp.a
  m
)
target_compile_options(hyu_tum_codegen PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:-fpermissive -Wno-unused-parameter -Wno-sign-compare>
  $<$<COMPILE_LANGUAGE:C>:-Wno-unused-parameter>
)

add_executable(${PROJECT_NAME}
  src/hyu_formular_control_node.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  include
)
target_link_libraries(${PROJECT_NAME}
  hyu_tum_codegen
)
target_compile_options(${PROJECT_NAME} PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:-fpermissive>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hyu_formular_control_msgs
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
