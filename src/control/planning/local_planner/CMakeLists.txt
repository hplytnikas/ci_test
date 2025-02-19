cmake_minimum_required(VERSION 3.8)
project(local_planner)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Optimization flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Ofast -ffast-math")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast -ffast-math")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(easy_profiler REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autonomous_msgs REQUIRED)
find_package(vcu_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(min_curvature_opt REQUIRED)
find_package(generate_parameter_library REQUIRED)

# Generate parameters from YAML
generate_parameter_library(
  local_planner_params
  param_definition/local_planner_params.yaml
)

# ACADOS configuration
set(ACADOS_DIR $ENV{ACADOS_DIR})
if(NOT ACADOS_DIR)
  set(ACADOS_DIR "/usr/local/share/amz/acados")
endif()

set(acados_include "${ACADOS_DIR}/include")
set(acados_lib "${ACADOS_DIR}/lib")

# Include directories
include_directories(
  include
  ${acados_include}
  ${acados_include}/blasfeo/include
  ${acados_include}/hpipm/include
)

# Link directories
link_directories(
  ${acados_lib}
)

# Executable definition
add_executable(${PROJECT_NAME}
  src/local_planner_node.cpp
  src/local_planner_node_main.cpp
)

# Target dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  autonomous_msgs
  control_msgs
  std_msgs
  vcu_msgs
  min_curvature_opt
  easy_profiler
  nav_msgs
)

# Linking libraries
target_link_libraries(${PROJECT_NAME}
  hpipm
  blasfeo
  local_planner_params
)

# Installation rules
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  autonomous_msgs
  std_msgs
  vcu_msgs
  control_msgs
  min_curvature_opt
  easy_profiler
  nav_msgs
)

ament_package()
