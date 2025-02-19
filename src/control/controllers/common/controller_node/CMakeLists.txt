cmake_minimum_required(VERSION 3.8)
project(controller_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17 -Ofast)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vcu_msgs REQUIRED)
find_package(autonomous_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(easy_profiler REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

set(ACADOS_DIR $ENV{ACADOS_DIR})
if(NOT DEFINED ENV{ACADOS_DIR})
  set(ACADOS_DIR /usr/local/share/amz/acados)
endif()

set(acados_include "${ACADOS_DIR}/include")
set(acados_lib "${ACADOS_DIR}/lib")

# Define library target library and its source files
add_library(${PROJECT_NAME} src/controller_node.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp vcu_msgs autonomous_msgs geometry_msgs tf2_ros tf2 tf2_geometry_msgs nav_msgs control_msgs easy_profiler)

#This adds all files in the folder ${CMAKE_CURRENT_SOURCE_DIR}/include to the public interface during build time and all files in the include folder
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    ${EASY_PROFILER_INCLUDE_DIR}
)

# Include directories of own library
include_directories(
  include
  ${acados_include}
  ${acados_include}/blasfeo/include
  ${acados_include}/hpipm/include
)

link_directories(
  include
  ${acados_lib}
  ${acados_include}
)

target_link_libraries(${PROJECT_NAME}
  hpipm
  blasfeo
  easy_profiler
)

# Expose headers to other libraries and executables
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install library so that it can be found by other packages
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp vcu_msgs autonomous_msgs generate_parameter_library geometry_msgs tf2_ros tf2 tf2_geometry_msgs nav_msgs control_msgs)

target_link_libraries(${PROJECT_NAME} easy_profiler)

ament_package()
