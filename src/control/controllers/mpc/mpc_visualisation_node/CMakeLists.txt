cmake_minimum_required(VERSION 3.8)
project(mpc_visualisation_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17 -Ofast)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

include_directories( include ${PCL_INCLUDE_DIRS} )

# Define library target
add_library(${PROJECT_NAME} src/mpc_visualisation_node.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  visualization_msgs
  geometry_msgs
  sensor_msgs
  control_msgs
  tf2_geometry_msgs
  rcutils
  nav_msgs
  rcl_interfaces
  Eigen3
)

#This adds all files in the folder ${CMAKE_CURRENT_SOURCE_DIR}/include to the public interface during build time and all files in the include folder
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
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
ament_export_dependencies(rclcpp visualization_msgs geometry_msgs sensor_msgs control_msgs tf2_geometry_msgs rcutils rcl_interfaces nav_msgs)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

ament_package()
