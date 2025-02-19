cmake_minimum_required(VERSION 3.8)
project(pure_pursuit_visualization_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(autonomous_msgs REQUIRED)
find_package(controller_node REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_node REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(easy_profiler REQUIRED)

# Define library target
add_library(${PROJECT_NAME} src/pure_pursuit_visualization_node.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp geometry_msgs autonomous_msgs nav_msgs control_msgs visualization_msgs std_msgs easy_profiler controller_node)

#This adds all files in the folder ${CMAKE_CURRENT_SOURCE_DIR}/include to the public interface during build time and all files in the include folder
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

# Include directories of own library
include_directories(
  include
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
ament_export_dependencies(rclcpp geometry_msgs autonomous_msgs nav_msgs control_msgs visualization_msgs std_msgs controller_node)

target_link_libraries(${PROJECT_NAME} easy_profiler)

target_link_libraries(${PROJECT_NAME} easy_profiler)

ament_package()
