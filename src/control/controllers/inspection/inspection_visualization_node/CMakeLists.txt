cmake_minimum_required(VERSION 3.8)
project(inspection_visualization_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autonomous_msgs REQUIRED)

add_library(${PROJECT_NAME} src/inspection_visualization_node.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp autonomous_msgs)

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
ament_export_dependencies(rclcpp autonomous_msgs)

ament_package()
