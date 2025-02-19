cmake_minimum_required(VERSION 3.8)
project(boundary_to_reference)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")

  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(autonomous_msgs REQUIRED)

# Define the executable and its source files
add_executable(${PROJECT_NAME}_node src/boundary_to_reference_node.cpp)
# list its dendencies
ament_target_dependencies(${PROJECT_NAME}_node  std_msgs geometry_msgs control_msgs autonomous_msgs rclcpp)

# Include directories of own library
include_directories(
  include
)

# Expose headers to other libraries and executables
# install(
#   DIRECTORY include/
#   DESTINATION include/${PROJECT_NAME}
# )

# Install the executable so it can be run from the command line
install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

# Install the launch files and config files
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
