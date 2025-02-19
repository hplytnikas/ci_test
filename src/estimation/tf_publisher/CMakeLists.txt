cmake_minimum_required(VERSION 3.5)
project(tf_publisher)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vcu_msgs REQUIRED) #Custom message package
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)


# Include directories
include_directories(include ${EIGEN3_INCLUDE_DIRS})

# Declare a C++ executable
add_executable(tf_publisher src/tf_publisher.cpp)
ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  vcu_msgs
  tf2
  tf2_ros
  Eigen3
  tf2_geometry_msgs
)

# Install the executable
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Export dependencies
ament_export_dependencies(Eigen3)
ament_export_include_directories(include ${EIGEN3_INCLUDE_DIRS})
ament_package()
