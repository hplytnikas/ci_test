set(CMAKE_CXX_STANDARD 17)
cmake_minimum_required(VERSION 3.5)
project(monitoring_pipelines)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autonomous_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vcu_msgs REQUIRED)
find_package(monitoring_msgs REQUIRED)

# Include directories
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS})

# Add the executable
add_executable(${PROJECT_NAME}_node
  src/monitoring_pipelines_main.cpp
  src/monitoring_pipelines_node.cpp
)

# Specify ament dependencies for the executable
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs autonomous_msgs sensor_msgs vcu_msgs monitoring_msgs)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install other directories
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Config Directory
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Export package dependencies
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs autonomous_msgs sensor_msgs vcu_msgs monitoring_msgs)

# Call ament_package
ament_package()
