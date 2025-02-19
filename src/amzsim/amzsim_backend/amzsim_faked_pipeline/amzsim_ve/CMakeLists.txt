cmake_minimum_required(VERSION 3.8)
project(amzsim_ve)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(autonomous_msgs REQUIRED)
find_package(vcu_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(amzsim_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(steering
  src/steering.cpp)

add_executable(velocity_est
  src/velocity_estimation.cpp)

add_executable(sensor_tf
  src/tf2_odom_base_link.cpp)

ament_target_dependencies(sensor_tf rclcpp tf2_ros geometry_msgs amzsim_msgs tf2_geometry_msgs)
ament_target_dependencies(steering rclcpp autonomous_msgs vcu_msgs)
ament_target_dependencies(velocity_est rclcpp vcu_msgs)

install(TARGETS
  sensor_tf
  steering
  velocity_est
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rclcpp)
ament_package()
