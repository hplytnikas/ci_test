cmake_minimum_required(VERSION 3.8)
project(amzsim_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(amzsim_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(state_publisher
  src/simulator_node.cpp
  src/simulator.cpp)

ament_target_dependencies(state_publisher rclcpp amzsim_msgs tf2_ros tf2_geometry_msgs nav_msgs)

install(TARGETS
  state_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_package()
