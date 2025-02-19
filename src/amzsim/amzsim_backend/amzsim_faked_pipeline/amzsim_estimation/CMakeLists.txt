cmake_minimum_required(VERSION 3.8)
project(amzsim_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autonomous_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(amzsim_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
)

include_directories(include)

add_executable(fake_estimation
  src/tf2_map_odom.cpp
  src/tf2_map_odom_node.cpp)

ament_target_dependencies(fake_estimation rclcpp tf2_ros geometry_msgs tf2_geometry_msgs amzsim_msgs autonomous_msgs)

install(TARGETS
  fake_estimation
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_package()
