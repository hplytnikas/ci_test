cmake_minimum_required(VERSION 3.8)
project(amzsim_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(autonomous_msgs REQUIRED)
find_package(amzsim_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)


include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
)

include_directories(include)

add_executable(fake_perception
  src/perception_faked.cpp
  src/perception_faked_node.cpp)

ament_target_dependencies(fake_perception rclcpp tf2_ros geometry_msgs tf2_geometry_msgs ament_index_cpp visualization_msgs autonomous_msgs amzsim_msgs)

install(TARGETS
  fake_perception
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_package()
