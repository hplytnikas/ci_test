cmake_minimum_required(VERSION 3.8)
project(amzsim_fakecontrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(amzsim_msgs REQUIRED)
find_package(vcu_msgs REQUIRED)
find_package(autonomous_msgs REQUIRED)

include_directories(include)

add_executable(talker
  src/control_publisher_node.cpp
  src/control_publisher.cpp)

ament_target_dependencies(talker rclcpp amzsim_msgs vcu_msgs autonomous_msgs)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

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
