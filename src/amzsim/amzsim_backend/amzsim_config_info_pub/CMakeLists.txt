cmake_minimum_required(VERSION 3.8)
project(amzsim_config_info_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(amzsim_msgs REQUIRED)

include_directories(include)

add_executable(config
  src/amzsim_config_info_pub.cpp
  src/amzsim_config_info_pub_node.cpp)

ament_target_dependencies(config rclcpp amzsim_msgs)

install(TARGETS
  config
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
