project(static_tf_publisher)

cmake_minimum_required(VERSION 3.8)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(${PROJECT_NAME} src/static_transform_publisher.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  yaml-cpp
)

target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
