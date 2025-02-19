cmake_minimum_required(VERSION 3.8)
project(amzsim_tracks_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(amzsim_utils REQUIRED)
find_package(ament_index_cpp REQUIRED)

install(DIRECTORY
  launch
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

include_directories(include)

add_executable(tracks_visualization_node
 src/tracks_visualization_node.cpp
 src/tracks_visualization.cpp
)

ament_target_dependencies(tracks_visualization_node rclcpp visualization_msgs amzsim_utils ament_index_cpp)

install(TARGETS
  tracks_visualization_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)

ament_package()
