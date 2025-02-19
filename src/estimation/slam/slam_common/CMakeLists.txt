cmake_minimum_required(VERSION 3.8)
project(slam_common)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(GTSAM REQUIRED)  # Find GTSAM package
find_package(Eigen3 REQUIRED)  # Find Eigen3 package
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(autonomous_msgs REQUIRED)
find_package(easy_profiler REQUIRED)


add_library(${PROJECT_NAME} SHARED
  src/graph_visualization.cpp
  src/slam_node.cpp
  src/utils.cpp
  src/visualizer.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  visualization_msgs
  tf2_msgs
  tf2_ros
  tf2
  autonomous_msgs
  GTSAM
  Eigen3
)

target_link_libraries(${PROJECT_NAME}
  easy_profiler
)

# Ensure include directories are correctly set
include_directories(
  include
  ${GTSAM_INCLUDE_DIR} # Add GTSAM includes
  ${EIGEN3_INCLUDE_DIR}  # Add Eigen's include directory
  ${EASY_PROFILER_INCLUDE_DIR}
)

# Install the library and headers
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Export the targets
ament_export_targets(${PROJECT_NAME}Targets)

# Export include directories
ament_export_include_directories(include)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
