cmake_minimum_required(VERSION 3.8)
project(slam_backend)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(GTSAM REQUIRED)  # Uses installed package
find_package(Eigen3 REQUIRED)  # Find Eigen3 package
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(autonomous_msgs REQUIRED)
find_package(slam_common REQUIRED) # Ensure slam_common is found
find_package(easy_profiler REQUIRED)

# Include headers
include_directories(
  include
  ${slam_common_INCLUDE_DIRS} # Ensure slam_common's include directories are added
  ${GTSAM_INCLUDE_DIR} # Add GTSAM includes
  ${EIGEN3_INCLUDE_DIR}  # Add Eigen's include directory
  ${EASY_PROFILER_INCLUDE_DIR}
)

# Declare a shared library
add_library(${PROJECT_NAME} SHARED
  src/slam_backend.cpp
  src/graph.cpp
  src/graph_slam.cpp
  src/fixed_lag_smoothing_backend.cpp
)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  visualization_msgs
  tf2_msgs
  tf2_ros
  tf2
  autonomous_msgs
  slam_common # Ensure dependencies are correctly linked
  GTSAM
  Eigen3
)

target_link_libraries(${PROJECT_NAME}
  ${slam_common_LIBRARIES} # Use the variable to link against the slam_common library
  ${slam_backend_LIBRARIES}
  gtsam
  Eigen3::Eigen
  easy_profiler
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
