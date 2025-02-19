cmake_minimum_required(VERSION 3.8)
project(slam_frontend)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
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
find_package(slam_common REQUIRED)
find_package(slam_backend REQUIRED)
find_package(easy_profiler REQUIRED)


# Include headers
include_directories(
  include
  ${slam_common_INCLUDE_DIRS} # Ensure slam_common's include directories are added
  ${slam_backend_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR} # Add GTSAM includes
  ${EIGEN3_INCLUDE_DIR}  # Add Eigen's include directory
  ${EASY_PROFILER_INCLUDE_DIR}
)

# Declare an executable
add_executable(${PROJECT_NAME}
  src/data_association.cpp
  src/global_map_tracker.cpp
  src/map_loader.cpp
  src/nn_data_association.cpp
  src/slam_frontend.cpp
  src/main.cpp
)

# Specify libraries to link an executable target against
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  visualization_msgs
  tf2_msgs
  tf2_ros
  tf2
  autonomous_msgs
  slam_common
  slam_backend
  GTSAM
  Eigen3
)

# Link against slam_common
# Make sure slam_common is built and sourced before slam_backend
target_link_libraries(${PROJECT_NAME}
  ${slam_common_LIBRARIES} # Use the variable to link against the slam_common library
  ${slam_backend_LIBRARIES}
  gtsam
  Eigen3::Eigen
  easy_profiler
)

# Install the executable
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and other directories
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
