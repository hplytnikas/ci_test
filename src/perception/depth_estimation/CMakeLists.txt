project(depth_estimation)

cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(ONNXRUNTIME_DIR $ENV{ONNXRUNTIME_DIR})
if(NOT DEFINED ENV{ONNXRUNTIME_DIR})
    set(ONNXRUNTIME_DIR ~/.amz/onnxruntime-linux-x64-gpu-1.12.1)  # Local + CB
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(autonomous_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(easy_profiler REQUIRED)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/utils.cpp
  src/depth_estimation.cpp
  src/helper.cpp)

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
  ${OpenCV_INCLUDE_DIRS}  # Add OpenCV include directory
  ${PROJECT_SOURCE_DIR}/include  # Add include directory of your package
  ${ONNXRUNTIME_DIR}/include # Add ONNX Runtime include directory
)

include_directories(include ${EASY_PROFILER_INCLUDE_DIR})

# Link libraries
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so
  ${YAML_CPP_LIBRARIES}
  easy_profiler
)

# Dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2_ros
  message_filters
  sensor_msgs
  perception_msgs
  autonomous_msgs
  cv_bridge
  tf2_geometry_msgs
  visualization_msgs
  yaml-cpp
)

# Install target
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
