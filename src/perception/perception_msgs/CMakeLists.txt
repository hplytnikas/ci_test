project(perception_msgs)

cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autonomous_msgs REQUIRED)

# Define message and service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoundingBox.msg"
  "msg/BoundingBoxDebug.msg"
  "msg/BoxArray.msg"
  "msg/BoxArrayDebug.msg"
  "msg/PipelineRuntime.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs autonomous_msgs
  )

ament_export_dependencies(rosidl_default_runtime)

ament_package()
