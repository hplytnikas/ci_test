set(CMAKE_CXX_STANDARD 17)
cmake_minimum_required(VERSION 3.5)
project(autonomous_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Define message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoolStamped.msg"
  "msg/Boundary.msg"
  "msg/Cone.msg"
  "msg/ConeArray.msg"
  "msg/ConeTypeProb.msg"
  "msg/Covariance2D.msg"
  "msg/CovariancePose.msg"
  "msg/DoubleStamped.msg"
  "msg/PointWithConfidence.msg"
  "msg/IntStamped.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
