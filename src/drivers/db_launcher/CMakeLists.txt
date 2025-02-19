cmake_minimum_required(VERSION 3.8)
project(db_launcher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vcu_msgs REQUIRED)
find_package(autonomous_msgs REQUIRED)

# Define the executable and its source files
add_executable(${PROJECT_NAME}_node src/db_launcher_node.cpp src/db_launcher_node_main.cpp)
# list its dendencies
ament_target_dependencies(${PROJECT_NAME}_node  vcu_msgs std_msgs autonomous_msgs rclcpp)

# Include directories of own library
include_directories(
  include
)

# Expose headers to other libraries and executables
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install the executable so it can be run from the command line
install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

# Install the launch files and config files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
