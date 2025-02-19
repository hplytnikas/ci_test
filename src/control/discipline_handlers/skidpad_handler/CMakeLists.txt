cmake_minimum_required(VERSION 3.8)
project(skidpad_handler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vcu_msgs REQUIRED)
find_package(autonomous_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(control_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(
  skidpad_handler_params
  param_definition/skidpad_handler_params.yaml
)

# Define the executable and its source files
add_executable(${PROJECT_NAME}_node src/skidpad_handler_node.cpp)
# list its dendencies
ament_target_dependencies(${PROJECT_NAME}_node  vcu_msgs std_msgs autonomous_msgs geometry_msgs tf2_ros tf2 tf2_geometry_msgs control_msgs)
# Use target_link_libraries for the locally generated target
target_link_libraries(${PROJECT_NAME}_node skidpad_handler_params)

include_directories(
  include
  /usr/local/share/amz/fast-cpp-csv-parser
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
  DIRECTORY launch config trajectory
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
