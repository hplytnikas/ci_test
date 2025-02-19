cmake_minimum_required(VERSION 3.8)
project(abstract_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_node REQUIRED)
find_package(autonomous_msgs REQUIRED)
find_package(vcu_msgs REQUIRED)

add_library(${PROJECT_NAME} src/abstract_controller.cpp)

ament_target_dependencies(${PROJECT_NAME} controller_node autonomous_msgs vcu_msgs)

# Include directories
include_directories(
  include
)
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install library so that it can be found by other packages
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  INCLUDES DESTINATION include/${PROJECT_NAME}
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(controller_node autonomous_msgs vcu_msgs)

ament_package()
