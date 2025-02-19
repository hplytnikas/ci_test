cmake_minimum_required(VERSION 3.8)
project(lap_counter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vcu_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


add_executable(lap_counter src/lap_counter.cpp)
ament_target_dependencies(lap_counter rclcpp std_msgs geometry_msgs tf2_ros vcu_msgs visualization_msgs)

include_directories(include)

install(TARGETS
  lap_counter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
launch
config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
