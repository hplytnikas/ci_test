project(cone_fusion)

cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(autonomous_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(easy_profiler REQUIRED)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/cone_fusion.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${PROJECT_SOURCE_DIR}/include  # Add include directory of your package
)

include_directories(include ${EASY_PROFILER_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME}
  easy_profiler
)

install(TARGETS cone_fusion
  DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  perception_msgs
  autonomous_msgs
  message_filters
  visualization_msgs
)

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
