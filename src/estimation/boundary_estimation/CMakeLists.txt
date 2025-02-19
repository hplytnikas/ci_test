cmake_minimum_required(VERSION 3.8)

project(boundary_estimation)

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(autonomous_msgs REQUIRED)
find_package(CGAL REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(easy_profiler REQUIRED)
find_package(message_filters REQUIRED)


# Set the include directories
include_directories(
   include
   ${EIGEN3_INCLUDE_DIRS}
   ${CGAL_INCLUDE_DIRS}
   ${EASY_PROFILER_INCLUDE_DIR}
)

# Each node in the package must be declared like this
add_executable(${PROJECT_NAME}
   src/helpers.cpp
   src/node_handle.cpp
   src/boundary_estimation_structs.cpp
   src/boundary_estimation_visualization.cpp
   src/boundary_estimation_graph.cpp
   src/boundary_estimation_backend.cpp
   src/boundary_estimation_frontend.cpp
   src/main.cpp
   )


ament_target_dependencies(${PROJECT_NAME}
   rclcpp
   std_msgs
   tf2_msgs
   tf2_ros
   tf2_geometry_msgs
   autonomous_msgs
   visualization_msgs
   message_filters
)

# Link against CGAL libraries
target_link_libraries(${PROJECT_NAME}
   ${CGAL_LIBRARIES}
   easy_profiler
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
   ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
