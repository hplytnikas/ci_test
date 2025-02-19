cmake_minimum_required(VERSION 3.8)
project(interpolant_2d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17 -Ofast)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(interpolant_1d REQUIRED)
find_package(autonomous_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(easy_profiler REQUIRED)

include_directories(include)

link_directories(
  include
  ${EASY_PROFILER_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} SHARED
  src/abstract_interpolant_2d.cpp
  src/bspline_interpolant_2d.cpp
)

target_link_libraries(${PROJECT_NAME}
  easy_profiler
)

ament_target_dependencies(${PROJECT_NAME}
  interpolant_1d
  autonomous_msgs
  Eigen3
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Include directories of own library
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_dependencies(interpolant_1d autonomous_msgs)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
ament_export_include_directories(include)

ament_package()
