cmake_minimum_required(VERSION 3.8)
project(mpc_solver_common)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17 -Ofast)
endif()

# Optimization flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Ofast -ffast-math")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast -ffast-math")

# find dependencies
find_package(ament_cmake REQUIRED)

add_library(mpc_solver INTERFACE)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS mpc_solver
  EXPORT export_mpc_solver
  INCLUDES DESTINATION include
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_mpc_solver)

ament_package()
