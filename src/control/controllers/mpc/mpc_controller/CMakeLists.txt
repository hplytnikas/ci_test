# Set the minimum CMake version required and the name of the project.
cmake_minimum_required(VERSION 3.8)
project(mpc_controller)

# Apply compiler options for GNU and Clang compilers to ensure strict compliance with code standards.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Ofast -ffast-math")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast -ffast-math")

# Find all the package dependencies required by the project using CMake's find_package function.
find_package(ament_cmake REQUIRED)
find_package(mpc_geometry REQUIRED)
find_package(abstract_controller REQUIRED)
find_package(autonomous_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(embotech_solver REQUIRED)
find_package(acados_solver REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(mpc_visualisation_node REQUIRED)
find_package(easy_profiler REQUIRED)

generate_parameter_library(
  mpc_controller_params
  param_definition/mpc_controller_params.yaml
)

add_executable(${PROJECT_NAME}
  src/mpc_controller.cpp
  src/mpc_controller_main.cpp
)

set(ACADOS_DIR $ENV{ACADOS_DIR})
if(NOT DEFINED ENV{ACADOS_DIR})
  set(ACADOS_DIR /usr/local/share/amz/acados)
endif()

# Set variables pointing to the acados and embotech library directories.
set(acados_lib_utils "${ACADOS_DIR}")
set(embotech_lib "${PROJECT_SOURCE_DIR}/../solvers/codegenerated_solvers/embotech_codegen/embotech/lib_target")
set(embotech_inc "${PROJECT_SOURCE_DIR}/../solvers/codegenerated_solvers/embotech_codegen/embotech/include")

# Link the project library with its dependencies.
ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
    abstract_controller
    controller_node
    mpc_visualisation_node
    autonomous_msgs
    control_msgs
    vcu_msgs
    geometry_msgs
    embotech_solver
    acados_solver
    mpc_geometry
)

# Specify include directories for the target, distinguishing between build and install interfaces.
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${embotech_inc}
    ${acados_lib_utils}/include
    ${acados_lib_utils}/include/blasfeo/include
    ${acados_lib_utils}/include/hpipm/include
    ${EASY_PROFILER_INCLUDE_DIR}
)

# Link the project library with external libraries.
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${embotech_lib}/libembotech.a
    ${acados_lib_utils}/lib/libacados.so
    ${acados_lib_utils}/lib/libhpipm.so
    ${acados_lib_utils}/lib/libblasfeo.so
    mpc_controller_params
    ${Boost_SYSTEM_LIBRARY}
    easy_profiler
)

target_link_directories(${PROJECT_NAME}
  PUBLIC
    ${acados_lib_utils}/lib
    ${embotech_lib}
  )

# Install the project's include directory to the install interface.
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install the executable so it can be run from the command line
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch files and config files
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Finalize the ament package configuration.
ament_package()
