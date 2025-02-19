# Set the minimum required version of CMake for this project.
cmake_minimum_required(VERSION 3.8)

# Define the project name.
project(embotech_solver)

# Add compiler options if using GNU C++ or Clang compiler.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17 -Ofast)
endif()

# Optimization flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Ofast -ffast-math")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast -ffast-math")

# Find required packages using CMake's find_package function.
find_package(ament_cmake REQUIRED)
find_package(mpc_solver_common REQUIRED)
find_package(codegenerated_solvers REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

# Set a variable to the path of the Embotech library and specify source files related to the Embotech solver.
set(embotech_lib "${PROJECT_SOURCE_DIR}/../codegenerated_solvers")
set(SOURCES
  ${embotech_lib}/embotech_codegen/embotech/embotech_interface.c
  ${embotech_lib}/embotech_codegen/embotech/embotech_model.c
  ${embotech_lib}/embotech_codegen/embotech/embotech_model.h
  ${embotech_lib}/embotech_codegen/embotech/include/embotech.h
  ${embotech_lib}/embotech_codegen/embotech/include/embotech_memory.h
  ${embotech_lib}/embotech_codegen/embotech/lib_target
)

# Include directories for the Embotech solver and its dependencies.
include_directories(
  include
  include/embotech_solver
  ${embotech_lib}/embotech_codegen/embotech/include
  ${embotech_lib}/embotech_codegen/embotech
  mpc_solver_common/include/mpc_solver_common
)

# Specify directories where the linker can find libraries.
link_directories(
  include
  codegenerated_solvers/embotech_codegen/embotech/lib_target
  ${SOURCES}
)

# Define the embotech_solver library, including its source files.
add_library(
  embotech_solver
  src/embotech_solver.cpp
  ${SOURCES}
)

# Link the embotech_solver library with the mpc_solver_common and codegenerated_solvers packages.
ament_target_dependencies(embotech_solver
  mpc_solver_common
  codegenerated_solvers
  Eigen3
)

# Install the include directory to the installation target.
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Install the embotech_solver library, specifying its destinations.
install(TARGETS embotech_solver
  EXPORT export_embotech_solver
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Export targets and dependencies for other CMake projects to use.
ament_export_targets(export_embotech_solver HAS_LIBRARY_TARGET)
ament_export_dependencies(mpc_solver_common codegenerated_solvers)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

# Conditional block to handle building of tests.
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Add a GTest-based test target.
  ament_add_gtest(${PROJECT_NAME}_test
    test/embotech_solver_test.cpp
  )

  # Specify include directories for the test target.
  target_include_directories(${PROJECT_NAME}_test
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )

  # Link the test target with dependencies.
  ament_target_dependencies(${PROJECT_NAME}_test
    mpc_solver_common
    codegenerated_solvers
  )

  # Link the test executable against the embotech_solver library and its specific static library.
  target_link_libraries(${PROJECT_NAME}_test
    embotech_solver
    ${embotech_lib}/embotech_codegen/embotech/lib_target/libembotech.a
  )
endif()

# Generate and configure ament package files.
ament_package()
