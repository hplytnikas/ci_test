cmake_minimum_required(VERSION 3.8)
project(acados_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17 -Ofast)
endif()

# Optimization flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Ofast -ffast-math")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast -ffast-math")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(mpc_solver_common REQUIRED)
find_package(codegenerated_solvers REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

set(ACADOS_DIR $ENV{ACADOS_DIR})
if(NOT DEFINED ENV{ACADOS_DIR})
  set(ACADOS_DIR /usr/local/share/amz/acados)
endif()

set(acados_include "${ACADOS_DIR}/include")
set(acados_lib "${ACADOS_DIR}/lib")
set(full_model_build "${PROJECT_SOURCE_DIR}/../codegenerated_solvers/acados_codegen/c_generated_solver_mpc")

# object target names
set(MODEL_OBJ model_veh_dynamics_ode)
set(OCP_OBJ ocp_veh_dynamics_ode)

include_directories(include
  ${full_model_build}
  ${acados_include}
  ${acados_include}/blasfeo/include
  ${acados_include}/hpipm/include
  ${acados_include}/acados
  ${acados_include}/qpOASES_e
)

# ========================= NLP ======================
# NLP model
file(GLOB MODEL_SRC_FILES ${full_model_build}/veh_dynamics_ode_model/*.c)
set(MODEL_SRC
  ${MODEL_SRC_FILES}
)
add_library(${MODEL_OBJ} OBJECT ${MODEL_SRC} )

# NLP optimal control problem - mostly CasADi exports
file(GLOB OCP_COST_SRC_FILES ${full_model_build}/veh_dynamics_ode_cost/*.c)
file(GLOB OCP_CONSTRAINTS_SRC_FILES ${full_model_build}/veh_dynamics_ode_constraints/*.c)
set(OCP_SRC
  ${OCP_COST_SRC_FILES}
  ${OCP_CONSTRAINTS_SRC_FILES}
  ${full_model_build}/acados_solver_veh_dynamics_ode.c
)
add_library(${OCP_OBJ} OBJECT ${OCP_SRC})

# ===============

link_directories(include
  ${acados_lib}
  ${acados_include}
  ${full_model_build}
)

link_libraries(acados hpipm blasfeo m )

add_library(acados_solver
  src/acados_solver.cpp
  $<TARGET_OBJECTS:${MODEL_OBJ}>
  $<TARGET_OBJECTS:${OCP_OBJ}>
)

ament_target_dependencies(acados_solver
  mpc_solver_common
  codegenerated_solvers
  Eigen3
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Export library target
install(TARGETS acados_solver
  EXPORT export_acados_solver
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
ament_export_targets(export_acados_solver HAS_LIBRARY_TARGET)
ament_export_dependencies(mpc_solver_common codegenerated_solvers)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test
    test/acados_solver_test.cpp
  )

  target_include_directories(${PROJECT_NAME}_test
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(${PROJECT_NAME}_test
    mpc_solver_common
    codegenerated_solvers
  )

  target_link_libraries(${PROJECT_NAME}_test
    acados_solver
    acados
    hpipm
    blasfeo
    m
  )

endif()

ament_package()
