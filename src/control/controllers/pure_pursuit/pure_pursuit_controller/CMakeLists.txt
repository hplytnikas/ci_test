cmake_minimum_required(VERSION 3.8)
project(pure_pursuit_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(abstract_controller REQUIRED)
find_package(controller_node REQUIRED)
find_package(pure_pursuit_visualization_node REQUIRED)
find_package(autonomous_msgs REQUIRED)
find_package(vcu_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(easy_profiler REQUIRED)

set(ACADOS_DIR $ENV{ACADOS_DIR})
if(NOT DEFINED ENV{ACADOS_DIR})
  set(ACADOS_DIR /usr/local/share/amz/acados)
endif()

set(acados_include "${ACADOS_DIR}/include")
set(acados_lib "${ACADOS_DIR}/lib")

generate_parameter_library(
  pure_pursuit_controller_params
  param_definition/pure_pursuit_controller_params.yaml
)

# Define the executable and its source files
add_executable(${PROJECT_NAME} src/pure_pursuit_controller.cpp src/pure_pursuit_controller_main.cpp)
# list its dendencies
ament_target_dependencies(${PROJECT_NAME}  controller_node pure_pursuit_visualization_node autonomous_msgs vcu_msgs geometry_msgs abstract_controller control_msgs)

# Include directories of own library
target_include_directories(${PROJECT_NAME}
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${acados_include}
    ${acados_include}/blasfeo/include
    ${acados_include}/hpipm/include
)

# Assume linking is needed only for the executable in this context
find_library(HPIPM_LIBRARY NAMES hpipm PATHS ${acados_lib} NO_DEFAULT_PATH)
find_library(BLASFEO_LIBRARY NAMES blasfeo PATHS ${acados_lib} NO_DEFAULT_PATH)

target_link_libraries(${PROJECT_NAME}
  ${HPIPM_LIBRARY}
  ${BLASFEO_LIBRARY}
  pure_pursuit_controller_params
  easy_profiler
)

target_link_directories(${PROJECT_NAME}
  PRIVATE
    ${acados_lib}
)

# Expose headers to other libraries and executables
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Install the executable so it can be run from the command line
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# Install the launch files and config files
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
