cmake_minimum_required(VERSION 3.8)
project(inspection_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(abstract_controller REQUIRED)
find_package(controller_node REQUIRED)
find_package(autonomous_msgs REQUIRED)
find_package(vcu_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(inspection_visualization_node REQUIRED)

generate_parameter_library(
  inspection_controller_params
  src/inspection_controller_params.yaml
)

set(ACADOS_DIR $ENV{ACADOS_DIR})
if(NOT DEFINED ENV{ACADOS_DIR})
  set(ACADOS_DIR /usr/local/share/amz/acados)
endif()

set(acados_include "${ACADOS_DIR}/include")
set(acados_lib "${ACADOS_DIR}/lib")

# Define the executable and its source files
add_executable(${PROJECT_NAME} src/inspection_controller.cpp src/inspection_controller_main.cpp)
# List its dendencies
ament_target_dependencies(${PROJECT_NAME}  controller_node inspection_visualization_node autonomous_msgs vcu_msgs geometry_msgs abstract_controller)
# Use target_link_libraries for the locally generated target

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
  inspection_controller_params
)

target_link_directories(${PROJECT_NAME}
  PRIVATE
    ${acados_lib}
)

# Include directories of own library
include_directories(
  include
)

# Expose headers to other libraries and executables
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

ament_package()
