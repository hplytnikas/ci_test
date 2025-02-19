cmake_minimum_required(VERSION 3.8)
project(mpc_geometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17 -Ofast)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(interpolant_2d REQUIRED)
find_package(autonomous_msgs REQUIRED)
find_package(min_curvature_opt REQUIRED)

# ACADOS configuration
set(ACADOS_DIR $ENV{ACADOS_DIR})
if(NOT ACADOS_DIR)
  set(ACADOS_DIR "/usr/local/share/amz/acados")
endif()

set(acados_include "${ACADOS_DIR}/include")
set(acados_lib "${ACADOS_DIR}/lib")

# Include directories
include_directories(
  include
  ${acados_include}
  ${acados_include}/blasfeo/include
  ${acados_include}/hpipm/include
)

# Link directories
link_directories(
  ${acados_lib}
)

# add lib target
add_library(${PROJECT_NAME}
  src/mpc_geometry.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  autonomous_msgs
  interpolant_2d
  min_curvature_opt
)

target_link_libraries(${PROJECT_NAME}
  blasfeo
  hpipm
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Include directories of own library
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
ament_export_include_directories(include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_dependencies(autonomous_msgs interpolant_2d min_curvature_opt)


if(BUILD_TESTING)


  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test
    test/mpc_geometry_test.cpp
  )

  target_include_directories(${PROJECT_NAME}_test
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(${PROJECT_NAME}_test
    autonomous_msgs
  )

  target_link_libraries(${PROJECT_NAME}_test
    mpc_geometry
  )


endif()

ament_package()
