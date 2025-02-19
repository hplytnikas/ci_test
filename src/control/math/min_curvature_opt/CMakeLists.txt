cmake_minimum_required(VERSION 3.8)
project(min_curvature_opt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native -Ofast -ffast-math")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Ofast -ffast-math")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Ofast -ffast-math")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast -ffast-math")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(easy_profiler REQUIRED)
find_package(interpolant_2d REQUIRED)

set(ACADOS_DIR $ENV{ACADOS_DIR})
if(NOT DEFINED ENV{ACADOS_DIR})
  set(ACADOS_DIR /usr/local/share/amz/acados)
endif()

set(acados_include "${ACADOS_DIR}/include")
set(acados_lib "${ACADOS_DIR}/lib")

include_directories(include
  ${acados_include}
  ${acados_include}/blasfeo/include
  ${acados_include}/hpipm/include
)

link_directories(
  include
  ${EASY_PROFILER_INCLUDE_DIR}
  ${acados_lib}
  ${acados_include}
)

add_library(${PROJECT_NAME} SHARED
  src/min_curvature_opt.cpp
)

add_library(dataset SHARED
  test/dataset.cpp
)

target_link_libraries(${PROJECT_NAME}
  easy_profiler
  hpipm
  blasfeo
)

target_link_libraries(dataset
  easy_profiler
  Eigen3::Eigen
  Python3::Python
  Python3::Module
)

ament_target_dependencies(${PROJECT_NAME}
  Eigen3
  interpolant_2d
)

ament_target_dependencies(dataset
  Eigen3
  interpolant_2d
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Include directories of own library
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_dependencies(
  eigen3_cmake_module
  Eigen3
  easy_profiler
  interpolant_2d
)
ament_export_include_directories(include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 COMPONENTS Development REQUIRED)
  find_package(Eigen3 3.3 REQUIRED)
  find_package(interpolant_2d REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test
    test/min_curvature_opt_test.cpp
  )

  set_tests_properties(${PROJECT_NAME}_test PROPERTIES TIMEOUT 3000)

  target_include_directories(${PROJECT_NAME}_test
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      ${Eigen3_INCLUDE_DIRS}
      ${Python3_INCLUDE_DIRS}
  )

  target_link_libraries(${PROJECT_NAME}_test
    min_curvature_opt
    dataset
    hpipm
    blasfeo
    Eigen3::Eigen
    Python3::Python
    Python3::Module
  )

endif()

ament_package()
