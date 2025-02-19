cmake_minimum_required(VERSION 3.8)
project(amzsim_codegen_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(amzsim_msgs REQUIRED)
find_package(vcu_msgs REQUIRED)
find_package(autonomous_msgs REQUIRED)

include_directories(include)
include_directories(codegen_vehicle_model/dufour/vcu_codegen_ert_rtw)
include_directories(codegen_vehicle_model/dufour/vehicle_codegen_ert_rtw)

add_library(my_library
  codegen_vehicle_model/dufour/vcu_codegen_ert_rtw/vcu_codegen_data_dufour.cpp
  codegen_vehicle_model/dufour/vcu_codegen_ert_rtw/vcu_codegen_dufour.cpp
  codegen_vehicle_model/dufour/vcu_codegen_ert_rtw/vcu_codegen_dufour.h
  codegen_vehicle_model/dufour/vehicle_codegen_ert_rtw/vehicle_codegen_data_dufour.cpp
  codegen_vehicle_model/dufour/vehicle_codegen_ert_rtw/vehicle_codegen_dufour.cpp
  codegen_vehicle_model/dufour/vehicle_codegen_ert_rtw/vehicle_codegen_dufour.h
  codegen_vehicle_model/dufour/vcu_codegen_ert_rtw/rt_nonfinite.cpp
  codegen_vehicle_model/dufour/vcu_codegen_ert_rtw/rtGetInf.cpp
  codegen_vehicle_model/dufour/vcu_codegen_ert_rtw/rtGetNaN.cpp
)

add_executable(model
  src/amzsim_model.cpp
  src/amzsim_model_node.cpp)

ament_target_dependencies(model rclcpp std_msgs amzsim_msgs vcu_msgs autonomous_msgs)

target_link_libraries(model my_library)

install(TARGETS
  model
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_package()
