cmake_minimum_required(VERSION 3.8)
project(vcu_comm_interface)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autonomous_msgs REQUIRED)
find_package(vcu_msgs REQUIRED)

include_directories(include
                   ${boost_INCLUDE_DIRS})

add_library(udp_socket
            src/socket.cpp)

ament_target_dependencies(udp_socket rclcpp)

add_executable(receiver_node src/receiver_node.cpp src/receiver.cpp)

ament_target_dependencies(receiver_node rclcpp autonomous_msgs vcu_msgs)

target_link_libraries(receiver_node
                      ${Boost_LIBRARIES}
                      ${boost_LIBRARIES}
                      udp_socket)

add_executable(sender_node src/sender_node.cpp src/sender.cpp)

ament_target_dependencies(sender_node rclcpp autonomous_msgs vcu_msgs)

target_link_libraries(sender_node
                    ${Boost_LIBRARIES}
                    ${boost_LIBRARIES}
                    udp_socket)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS receiver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS sender_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
