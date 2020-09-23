cmake_minimum_required(VERSION 3.5)
project(cpp_srvcli)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(tutorial_interfaces REQUIRED)

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server
rclcpp example_interfaces)

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client
  rclcpp example_interfaces)

add_executable(server_three src/add_three_ints_server.cpp)
ament_target_dependencies(server_three rclcpp tutorial_interfaces)

add_executable(client_three src/add_three_ints_client.cpp)
ament_target_dependencies(client_three rclcpp tutorial_interfaces)

install(TARGETS
  server
  client
  server_three
  client_three
  DESTINATION lib/${PROJECT_NAME})

ament_package()