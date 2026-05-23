cmake_minimum_required(VERSION 3.8)
project(path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(OsqpEigen REQUIRED)

add_executable(state_machine_node src/state_machine_node.cpp)
ament_target_dependencies(state_machine_node
  rclcpp
  nav_msgs
  std_msgs
  visualization_msgs
  carla_msgs
  interfaces
)
target_link_libraries(state_machine_node
  Eigen3::Eigen
  OsqpEigen::OsqpEigen
)

install(
  TARGETS
  state_machine_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
