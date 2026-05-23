cmake_minimum_required(VERSION 3.8)
project(localization)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interfaces REQUIRED)

add_executable(ekf src/ekf_node.cpp)
ament_target_dependencies(ekf rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros)
target_link_libraries(ekf Eigen3::Eigen)

add_executable(slam src/graph_slam_node.cpp)
ament_target_dependencies(slam rclcpp nav_msgs visualization_msgs interfaces message_filters)
target_link_libraries(slam Eigen3::Eigen)

add_executable(viz src/slam_visualization.cpp)
ament_target_dependencies(viz rclcpp geometry_msgs nav_msgs sensor_msgs interfaces)

install(
  TARGETS
  ekf
  slam
  viz
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
