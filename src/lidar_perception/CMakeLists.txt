cmake_minimum_required(VERSION 3.8)
project(lidar_perception)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

add_executable(lidar src/lidar_detector_node.cpp)
ament_target_dependencies(lidar rclcpp sensor_msgs pcl_conversions)

install(
  TARGETS
  lidar
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
