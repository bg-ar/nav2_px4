cmake_minimum_required(VERSION 3.8)
project(nav2_px4_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(nav2_px4_bridge_node src/nav2_px4_bridge_node.cpp)
ament_target_dependencies(nav2_px4_bridge_node
  rclcpp px4_msgs geometry_msgs std_srvs)

install(TARGETS nav2_px4_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
