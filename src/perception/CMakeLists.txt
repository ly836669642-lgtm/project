cmake_minimum_required(VERSION 3.8)
project(perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(project_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_executable(obstacle_guard_node src/obstacle_guard_node.cpp)
ament_target_dependencies(obstacle_guard_node
  rclcpp sensor_msgs std_msgs geometry_msgs tf2_ros project_interfaces)

add_executable(traffic_light_node src/traffic_light_node.cpp)
ament_target_dependencies(traffic_light_node
  rclcpp sensor_msgs project_interfaces cv_bridge)
target_link_libraries(traffic_light_node opencv_core opencv_imgproc)

install(TARGETS obstacle_guard_node traffic_light_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
