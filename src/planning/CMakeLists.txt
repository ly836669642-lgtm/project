cmake_minimum_required(VERSION 3.8)
project(planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(project_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(route_planner_node src/route_planner_node.cpp)
ament_target_dependencies(route_planner_node
  rclcpp nav_msgs geometry_msgs std_msgs project_interfaces ament_index_cpp)

install(TARGETS route_planner_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY data DESTINATION share/${PROJECT_NAME})

ament_package()
