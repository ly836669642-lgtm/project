cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(project_interfaces REQUIRED)
find_package(simulation REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(pure_pursuit_node src/pure_pursuit_node.cpp)
ament_target_dependencies(pure_pursuit_node
  rclcpp geometry_msgs std_msgs std_srvs project_interfaces simulation)

install(TARGETS pure_pursuit_node DESTINATION lib/${PROJECT_NAME})

ament_package()
