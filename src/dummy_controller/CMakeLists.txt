cmake_minimum_required(VERSION 3.8)
project(dummy_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(simulation REQUIRED)

add_executable(dummy_controller_node src/dummy_controller_node.cpp)
ament_target_dependencies(dummy_controller_node
  rclcpp
  simulation
)

install(TARGETS
  dummy_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
