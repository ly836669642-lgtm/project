cmake_minimum_required(VERSION 3.8)
project(simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleControl.msg"
  DEPENDENCIES std_msgs
)

include_directories(
  src
  tcpimage
)

add_library(tcpstreamreader STATIC
  tcpimage/TCPStreamReader.cpp
)
target_include_directories(tcpstreamreader PUBLIC tcpimage)

add_library(tcpimage STATIC
  tcpimage/TCPImageServer.cpp
)
target_link_libraries(tcpimage tcpstreamreader)


add_executable(unity_TCP_stream_receiver src/unity_TCP_stream_receiver.cpp)
ament_target_dependencies(unity_TCP_stream_receiver
  rclcpp sensor_msgs geometry_msgs tf2_ros ament_index_cpp
)
target_link_libraries(unity_TCP_stream_receiver tcpstreamreader tcpimage)

add_executable(ROS_command_transmitter src/ROS_command_transmitter.cpp)
ament_target_dependencies(ROS_command_transmitter rclcpp ament_index_cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ROS_command_transmitter "${cpp_typesupport_target}")

add_executable(JSON_param_reader src/JSON_param_reader.cpp)
ament_target_dependencies(JSON_param_reader rclcpp ament_index_cpp)

install(TARGETS
  unity_TCP_stream_receiver
  ROS_command_transmitter
  JSON_param_reader
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  msg
  rviz
  scripts
  test_scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
