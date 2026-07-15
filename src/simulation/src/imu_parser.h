#pragma once

#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "rgb_camera_parser.h"
#include "unity_stream_parser.h"

class IMUParser : public UnityStreamParser {
public:
  explicit IMUParser(rclcpp::Node *node) : node_(node) {}

  bool ParseMessage(const UnityHeader &header,
                    TCPStreamReader &stream_reader,
                    double time_offset) override {
    const float ax = stream_reader.ReadFloat();
    const float ay = stream_reader.ReadFloat();
    const float az = stream_reader.ReadFloat();
    const float gx = stream_reader.ReadFloat();
    const float gy = stream_reader.ReadFloat();
    const float gz = stream_reader.ReadFloat();

    if (publishers_.find(header.name) == publishers_.end()) {
      publishers_[header.name] = node_->create_publisher<sensor_msgs::msg::Imu>(
          header.name, rclcpp::SensorDataQoS());
      RCLCPP_INFO(node_->get_logger(), "Publishing IMU '%s'", header.name.c_str());
    }

    sensor_msgs::msg::Imu msg;
    msg.header.frame_id = header.name;
    msg.header.stamp = TimeFromSeconds(header.timestamp + time_offset);
    msg.angular_velocity.x = -gx;
    msg.angular_velocity.y = -gz;
    msg.angular_velocity.z = -gy;
    msg.linear_acceleration.x = ax;
    msg.linear_acceleration.y = az;
    msg.linear_acceleration.z = ay;

    publishers_[header.name]->publish(msg);
    return true;
  }

private:
  rclcpp::Node *node_;
  std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> publishers_;
};
