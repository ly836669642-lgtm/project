#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "rgb_camera_parser.h"
#include "unity_stream_parser.h"

class TrueStateParser : public UnityStreamParser {
public:
  explicit TrueStateParser(rclcpp::Node *node)
      : node_(node), tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*node)) {}

  bool ParseMessage(const UnityHeader &header,
                    TCPStreamReader &stream_reader,
                    double time_offset) override {
    const float px = stream_reader.ReadFloat();
    const float py = stream_reader.ReadFloat();
    const float pz = stream_reader.ReadFloat();

    const float qx = stream_reader.ReadFloat();
    const float qy = stream_reader.ReadFloat();
    const float qz = stream_reader.ReadFloat();
    const float qw = stream_reader.ReadFloat();

    const float vx = stream_reader.ReadFloat();
    const float vy = stream_reader.ReadFloat();
    const float vz = stream_reader.ReadFloat();

    const float rx = stream_reader.ReadFloat();
    const float ry = stream_reader.ReadFloat();
    const float rz = stream_reader.ReadFloat();

    if (pose_publishers_.find(header.name) == pose_publishers_.end()) {
      pose_publishers_[header.name] = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
          header.name + "/pose", rclcpp::QoS(10));
      twist_publishers_[header.name] = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
          header.name + "/twist", rclcpp::QoS(10));
      RCLCPP_INFO(node_->get_logger(), "Publishing true state '%s'", header.name.c_str());
    }

    const double current_stamp = header.timestamp + time_offset;
    if (std::abs(last_time_stamp_ - current_stamp) < 1e-12) {
      return false;
    }
    last_time_stamp_ = current_stamp;

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = "world";
    pose_msg.header.stamp = TimeFromSeconds(last_time_stamp_);
    pose_msg.pose.position.x = pz;
    pose_msg.pose.position.y = -px;
    pose_msg.pose.position.z = py;
    pose_msg.pose.orientation.x = -qz;
    pose_msg.pose.orientation.y = qx;
    pose_msg.pose.orientation.z = -qy;
    pose_msg.pose.orientation.w = qw;

    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header = pose_msg.header;
    twist_msg.twist.linear.x = vz;
    twist_msg.twist.linear.y = -vx;
    twist_msg.twist.linear.z = vy;
    twist_msg.twist.angular.x = rz;
    twist_msg.twist.angular.y = -rx;
    twist_msg.twist.angular.z = -ry;

    pose_publishers_[header.name]->publish(pose_msg);
    twist_publishers_[header.name]->publish(twist_msg);

    geometry_msgs::msg::TransformStamped tf;
    tf.header = pose_msg.header;
    tf.child_frame_id = "OurCar/INS";
    tf.transform.translation.x = pose_msg.pose.position.x;
    tf.transform.translation.y = pose_msg.pose.position.y;
    tf.transform.translation.z = pose_msg.pose.position.z;
    tf.transform.rotation = pose_msg.pose.orientation;
    tf_broadcaster_->sendTransform(tf);

    return true;
  }

private:
  rclcpp::Node *node_;
  std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_publishers_;
  std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr> twist_publishers_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  double last_time_stamp_{0.0};
};
