#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "TCPImageServer.h"
#include "unity_stream_parser.h"

inline builtin_interfaces::msg::Time TimeFromSeconds(double seconds) {
  builtin_interfaces::msg::Time stamp;
  const int64_t nanoseconds = static_cast<int64_t>(seconds * 1e9);
  stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
  return stamp;
}

class RGBCameraParser : public UnityStreamParser {
public:
  explicit RGBCameraParser(rclcpp::Node *node) : node_(node) {}

  bool ParseMessage(const UnityHeader &header,
                    TCPStreamReader &stream_reader,
                    double time_offset) override {
    auto img = ParseImage(stream_reader);
    PublishImage(header, img, time_offset);
    return true;
  }

  float GetFieldOfView() const { return fov_; }
  float GetMaxRange() const { return clip_plane_; }

protected:
  virtual void PublishImage(const UnityHeader &header, const ImageData &img, double time_offset) {
    if (image_publishers_.find(header.name) == image_publishers_.end()) {
      image_publishers_[header.name] = node_->create_publisher<sensor_msgs::msg::Image>(
          header.name + "/image_raw", rclcpp::SensorDataQoS());
      info_publishers_[header.name] = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
          header.name + "/camera_info", rclcpp::SensorDataQoS());
      RCLCPP_INFO(node_->get_logger(), "Publishing camera '%s'", header.name.c_str());
    }

    sensor_msgs::msg::Image img_msg;
    sensor_msgs::msg::CameraInfo info_msg;
    ImageToRos(img, header.name, &img_msg);
    img_msg.header.stamp = TimeFromSeconds(header.timestamp + time_offset);
    BuildCameraInfoMessage(img_msg, info_msg);

    image_publishers_[header.name]->publish(img_msg);
    info_publishers_[header.name]->publish(info_msg);
  }

  virtual void ImageToRos(const ImageData &img,
                          const std::string &frame,
                          sensor_msgs::msg::Image *img_msg) const {
    img_msg->width = static_cast<uint32_t>(img.width);
    img_msg->height = static_cast<uint32_t>(img.height);
    img_msg->header.frame_id = frame;

    if (monochrome_) {
      img_msg->encoding = "mono8";
      img_msg->step = static_cast<uint32_t>(img.width);
      img_msg->data.resize(static_cast<size_t>(img.width) * static_cast<size_t>(img.height));
      const auto *src = img.data.get();
      for (int i = 0; i < img.width * img.height; ++i) {
        const float r = static_cast<float>(src[3 * i]);
        const float g = static_cast<float>(src[3 * i + 1]);
        const float b = static_cast<float>(src[3 * i + 2]);
        img_msg->data[i] = static_cast<uint8_t>(0.299f * r + 0.587f * g + 0.114f * b);
      }
    } else {
      img_msg->encoding = "rgb8";
      img_msg->step = static_cast<uint32_t>(img.width * 3);
      img_msg->data.assign(img.data.get(), img.data.get() + static_cast<size_t>(img.height) * img_msg->step);
    }
  }

  virtual void BuildCameraInfoMessage(const sensor_msgs::msg::Image &img_msg,
                                      sensor_msgs::msg::CameraInfo &info_msg) {
    info_msg.width = img_msg.width;
    info_msg.height = img_msg.height;
    info_msg.distortion_model = "plumb_bob";
    info_msg.header = img_msg.header;

    const double cx = static_cast<double>(img_msg.width) * 0.5;
    const double cy = static_cast<double>(img_msg.height) * 0.5;
    const double fx = 0.5 * static_cast<double>(img_msg.height) / std::tan(static_cast<double>(fov_) * M_PI / 360.0);
    const double fy = fx;

    info_msg.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    info_msg.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info_msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    info_msg.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  }

  ImageData ParseImage(TCPStreamReader &stream_reader) {
    if (!image_server_) {
      image_server_ = std::make_unique<TCPImageServer>(&stream_reader, true);
    }
    monochrome_ = stream_reader.ReadInt() == 1;
    fov_ = stream_reader.ReadFloat();
    clip_plane_ = stream_reader.ReadFloat();
    return image_server_->GetImage();
  }

  rclcpp::Node *node_;
  std::unique_ptr<TCPImageServer> image_server_;
  float fov_{60.0f};
  float clip_plane_{1000.0f};
  bool monochrome_{false};

private:
  std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_publishers_;
  std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> info_publishers_;
};
