#pragma once

#include <cstdint>
#include <vector>
#include "rgb_camera_parser.h"

class DepthCameraParser : public RGBCameraParser {
public:
  explicit DepthCameraParser(rclcpp::Node *node) : RGBCameraParser(node) {}

protected:
  void ImageToRos(const ImageData &img,
                  const std::string &frame,
                  sensor_msgs::msg::Image *img_msg) const override {
    img_msg->width = static_cast<uint32_t>(img.width);
    img_msg->height = static_cast<uint32_t>(img.height);
    img_msg->encoding = "16UC1";
    img_msg->header.frame_id = frame;
    img_msg->step = static_cast<uint32_t>(img.width * 2);
    img_msg->data.resize(static_cast<size_t>(img.width) * static_cast<size_t>(img.height) * 2);

    const int pixels = img.width * img.height;
    const auto *src = img.data.get();
    const float max_range_millimeters = GetMaxRange() * 1000.0f;

    for (int i = 0; i < pixels; ++i) {
      const uint8_t r = src[3 * i];
      const uint8_t g = src[3 * i + 1];
      const uint8_t b = src[3 * i + 2];
      float depth = static_cast<float>(r) / 255.0f +
                    static_cast<float>(g) / (255.0f * 255.0f) +
                    static_cast<float>(b) / (255.0f * 255.0f * 255.0f);
      if (depth > 0.99f) {
        depth = 0.0f;
      }
      const auto mm = static_cast<uint16_t>(depth * max_range_millimeters);
      img_msg->data[2 * i] = static_cast<uint8_t>(mm & 0xFF);
      img_msg->data[2 * i + 1] = static_cast<uint8_t>((mm >> 8) & 0xFF);
    }
  }
};
