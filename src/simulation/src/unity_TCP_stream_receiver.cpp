#include <chrono>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "depth_camera_parser.h"
#include "imu_parser.h"
#include "json_nlohmann/json.hpp"
#include "rgb_camera_parser.h"
#include "true_state_parser.h"
#include "unity_stream_parser.h"

using json = nlohmann::json;

namespace {
std::string default_config_path() {
  try {
    return ament_index_cpp::get_package_share_directory("simulation") +
           "/unity_sim/Build_Ubuntu/AD_Sim_Data/StreamingAssets/simulation_config.json";
  } catch (const std::exception &) {
    return "";
  }
}

double now_in_unity_tick_seconds(const rclcpp::Clock &clock) {
  return static_cast<double>(clock.now().nanoseconds()) * 1e-9;
}
}

class UnityTCPStreamReceiver : public rclcpp::Node {
public:
  UnityTCPStreamReceiver() : Node("Unity_ROS_message_Rx") {
    declare_parameter<std::string>("bind_host", "0.0.0.0");
    declare_parameter<int>("port", 9998);
    declare_parameter<std::string>("config_file", default_config_path());
    LoadConfigParameters();
  }

  void Run() {
    const std::string bind_host = get_parameter("bind_host").as_string();
    const int port = get_parameter("port").as_int();

    std::unique_ptr<TCPStreamReader> stream_reader;
    try {
      stream_reader = std::make_unique<TCPStreamReader>(bind_host, std::to_string(port));
    } catch (const TCPStreamReaderException &e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      return;
    }

    RCLCPP_INFO(get_logger(), "Waiting for Unity TCP connection on %s:%d", bind_host.c_str(), port);
    try {
      stream_reader->WaitConnect();
    } catch (const TCPStreamReaderException &e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      return;
    }
    RCLCPP_INFO(get_logger(), "Got a Unity TCP connection");

    std::vector<std::shared_ptr<UnityStreamParser>> stream_parsers(UnityMessageType::MESSAGE_TYPE_COUNT);
    stream_parsers[UnityMessageType::UNITY_STATE] = std::make_shared<TrueStateParser>(this);
    stream_parsers[UnityMessageType::UNITY_IMU] = std::make_shared<IMUParser>(this);
    stream_parsers[UnityMessageType::UNITY_CAMERA] = std::make_shared<RGBCameraParser>(this);
    stream_parsers[UnityMessageType::UNITY_DEPTH] = std::make_shared<DepthCameraParser>(this);

    bool first_message_received = false;
    double time_offset = 0.0;

    while (rclcpp::ok() && stream_reader->Good()) {
      try {
        const uint32_t magic = stream_reader->ReadUInt();
        if (magic != 0xDEADC0DE) {
          RCLCPP_ERROR(get_logger(), "Stream corrupted: magic mismatch 0x%08x", magic);
          continue;
        }

        UnityHeader header;
        header.type = static_cast<UnityMessageType>(stream_reader->ReadUInt());
        const uint64_t ticks = stream_reader->ReadUInt64();
        header.name = stream_reader->ReadString();
        header.timestamp = static_cast<double>(ticks) * 1e-7;

        if (!first_message_received) {
          time_offset = now_in_unity_tick_seconds(*get_clock()) - header.timestamp;
          first_message_received = true;
          RCLCPP_INFO(get_logger(), "Unity-ROS timeSync: time_offset = %.9f s", time_offset);
        }

        if (header.type < UnityMessageType::MESSAGE_TYPE_COUNT && stream_parsers[header.type]) {
          stream_parsers[header.type]->ParseMessage(header, *stream_reader, time_offset);
        } else if (header.type < UnityMessageType::MESSAGE_TYPE_COUNT) {
          RCLCPP_WARN(get_logger(), "Message type %u is known but not implemented in this bridge", static_cast<unsigned>(header.type));
        } else {
          RCLCPP_ERROR(get_logger(), "Received unknown Unity message type %u", static_cast<unsigned>(header.type));
        }
      } catch (const TCPStreamReaderException &e) {
        RCLCPP_ERROR(get_logger(), "TCP stream ended: %s", e.what());
        break;
      } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Parser error: %s", e.what());
        break;
      }
      rclcpp::spin_some(this->get_node_base_interface());
    }
  }

private:
  void LoadConfigParameters() {
    const std::string config_path = get_parameter("config_file").as_string();
    if (config_path.empty()) {
      return;
    }
    std::ifstream file(config_path);
    if (!file.is_open()) {
      RCLCPP_WARN(get_logger(), "Config file not found at %s; using node parameters/defaults", config_path.c_str());
      return;
    }

    try {
      json config;
      file >> config;
      if (config.contains("sensorConfig")) {
        if (config["sensorConfig"].contains("port")) {
          set_parameter(rclcpp::Parameter("port", static_cast<int>(config["sensorConfig"]["port"])));
        }
        // Unity connects to ROS. For the ROS server, bind_host should usually stay 0.0.0.0.
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "Could not parse config file %s: %s", config_path.c_str(), e.what());
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UnityTCPStreamReceiver>();
  node->Run();
  rclcpp::shutdown();
  return 0;
}
