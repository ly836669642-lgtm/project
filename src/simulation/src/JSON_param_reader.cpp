#include <fstream>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include "json_nlohmann/json.hpp"

using json = nlohmann::json;

namespace {
std::string default_config_path() {
  try {
    return ament_index_cpp::get_package_share_directory("simulation") +
           "/unity_sim/Build_Ubuntu/AD_Sim_Data/StreamingAssets/simulation_config.json";
  } catch (const std::exception &) {
    return "unity_sim/Build_Ubuntu/AD_Sim_Data/StreamingAssets/simulation_config.json";
  }
}
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("json_param_loader");

  node->declare_parameter<std::string>("config_file", default_config_path());
  const std::string config_path = node->get_parameter("config_file").as_string();

  std::ifstream config_file(config_path);
  if (!config_file.is_open()) {
    RCLCPP_WARN(node->get_logger(),
                "Cannot open JSON config at %s; continuing with node parameters/defaults",
                config_path.c_str());
    rclcpp::shutdown();
    return 0;
  }

  json config;
  try {
    config_file >> config;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
    RCLCPP_ERROR(node->get_logger(), "JSON reader failed. Is the configuration file correctly structured?");
    rclcpp::shutdown();
    return 1;
  }

  try {
    const std::string ctrl_addr = config["controlConfig"]["address"];
    const int ctrl_port = config["controlConfig"]["port"];
    node->declare_parameter<std::string>("controlConfig.address", ctrl_addr);
    node->declare_parameter<int>("controlConfig.port", ctrl_port);
    RCLCPP_INFO(node->get_logger(), "Read control message Tx address: %s, port: %d", ctrl_addr.c_str(), ctrl_port);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "controlConfig is incomplete or invalid in %s: %s", config_path.c_str(), e.what());
  }

  try {
    const std::string sensor_addr = config["sensorConfig"]["address"];
    const int sensor_port = config["sensorConfig"]["port"];
    node->declare_parameter<std::string>("sensorConfig.address", sensor_addr);
    node->declare_parameter<int>("sensorConfig.port", sensor_port);
    RCLCPP_INFO(node->get_logger(), "Read sensor message Rx address: %s, port: %d", sensor_addr.c_str(), sensor_port);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "sensorConfig is incomplete or invalid in %s: %s", config_path.c_str(), e.what());
  }

  rclcpp::shutdown();
  return 0;
}
