#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <string>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <netdb.h>
#include <rclcpp/rclcpp.hpp>
#include <sys/socket.h>
#include <unistd.h>

#include "json_nlohmann/json.hpp"
#include "simulation/msg/vehicle_control.hpp"

using json = nlohmann::json;
using VehicleControl = simulation::msg::VehicleControl;

namespace {
std::string default_config_path() {
  try {
    return ament_index_cpp::get_package_share_directory("simulation") +
           "/unity_sim/Build_Ubuntu/AD_Sim_Data/StreamingAssets/simulation_config.json";
  } catch (const std::exception &) {
    return "";
  }
}

class UDPClient {
public:
  UDPClient(const std::string &host, int port) {
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;

    addrinfo *result = nullptr;
    const int rc = ::getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &result);
    if (rc != 0) {
      throw std::runtime_error(std::string("getaddrinfo failed: ") + gai_strerror(rc));
    }

    for (addrinfo *rp = result; rp != nullptr; rp = rp->ai_next) {
      fd_ = ::socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
      if (fd_ < 0) {
        continue;
      }
      addr_len_ = rp->ai_addrlen;
      std::memcpy(&addr_, rp->ai_addr, rp->ai_addrlen);
      break;
    }
    ::freeaddrinfo(result);

    if (fd_ < 0) {
      throw std::runtime_error("failed to create UDP socket");
    }
  }

  ~UDPClient() {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  void send_command(const std::array<float, 4> &command) {
    ::sendto(fd_, command.data(), sizeof(float) * command.size(), 0,
             reinterpret_cast<sockaddr *>(&addr_), addr_len_);
  }

private:
  int fd_{-1};
  sockaddr_storage addr_{};
  socklen_t addr_len_{};
};
}  // namespace

class ROSCommandTransmitter : public rclcpp::Node {
public:
  ROSCommandTransmitter() : Node("ROS_Unity_command_Tx") {
    declare_parameter<std::string>("address", "127.0.0.1");
    declare_parameter<int>("port", 6657);
    declare_parameter<std::string>("command_topic", "car_command");
    declare_parameter<std::string>("config_file", default_config_path());
    LoadConfigParameters();

    const auto topic = get_parameter("command_topic").as_string();
    subscription_ = create_subscription<VehicleControl>(
        topic, rclcpp::QoS(1),
        [this](const VehicleControl::SharedPtr msg) {
          current_command_[0] = msg->throttle;
          current_command_[1] = msg->steering;
          current_command_[2] = msg->brake;
          current_command_[3] = msg->reserved;
        });
  }

  void Run() {
    const auto address = get_parameter("address").as_string();
    const int port = get_parameter("port").as_int();
    const auto topic = get_parameter("command_topic").as_string();

    RCLCPP_INFO(get_logger(), "Starting command transmit to Unity via UDP %s:%d", address.c_str(), port);
    RCLCPP_INFO(get_logger(), "Listening to command topic: %s", topic.c_str());

    UDPClient client(address, port);
    rclcpp::Rate rate(100.0);
    while (rclcpp::ok()) {
      bool valid = true;
      for (float value : current_command_) {
        if (std::isnan(value)) {
          valid = false;
          break;
        }
      }
      if (valid) {
        client.send_command(current_command_);
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Received NaN in command message");
      }
      rclcpp::spin_some(this->get_node_base_interface());
      rate.sleep();
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
      if (config.contains("controlConfig")) {
        if (config["controlConfig"].contains("address")) {
          set_parameter(rclcpp::Parameter("address", static_cast<std::string>(config["controlConfig"]["address"])));
        }
        if (config["controlConfig"].contains("port")) {
          set_parameter(rclcpp::Parameter("port", static_cast<int>(config["controlConfig"]["port"])));
        }
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "Could not parse config file %s: %s", config_path.c_str(), e.what());
    }
  }

  std::array<float, 4> current_command_{0.0f, 0.0f, 0.0f, 0.0f};
  rclcpp::Subscription<VehicleControl>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ROSCommandTransmitter>();
  try {
    node->Run();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
