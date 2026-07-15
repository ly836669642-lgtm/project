#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <simulation/msg/vehicle_control.hpp>

constexpr float loop_interval = 0.05F;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("dummy_controller_node");
  auto pub = node->create_publisher<simulation::msg::VehicleControl>("car_command", rclcpp::QoS(1));
  rclcpp::Rate loop_rate(1.0F / loop_interval);
  float elapsed_time = 0.0F;

  while (rclcpp::ok())
  {
    simulation::msg::VehicleControl msg;
    msg.throttle = 0.5F;  // Throttle value from -1 to 1, this is the torque applied to the motors
    msg.steering = std::sin(6.28F * elapsed_time) * 0.5F;  // Positive value <=> turning right
    msg.brake = 0.0F;  // Brake value from 0 to 1, this will apply brake torque to stop the car
    msg.reserved = 0.0F;  // Not used

    pub->publish(msg);

    rclcpp::spin_some(node);
    loop_rate.sleep();
    elapsed_time += loop_interval;
  }

  rclcpp::shutdown();
  return 0;
}
