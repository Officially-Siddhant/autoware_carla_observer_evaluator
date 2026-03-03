#include "autoware_carla_cpp_bridge/aw_carla_bridge.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<autoware_carla_cpp_bridge::AWCarlaCPPBridge>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
