#ifndef AW_CARLA_BRIDGE_HPP_
#define AW_CARLA_BRIDGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace autoware_carla_cpp_bridge
{

class AWCarlaCPPBridge : public rclcpp::Node
{
public:
  explicit AWCarlaCPPBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg);
  void ready_callback(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg);

  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr carla_clock_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr ready_state_sub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr aw_time_pub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

  rclcpp::Time last_carla_clock_;
  rclcpp::Time last_autoware_clock_;

  std::vector<std::string> nodes_to_wait_for_;
  std::set<std::string> expected_nodes_;
  std::set<std::string> current_ready_nodes_;
};

#endif  // AW_CARLA_BRIDGE_HPP_

}  // namespace autoware_carla_cpp_bridge