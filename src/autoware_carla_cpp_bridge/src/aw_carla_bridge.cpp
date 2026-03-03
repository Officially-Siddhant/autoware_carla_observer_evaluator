// This node primarily handles the conversion between the Carla timestamp, which resets to 0 with every scenario, and the Autoware timestamp, which is continuous. Additionally, it manages synchronization between the Python and C++ data providers.
#include "autoware_carla_cpp_bridge/aw_carla_bridge.hpp"
#include <string>
#include <memory>
#include <chrono>

namespace autoware_carla_cpp_bridge
{

using std::placeholders::_1;
using namespace std::chrono_literals;

AWCarlaCPPBridge::AWCarlaCPPBridge(const rclcpp::NodeOptions & options)
: Node("aw_carla_cpp_bridge", options)
{
  RCLCPP_INFO(this->get_logger(), "AW Carla CPP Bridge Node started.");

  last_carla_clock_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  last_autoware_clock_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

  // Sync Stuff
  this->declare_parameter<std::vector<std::string>>(
    "nodes_to_wait_for", {"node_a", "node_b", "node_c"});

  this->get_parameter("nodes_to_wait_for", nodes_to_wait_for_);
  expected_nodes_ = std::set<std::string>(nodes_to_wait_for_.begin(),
                                          nodes_to_wait_for_.end());
  RCLCPP_INFO(this->get_logger(), "CPP Bridge waiting for %ld nodes", 
  expected_nodes_.size());

  // Publisher
  aw_time_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/autoware_time", 10);
  clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

  // Subscriber
  carla_clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
    "/carla/clock",
    10,
    std::bind(&AWCarlaCPPBridge::clockCallback, this, _1)
  );

  ready_state_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
    "/ready_state", 10,
    std::bind(&AWCarlaCPPBridge::ready_callback, this, std::placeholders::_1));

}

void AWCarlaCPPBridge::clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
  rclcpp::Time current_carla_time(msg->clock);
  rclcpp::Duration diff(0,0);

  // This happens when a new scenario is started in Carla
  if (current_carla_time <= last_carla_clock_) {
    RCLCPP_INFO(this->get_logger(), 
                "Carla clock reset detected. Using full Carla time as diff.");

    // Set a static diff of one second
    diff = rclcpp::Duration(1, 0);
  }
  else {
    diff = current_carla_time - last_carla_clock_;
  }

  // Always increase Autoware time
  last_autoware_clock_ = last_autoware_clock_ + diff;

  rosgraph_msgs::msg::Clock clock_msg;
  clock_msg.clock = last_autoware_clock_;
  clock_pub_->publish(clock_msg);
  // TODO: With the new sync we could basically use only clock_pub at /clock topic
  aw_time_pub_->publish(clock_msg);
  last_carla_clock_ = current_carla_time;
}

void AWCarlaCPPBridge::ready_callback(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg)
{
  if (msg->level != diagnostic_msgs::msg::DiagnosticStatus::OK)
  {
      return;
  }

  const std::string &node_name = msg->name;

  if (expected_nodes_.count(node_name) == 0)
  {
      return;
  }

  current_ready_nodes_.insert(node_name);
  RCLCPP_DEBUG(this->get_logger(),
                    "Received ready from %s (%ld/%ld)",
                    node_name.c_str(),
                    current_ready_nodes_.size(),
                    expected_nodes_.size());

  if (current_ready_nodes_.size() == expected_nodes_.size())
  {
      RCLCPP_INFO(this->get_logger(),
                  "All nodes ready!");
      // Here can happen sync stuff if needed
      current_ready_nodes_.clear();
  }

}

}  // namespace autoware_carla_cpp_bridge