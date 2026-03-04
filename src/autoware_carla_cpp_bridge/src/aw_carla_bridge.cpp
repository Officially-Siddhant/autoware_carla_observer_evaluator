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
  last_sync_clock_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

  // Sync Stuff
  this->declare_parameter<std::vector<std::string>>(
    "nodes_to_wait_for", {"node_a", "node_b", "node_c"});

  this->get_parameter("nodes_to_wait_for", nodes_to_wait_for_);
  expected_nodes_ = std::set<std::string>(nodes_to_wait_for_.begin(),
                                          nodes_to_wait_for_.end());
  RCLCPP_INFO(this->get_logger(), "CPP Bridge waiting for %ld nodes", 
  expected_nodes_.size());

  sync_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&AWCarlaCPPBridge::syncTimerCallback, this)
  );

  gnss_pose_pub_= 
  this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/sensing/gnss/pose_with_covariance", 10);
  imu_pub_= 
  this->create_publisher<sensor_msgs::msg::Imu>("/sensing/imu/imu_data", 10);
  control_mode_pub_= 
  this->create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", 10);
  steering_status_pub_= 
  this->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", 10);
  velocity_status_pub_= 
  this->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 10);
  engage_pub_= 
  this->create_publisher<autoware_vehicle_msgs::msg::Engage>("/autoware/engage", 10);
  rclcpp::QoS trans_qos(rclcpp::KeepLast(1));
  trans_qos.transient_local();
  sync_state_pub_= 
  this->create_publisher<std_msgs::msg::Bool>("/awcarla/sync_state", trans_qos);

  gnss_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/sensing/gnss/pose_with_covariance", 10,
    [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) { last_gnss_pose_ = msg; });
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/sensing/imu/imu_data", 10,
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) { last_imu_data_ = msg; });
  control_mode_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::ControlModeReport>(
    "/vehicle/status/control_mode", 10,
    [this](const autoware_vehicle_msgs::msg::ControlModeReport::SharedPtr msg) { last_control_mode_ = msg; });
  steering_status_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", 10,
    [this](const autoware_vehicle_msgs::msg::SteeringReport::SharedPtr msg) { last_steering_status_ = msg; });
  velocity_status_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", 10,
    [this](const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg) { last_velocity_status_ = msg; });
  aw_ctrl_sub_ = this->create_subscription<autoware_control_msgs::msg::Control>(
    "/control/trajectory_follower/control_cmd", 10,
    std::bind(&AWCarlaCPPBridge::ctrlCallback, this, std::placeholders::_1));



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
    std::bind(&AWCarlaCPPBridge::readyCallback, this, std::placeholders::_1));

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
    setSyncState();
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

void AWCarlaCPPBridge::readyCallback(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg)
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

void AWCarlaCPPBridge::setSyncState()
{

  RCLCPP_INFO(this->get_logger(), "Setting sync state to true.");
  sync_state_ = true;
  std_msgs::msg::Bool sync_msg;
  sync_msg.data = sync_state_;
  sync_state_pub_->publish(sync_msg);
}

void AWCarlaCPPBridge::unsetSyncState()
{
  RCLCPP_INFO(this->get_logger(), "Setting sync state to false.");
  sync_state_ = false;
  std_msgs::msg::Bool sync_msg;
  sync_msg.data = sync_state_;
  sync_state_pub_->publish(sync_msg);
}

void AWCarlaCPPBridge::syncTimerCallback()
{
  if (!sync_state_) {
    return;
  }

  // Will get now new clock since sim will not be ticked while sync mode
  // TODO: Do we need last_sync_clock at all?
  if (last_sync_clock_ == last_autoware_clock_){
    rclcpp::Duration diff(0,50000000); // 50ms
    last_autoware_clock_ = last_autoware_clock_ + diff;
  }
  last_sync_clock_ = last_autoware_clock_;
  rosgraph_msgs::msg::Clock clock_msg;
  clock_msg.clock = last_autoware_clock_;
  clock_pub_->publish(clock_msg);
  // TODO: With the new sync we could basically use only clock_pub at /clock topic
  aw_time_pub_->publish(clock_msg);

  // Publish last messages with updated timestamps
  updateAndPublishWithHeader<geometry_msgs::msg::PoseWithCovarianceStamped>(last_gnss_pose_, gnss_pose_pub_);
  updateAndPublishWithHeader<sensor_msgs::msg::Imu>(last_imu_data_, imu_pub_);
  updateAndPublishWithStamp<autoware_vehicle_msgs::msg::ControlModeReport>(last_control_mode_, control_mode_pub_);
  updateAndPublishWithStamp<autoware_vehicle_msgs::msg::SteeringReport>(last_steering_status_, steering_status_pub_);
  updateAndPublishWithHeader<autoware_vehicle_msgs::msg::VelocityReport>(last_velocity_status_, velocity_status_pub_);
  autoware_vehicle_msgs::msg::Engage engage_msg;
  engage_msg.engage = true;
  engage_pub_->publish(engage_msg);
}

void AWCarlaCPPBridge::ctrlCallback(const autoware_control_msgs::msg::Control::SharedPtr msg)
{
  unsetSyncState();
}



}  // namespace autoware_carla_cpp_bridge