#ifndef AW_CARLA_BRIDGE_HPP_
#define AW_CARLA_BRIDGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_vehicle_msgs/msg/engage.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_control_msgs/msg/control.hpp>

namespace autoware_carla_cpp_bridge
{

class AWCarlaCPPBridge : public rclcpp::Node
{
public:
  explicit AWCarlaCPPBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:  
  // Sync stuff
  void setSyncState();
  void unsetSyncState();
  void syncTimerCallback();
  void ctrlCallback(const autoware_control_msgs::msg::Control::SharedPtr msg);

  template <typename T>
  void updateAndPublishWithHeader(
    typename T::SharedPtr & msg_ptr,
    typename rclcpp::Publisher<T>::SharedPtr & pub)
  {
    if (msg_ptr && pub) {
      msg_ptr->header.stamp = last_autoware_clock_;
      pub->publish(*msg_ptr);
    }
  }

  template <typename T>
  void updateAndPublishWithStamp(
    typename T::SharedPtr & msg_ptr,
    typename rclcpp::Publisher<T>::SharedPtr & pub)
  {
    if (msg_ptr && pub) {
      msg_ptr->stamp = last_autoware_clock_;
      pub->publish(*msg_ptr);
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_status_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_status_sub_;
  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr aw_ctrl_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_status_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_status_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::Engage>::SharedPtr engage_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sync_state_pub_;

  bool sync_state_ = true;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_gnss_pose_ = nullptr;
  sensor_msgs::msg::Imu::SharedPtr                         last_imu_data_ = nullptr;
  autoware_vehicle_msgs::msg::ControlModeReport::SharedPtr last_control_mode_ = nullptr;
  autoware_vehicle_msgs::msg::SteeringReport::SharedPtr    last_steering_status_ = nullptr;
  autoware_vehicle_msgs::msg::VelocityReport::SharedPtr    last_velocity_status_ = nullptr;
  // End of Sync stuff


  void clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg);
  void readyCallback(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg);

  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr carla_clock_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr ready_state_sub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr aw_time_pub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  

  rclcpp::TimerBase::SharedPtr sync_timer_;
  rclcpp::Time last_sync_clock_;

  rclcpp::Time last_carla_clock_;
  rclcpp::Time last_autoware_clock_;

  std::vector<std::string> nodes_to_wait_for_;
  std::set<std::string> expected_nodes_;
  std::set<std::string> current_ready_nodes_;
};
}  // namespace autoware_carla_cpp_bridge

#endif  // AW_CARLA_BRIDGE_HPP_

