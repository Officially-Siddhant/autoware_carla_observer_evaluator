#ifndef LIDAR_BRIDGE_HPP_
#define LIDAR_BRIDGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace autoware_carla_cpp_bridge
{

class LidarBridge : public rclcpp::Node
{
public:
  explicit LidarBridge(
    const std::string & node_name,
    const std::string & input_topic,
    const std::string & output_topic,
    const std::string & frame_id,
    size_t num_channels,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg);
  void syncStateCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void setSyncState();
  void unsetSyncState();
  void syncTimerCallback();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr _aw_time_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sync_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr ready_state_pub_;

  rclcpp::TimerBase::SharedPtr sync_timer_;

  std::string frame_id_;
  size_t num_channels_;
  rclcpp::Time aw_time_;
  bool sync_state_ = true;
  sensor_msgs::msg::PointCloud2 last_lidar_msg_;
  bool last_lidar_msg_received = false;

};

}  // namespace autoware_carla_cpp_bridge

#endif  // LIDAR_BRIDGE_HPP_