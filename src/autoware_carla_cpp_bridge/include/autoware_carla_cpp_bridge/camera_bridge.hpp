#ifndef CAMERA_BRIDGE_HPP_
#define CAMERA_BRIDGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace autoware_carla_cpp_bridge
{

class CameraBridge : public rclcpp::Node
{
public:
  explicit CameraBridge(
    const std::string & node_name,
    const std::string & input_topic,
    const std::string & output_topic,
    const std::string & frame_id,
    const int & fov,
    const int & height,
    const int & width,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr _aw_time_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr ready_state_pub_;
  std::string frame_id_;
  rclcpp::Time aw_time_;
  sensor_msgs::msg::CameraInfo camera_info_;
};

}  // namespace autoware_carla_cpp_bridge

#endif  // CAMERA_BRIDGE_HPP_