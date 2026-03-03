#include "autoware_carla_cpp_bridge/camera_bridge.hpp"
#include <memory>
#include <cmath>

namespace autoware_carla_cpp_bridge
{

  CameraBridge::CameraBridge(
  const std::string & node_name,
  const std::string & input_topic,
  const std::string & output_topic,
  const std::string & frame_id,
  const int & fov,
  const int & height,
  const int & width,
  const rclcpp::NodeOptions & options)
: Node(node_name, options),
  frame_id_(frame_id)
{
  RCLCPP_INFO(this->get_logger(), "CameraBridge Node '%s' started.", node_name.c_str());

  // Initialize Camera Info
  camera_info_.header.frame_id = frame_id_;
  camera_info_.height = height;
  camera_info_.width = width;
  camera_info_.distortion_model = "plumb_bob";
  double cx = width / 2.0;
  double cy = height / 2.0;
  double fx = width / (2.0 * std::tan(fov * M_PI / 360.0));
  double fy = fx;

  camera_info_.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  camera_info_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  camera_info_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camera_info_.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};

  // Publisher
  rclcpp::QoS qos(rclcpp::KeepLast(5));
  qos.best_effort();
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic, qos);

  std::string cam_info_topic = output_topic;
  size_t pos = cam_info_topic.find("image_raw");
  if (pos != std::string::npos) {
      cam_info_topic.replace(pos, 9, "camera_info");
  }
  cam_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(cam_info_topic, qos);

  rclcpp::QoS diagnostic_qos(rclcpp::KeepLast(5));
  diagnostic_qos.reliable();
  diagnostic_qos.transient_local();
  ready_state_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
    "/ready_state",
    diagnostic_qos
  );

  // Subscriber
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    input_topic,
    10,
    std::bind(&CameraBridge::imageCallback, this, std::placeholders::_1)
  );

  _aw_time_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
    "/autoware_time",
    10,
    std::bind(&CameraBridge::clockCallback, this, std::placeholders::_1)
  );
}

void CameraBridge::clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
  aw_time_ = msg->clock;
}

void CameraBridge::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  auto modified_msg = *msg;
  modified_msg.header.frame_id = frame_id_;
  modified_msg.header.stamp = aw_time_;
  image_pub_->publish(modified_msg);

  camera_info_.header.stamp = aw_time_;
  cam_info_pub_->publish(camera_info_);

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.name = this->get_name();
  ready_state_pub_->publish(status);
}


}  // namespace autoware_carla_cpp_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 7) {
    RCLCPP_ERROR(
      rclcpp::get_logger("camera_bridge"),
      "Usage: camera_bridge_node "
      "<input_topic> <output_topic> <frame_id> <num_channels> "
      "<fov> <height> <width>"
    );
    return 1;
  }

  std::string input_topic = argv[1];
  std::string output_topic = argv[2];
  std::string frame_id = argv[3];
  int fov = std::stoi(argv[4]);
  int height = std::stoi(argv[5]);
  int width = std::stoi(argv[6]);
  

  auto node = std::make_shared<autoware_carla_cpp_bridge::CameraBridge>(
    "camera_bridge_node", input_topic, output_topic, frame_id, fov, height, width);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}