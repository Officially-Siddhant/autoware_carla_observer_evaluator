#include "autoware_carla_cpp_bridge/lidar_bridge.hpp"
#include <memory>

namespace autoware_carla_cpp_bridge
{

LidarBridge::LidarBridge(
  const std::string & node_name,
  const std::string & input_topic,
  const std::string & output_topic,
  const std::string & frame_id,
  size_t num_channels,
  const rclcpp::NodeOptions & options)
: Node(node_name, options),
  frame_id_(frame_id),
  num_channels_(num_channels)
{
  RCLCPP_INFO(this->get_logger(), "LidarBridge Node '%s' started.", node_name.c_str());

  // Publisher
  rclcpp::QoS qos(rclcpp::KeepLast(5));
  qos.best_effort();
  lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, qos);

  rclcpp::QoS diagnostic_qos(rclcpp::KeepLast(5));
  diagnostic_qos.reliable();
  diagnostic_qos.transient_local();
  ready_state_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
    "/ready_state",
    diagnostic_qos
  );

  sync_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&LidarBridge::syncTimerCallback, this)
  );

  // Subscriber
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic,
    10,
    std::bind(&LidarBridge::lidarCallback, this, std::placeholders::_1)
  );

  _aw_time_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
    "/autoware_time",
    10,
    std::bind(&LidarBridge::clockCallback, this, std::placeholders::_1)
  );

  sync_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/awcarla/sync_state",
    10,
    std::bind(&LidarBridge::syncStateCallback, this, std::placeholders::_1)
  );

}

void LidarBridge::clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
  aw_time_ = msg->clock;
}

void LidarBridge::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  sensor_msgs::msg::PointCloud2 modified_msg;
  modified_msg.header = msg->header;
  modified_msg.header.frame_id = frame_id_;
  modified_msg.header.stamp = aw_time_;

  size_t total_points = msg->width * msg->height;
  size_t points_per_channel = total_points / num_channels_;
  size_t valid_point_count = points_per_channel * num_channels_;

  modified_msg.point_step = 16;
  modified_msg.row_step = modified_msg.point_step * valid_point_count;
  modified_msg.width = valid_point_count;
  modified_msg.height = 1;
  modified_msg.is_dense = true;
  modified_msg.data.resize(valid_point_count * modified_msg.point_step);

  // Resize fields
  modified_msg.fields.resize(6);

  modified_msg.fields[0].name = "x";
  modified_msg.fields[0].offset = 0;
  modified_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  modified_msg.fields[0].count = 1;

  modified_msg.fields[1].name = "y";
  modified_msg.fields[1].offset = 4;
  modified_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  modified_msg.fields[1].count = 1;

  modified_msg.fields[2].name = "z";
  modified_msg.fields[2].offset = 8;
  modified_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  modified_msg.fields[2].count = 1;

  modified_msg.fields[3].name = "intensity";
  modified_msg.fields[3].offset = 12;
  modified_msg.fields[3].datatype = sensor_msgs::msg::PointField::UINT8;
  modified_msg.fields[3].count = 1;

  modified_msg.fields[4].name = "return_type";
  modified_msg.fields[4].offset = 13;
  modified_msg.fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
  modified_msg.fields[4].count = 1;

  modified_msg.fields[5].name = "channel";
  modified_msg.fields[5].offset = 14;
  modified_msg.fields[5].datatype = sensor_msgs::msg::PointField::UINT16;
  modified_msg.fields[5].count = 1;

  struct PointXYZIRC {
      float x;
      float y;
      float z;
      uint8_t intensity;
      uint8_t return_type;
      uint16_t channel;
  };

  PointXYZIRC* points = reinterpret_cast<PointXYZIRC*>(modified_msg.data.data());

  for (size_t i = 0; i < valid_point_count; ++i) {
      const uint8_t* src = &msg->data[i * msg->point_step];

      float x = *reinterpret_cast<const float*>(src + 0);
      float y = *reinterpret_cast<const float*>(src + 4);
      float z = *reinterpret_cast<const float*>(src + 8);
      float intensity_f = *reinterpret_cast<const float*>(src + 12);

      uint8_t intensity = static_cast<uint8_t>(std::clamp(intensity_f * 255.0f, 0.0f, 255.0f));

      points[i].x = x;
      points[i].y = y;
      points[i].z = z;
      points[i].intensity = intensity;
      points[i].return_type = 0;
      points[i].channel = static_cast<uint16_t>(i % num_channels_);
  }

    lidar_pub_->publish(modified_msg);

    if (sync_state_){
      last_lidar_msg_ = modified_msg;
      last_lidar_msg_received = true;
    }

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.name = this->get_name();
    ready_state_pub_->publish(status);
}

void LidarBridge::setSyncState()
{

  RCLCPP_INFO(this->get_logger(), "Setting lidar sync state to true.");
  sync_state_ = true;
  
}

void LidarBridge::unsetSyncState()
{
  RCLCPP_INFO(this->get_logger(), "Setting lidar sync state to false.");
  sync_state_ = false;
  last_lidar_msg_received = false;
}

void LidarBridge::syncStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        this->setSyncState();
    } else {
        this->unsetSyncState();
    }
}

void LidarBridge::syncTimerCallback()
{
  if (!sync_state_) {
    return;
  }

  if (last_lidar_msg_received) {
    last_lidar_msg_.header.stamp = aw_time_;
    lidar_pub_->publish(last_lidar_msg_);
  }
}


}  // namespace autoware_carla_cpp_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 5) {
    RCLCPP_ERROR(rclcpp::get_logger("lidar_bridge"), "Usage: lidar_bridge_node <input_topic> <output_topic> <frame_id> <num_channels>");
    return 1;
  }

  std::string input_topic = argv[1];
  std::string output_topic = argv[2];
  std::string frame_id = argv[3];
  size_t num_channels = std::stoul(argv[4]);

  auto node = std::make_shared<autoware_carla_cpp_bridge::LidarBridge>(
    "lidar_bridge_node", input_topic, output_topic, frame_id, num_channels);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}