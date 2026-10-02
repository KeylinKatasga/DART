#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "dart_interfaces/msg/green_light.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "estimate_node/msg/target_estimate.hpp"
#include "mcu_node/msg/rx.hpp"
#include "rclcpp/rclcpp.hpp"

namespace estimate_node
{
namespace
{
diagnostic_msgs::msg::KeyValue metric(const std::string & key, const auto value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = std::to_string(value);
  return item;
}

bool finite_quaternion(const mcu_node::msg::Rx & message)
{
  const double norm_squared =
    message.imu_quat_w * message.imu_quat_w +
    message.imu_quat_x * message.imu_quat_x +
    message.imu_quat_y * message.imu_quat_y +
    message.imu_quat_z * message.imu_quat_z;
  return std::isfinite(norm_squared) && norm_squared > 1.0e-8;
}
}  // namespace

class EstimateNode final : public rclcpp::Node
{
public:
  EstimateNode() : Node("estimate_node")
  {
    detection_topic_ = declare_parameter<std::string>(
      "detection_topic", "/detect_node/detection");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/mcu_node/message/rx");
    max_imu_age_s_ = declare_parameter<double>("max_imu_age_s", 0.05);
    const double diagnostic_period_s = declare_parameter<double>("diagnostic_period_s", 1.0);
    if (detection_topic_.empty() || imu_topic_.empty() || max_imu_age_s_ <= 0.0 ||
      diagnostic_period_s <= 0.0)
    {
      throw std::invalid_argument("invalid estimate_node configuration");
    }

    const auto reliable = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
    estimate_pub_ = create_publisher<msg::TargetEstimate>("~/target", reliable);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10).reliable());
    imu_sub_ = create_subscription<mcu_node::msg::Rx>(
      imu_topic_, reliable, [this](mcu_node::msg::Rx::ConstSharedPtr message) {
        std::lock_guard lock(mutex_);
        latest_imu_ = std::move(message);
        ++imu_messages_;
      });
    detection_sub_ = create_subscription<dart_interfaces::msg::GreenLight>(
      detection_topic_, reliable,
      [this](const dart_interfaces::msg::GreenLight & detection) {on_detection(detection);});
    diagnostics_timer_ = create_wall_timer(
      std::chrono::duration<double>(diagnostic_period_s), [this] {publish_diagnostics();});
    RCLCPP_INFO(
      get_logger(), "started: detection=%s imu=%s output=/estimate_node/target",
      detection_topic_.c_str(), imu_topic_.c_str());
  }

private:
  void on_detection(const dart_interfaces::msg::GreenLight & detection)
  {
    mcu_node::msg::Rx::ConstSharedPtr imu;
    {
      std::lock_guard lock(mutex_);
      imu = latest_imu_;
    }
    msg::TargetEstimate output;
    output.header = detection.header;
    output.target_detected = detection.detected;
    output.detection_sequence = detection.sequence;
    output.center_x_px = detection.center_x_px;
    output.center_y_px = detection.center_y_px;
    output.radius_px = detection.radius_px;
    output.bearing_x_rad = detection.bearing_x_rad;
    output.bearing_y_rad = detection.bearing_y_rad;
    output.los_rate_x_rad_s = detection.velocity_bearing_x_rad_s;
    output.los_rate_y_rad_s = detection.velocity_bearing_y_rad_s;
    output.imu_units_verified = false;
    output.imu_age_s = std::numeric_limits<float>::infinity();

    if (imu) {
      const auto detection_time = rclcpp::Time(detection.header.stamp);
      const auto imu_time = rclcpp::Time(imu->stamp);
      const double age = std::abs((detection_time - imu_time).seconds());
      output.imu_age_s = static_cast<float>(age);
      output.imu_valid = age <= max_imu_age_s_ && finite_quaternion(*imu);
      output.imu_sequence = imu->sequence;
      output.imu_stamp = imu->stamp;
      output.imu_device_sample_time_us = imu->device_sample_time_us;
      output.imu_yaw_raw = imu->imu_yaw;
      output.imu_pitch_raw = imu->imu_pitch;
      output.imu_roll_raw = imu->imu_roll;
      output.imu_yaw_velocity_raw = imu->imu_yaw_vel;
      output.imu_pitch_velocity_raw = imu->imu_pitch_vel;
      output.imu_roll_velocity_raw = imu->imu_roll_vel;
      output.imu_yaw_acceleration_raw = imu->imu_yaw_acc;
      output.imu_pitch_acceleration_raw = imu->imu_pitch_acc;
      output.imu_roll_acceleration_raw = imu->imu_roll_acc;
      output.imu_quaternion_w = imu->imu_quat_w;
      output.imu_quaternion_x = imu->imu_quat_x;
      output.imu_quaternion_y = imu->imu_quat_y;
      output.imu_quaternion_z = imu->imu_quat_z;
    }
    output.valid = output.target_detected && output.imu_valid;
    estimate_pub_->publish(output);
    ++detection_messages_;
    if (output.valid) ++valid_estimates_;
    last_target_detected_ = output.target_detected;
    last_imu_valid_ = output.imu_valid;
    last_imu_age_s_ = output.imu_age_s;
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "estimate_node/status";
    status.hardware_id = "green_light_mcu_observation";
    if (!last_target_detected_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "searching";
    } else if (!last_imu_valid_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "imu_missing_or_stale";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "observation_valid";
    }
    status.values = {
      metric("detection_messages", detection_messages_),
      metric("imu_messages", imu_messages_),
      metric("valid_estimates", valid_estimates_),
      metric("last_imu_age_s", last_imu_age_s_),
      metric("imu_units_verified", false)};
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(std::move(array));
  }

  std::string detection_topic_;
  std::string imu_topic_;
  double max_imu_age_s_{0.05};
  std::mutex mutex_;
  mcu_node::msg::Rx::ConstSharedPtr latest_imu_;
  rclcpp::Subscription<dart_interfaces::msg::GreenLight>::SharedPtr detection_sub_;
  rclcpp::Subscription<mcu_node::msg::Rx>::SharedPtr imu_sub_;
  rclcpp::Publisher<msg::TargetEstimate>::SharedPtr estimate_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::uint64_t detection_messages_{0};
  std::uint64_t imu_messages_{0};
  std::uint64_t valid_estimates_{0};
  bool last_target_detected_{false};
  bool last_imu_valid_{false};
  double last_imu_age_s_{-1.0};
};
}  // namespace estimate_node

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<estimate_node::EstimateNode>());
  rclcpp::shutdown();
  return 0;
}
