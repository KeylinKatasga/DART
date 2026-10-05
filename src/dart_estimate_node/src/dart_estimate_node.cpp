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
#include "dart_interfaces/msg/target_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "mcu_node/msg/rx.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dart_estimate_node
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

class DartEstimateNode final : public rclcpp::Node
{
public:
  DartEstimateNode() : Node("dart_estimate_node")
  {
    detection_topic_ = declare_parameter<std::string>(
      "detection_topic", "/detect_node/detection");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/mcu_node/message/rx");
    prediction_model_ = declare_parameter<std::string>("prediction.model", "static");
    prediction_horizon_s_ = declare_parameter<double>("prediction.horizon_s", 0.0);
    max_imu_age_s_ = declare_parameter<double>("max_imu_age_s", 0.05);
    const double diagnostic_period_s = declare_parameter<double>("diagnostic_period_s", 1.0);
    if (detection_topic_.empty() || imu_topic_.empty() || max_imu_age_s_ <= 0.0 ||
      diagnostic_period_s <= 0.0)
    {
      throw std::invalid_argument("invalid dart_estimate_node configuration");
    }
    if (prediction_model_ != "static" && prediction_model_ != "sinusoidal_velocity") {
      throw std::invalid_argument("prediction.model must be static or sinusoidal_velocity");
    }
    if (prediction_horizon_s_ < 0.0) {throw std::invalid_argument("prediction.horizon_s must be non-negative");}

    const auto reliable = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
    estimate_pub_ = create_publisher<dart_interfaces::msg::TargetState>("~/target_state", reliable);
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
      get_logger(), "started: detection=%s imu=%s output=/dart_estimate_node/target_state",
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
    dart_interfaces::msg::TargetState output;
    output.header = detection.header;
    output.target_detected = detection.detected;
    output.sequence = detection.sequence;
    output.radius_px = detection.radius_px;
    output.bearing_x_rad = detection.bearing_x_rad;
    output.bearing_y_rad = detection.bearing_y_rad;
    output.bearing_rate_x_rad_s = detection.velocity_bearing_x_rad_s;
    output.bearing_rate_y_rad_s = detection.velocity_bearing_y_rad_s;
    output.prediction_horizon_s = static_cast<float>(prediction_horizon_s_);
    output.motion_model = prediction_model_ == "static" ?
      dart_interfaces::msg::TargetState::MOTION_STATIC :
      dart_interfaces::msg::TargetState::MOTION_SINUSOIDAL_VELOCITY;
    // The framework exposes the selected model now. Model fitting is deliberately not enabled
    // until its frequency/phase parameters and real target data are commissioned.
    output.predicted_bearing_x_rad = output.bearing_x_rad;
    output.predicted_bearing_y_rad = output.bearing_y_rad;
    output.predicted_bearing_rate_x_rad_s = output.bearing_rate_x_rad_s;
    output.predicted_bearing_rate_y_rad_s = output.bearing_rate_y_rad_s;
    output.model_confidence = detection.detected ? 1.0F : 0.0F;
    output.imu_units_verified = false;
    output.imu_age_s = std::numeric_limits<float>::infinity();

    if (imu) {
      const auto detection_time = rclcpp::Time(detection.header.stamp);
      const auto imu_time = rclcpp::Time(imu->stamp);
      const double age = std::abs((detection_time - imu_time).seconds());
      output.imu_age_s = static_cast<float>(age);
      output.imu_valid = age <= max_imu_age_s_ && finite_quaternion(*imu);
      output.imu_roll_raw = imu->imu_roll;
      output.imu_roll_velocity_raw = imu->imu_roll_vel;
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
    status.name = "dart_estimate_node/status";
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
  std::string prediction_model_;
  double max_imu_age_s_{0.05};
  double prediction_horizon_s_{0.0};
  std::mutex mutex_;
  mcu_node::msg::Rx::ConstSharedPtr latest_imu_;
  rclcpp::Subscription<dart_interfaces::msg::GreenLight>::SharedPtr detection_sub_;
  rclcpp::Subscription<mcu_node::msg::Rx>::SharedPtr imu_sub_;
  rclcpp::Publisher<dart_interfaces::msg::TargetState>::SharedPtr estimate_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::uint64_t detection_messages_{0};
  std::uint64_t imu_messages_{0};
  std::uint64_t valid_estimates_{0};
  bool last_target_detected_{false};
  bool last_imu_valid_{false};
  double last_imu_age_s_{-1.0};
};
}  // namespace dart_estimate_node

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dart_estimate_node::DartEstimateNode>());
  rclcpp::shutdown();
  return 0;
}
