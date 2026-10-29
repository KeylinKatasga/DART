#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "dart_interfaces/msg/coordinate_observation.hpp"
#include "dart_interfaces/msg/target_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dart_estimate_node
{
class DartEstimateNode final : public rclcpp::Node
{
public:
  DartEstimateNode() : Node("dart_estimate_node")
  {
    observation_topic_ = declare_parameter<std::string>(
      "observation_topic", "/dart_coordinate_node/observation");
    prediction_model_ = declare_parameter<std::string>("prediction.model", "static");
    prediction_horizon_s_ = declare_parameter<double>("prediction.horizon_s", 0.0);
    if (observation_topic_.empty() || prediction_horizon_s_ < 0.0) {
      throw std::invalid_argument("invalid dart_estimate_node configuration");
    }
    if (prediction_model_ != "static" && prediction_model_ != "sinusoidal_velocity") {
      throw std::invalid_argument("prediction.model must be static or sinusoidal_velocity");
    }
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
    estimate_pub_ = create_publisher<dart_interfaces::msg::TargetState>("~/target_state", qos);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10).reliable());
    observation_sub_ = create_subscription<dart_interfaces::msg::CoordinateObservation>(
      observation_topic_, qos,
      [this](const dart_interfaces::msg::CoordinateObservation & observation) {
        on_observation(observation);
      });
    diagnostics_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {publish_diagnostics();});
  }

private:
  void on_observation(const dart_interfaces::msg::CoordinateObservation & observation)
  {
    dart_interfaces::msg::TargetState output;
    output.header = observation.header;
    output.sequence = observation.sequence;
    output.valid = observation.valid;
    output.target_detected = observation.target_detected;
    output.imu_valid = observation.imu_valid;
    output.imu_quaternion_valid = observation.imu_valid;
    output.imu_units_verified = false;
    output.imu_age_s = observation.imu_age_s;
    output.motion_model = prediction_model_ == "static" ?
      dart_interfaces::msg::TargetState::MOTION_STATIC :
      dart_interfaces::msg::TargetState::MOTION_SINUSOIDAL_VELOCITY;
    output.prediction_horizon_s = static_cast<float>(prediction_horizon_s_);
    output.bearing_x_rad = observation.bearing_x_rad;
    output.bearing_y_rad = observation.bearing_y_rad;
    output.bearing_rate_x_rad_s = observation.bearing_rate_x_rad_s;
    output.bearing_rate_y_rad_s = observation.bearing_rate_y_rad_s;
    // Framework stage: both selectors preserve the current observation. The estimator interface
    // is now isolated so the user can add motion-model parameters without changing consumers.
    output.predicted_bearing_x_rad = observation.bearing_x_rad;
    output.predicted_bearing_y_rad = observation.bearing_y_rad;
    output.predicted_bearing_rate_x_rad_s = observation.bearing_rate_x_rad_s;
    output.predicted_bearing_rate_y_rad_s = observation.bearing_rate_y_rad_s;
    output.radius_px = observation.radius_px;
    output.model_confidence = observation.valid ? 1.0F : 0.0F;
    output.imu_quaternion_w = observation.imu_quaternion_w;
    output.imu_quaternion_x = observation.imu_quaternion_x;
    output.imu_quaternion_y = observation.imu_quaternion_y;
    output.imu_quaternion_z = observation.imu_quaternion_z;
    output.vehicle_x_vel = observation.vehicle_x_vel;
    output.vehicle_y_vel = observation.vehicle_y_vel;
    output.angle1 = observation.angle1;
    output.angle2 = observation.angle2;
    output.angle3 = observation.angle3;
    output.angle4 = observation.angle4;
    estimate_pub_->publish(output);
    ++observations_;
    if (output.valid) {++valid_estimates_;}
    last_valid_ = output.valid;
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "dart_estimate_node/status";
    status.hardware_id = "pure_bearing_estimator";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = last_valid_ ? "estimate_valid" : "waiting_for_observation";
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(std::move(array));
  }

  std::string observation_topic_;
  std::string prediction_model_;
  double prediction_horizon_s_{0.0};
  std::uint64_t observations_{0};
  std::uint64_t valid_estimates_{0};
  bool last_valid_{false};
  rclcpp::Subscription<dart_interfaces::msg::CoordinateObservation>::SharedPtr observation_sub_;
  rclcpp::Publisher<dart_interfaces::msg::TargetState>::SharedPtr estimate_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};
}  // namespace dart_estimate_node

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dart_estimate_node::DartEstimateNode>());
  rclcpp::shutdown();
  return 0;
}
