#include <chrono>
#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "dart_calculate_node/fin_allocator.hpp"
#include "dart_calculate_node/attitude_adjuster.hpp"
#include "dart_interfaces/msg/fin_command.hpp"
#include "dart_interfaces/msg/guidance_command.hpp"
#include "dart_interfaces/msg/target_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dart_calculate_node
{
namespace
{
template<std::size_t Size>
std::array<double, Size> array_parameter(
  rclcpp::Node & node, const std::string & name, const std::array<double, Size> & defaults)
{
  const auto values = node.declare_parameter<std::vector<double>>(
    name, std::vector<double>(defaults.begin(), defaults.end()));
  if (values.size() != Size) {throw std::invalid_argument(name + " has invalid length");}
  std::array<double, Size> result{};
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}
}  // namespace

class DartCalculateNode final : public rclcpp::Node
{
public:
  DartCalculateNode() : Node("dart_calculate_node")
  {
    target_topic_ = declare_parameter<std::string>("target_topic", "/dart_estimate_node/target_state");
    const auto airframe = declare_parameter<std::string>("model.airframe", "four_independent");
    controller_ = declare_parameter<std::string>("model.controller", "pid");
    output_enabled_ = declare_parameter<bool>("safety.output_enabled", false);
    target_timeout_s_ = declare_parameter<double>("target_timeout_s", 0.1);
    winding_offset_rad_ = declare_parameter<double>("attitude.winding_offset_rad", 0.0);
    minimum_attack_rad_ = declare_parameter<double>("attitude.minimum_attack_rad", 0.001);
    attack_gain_ = declare_parameter<double>("attitude.attack_gain", 1.0);
    winding_gain_ = declare_parameter<double>("attitude.winding_gain", 1.0);
    attack_control_limit_ = declare_parameter<double>("attitude.attack_control_limit", 0.35);
    winding_control_limit_ = declare_parameter<double>("attitude.winding_control_limit", 0.35);
    const auto rate_hz = declare_parameter<double>("control_rate_hz", 100.0);
    if (controller_ != "pid" && controller_ != "mpc") {
      throw std::invalid_argument("model.controller must be pid or mpc");
    }
    if (controller_ == "mpc") {
      throw std::invalid_argument("MPC is reserved by the framework and is not implemented yet");
    }
    if (rate_hz <= 0.0 || target_topic_.empty() || target_timeout_s_ <= 0.0 ||
      minimum_attack_rad_ < 0.0 || attack_gain_ < 0.0 || winding_gain_ < 0.0 ||
      attack_control_limit_ <= 0.0 || winding_control_limit_ <= 0.0)
    {
      throw std::invalid_argument("invalid dart_calculate_node configuration");
    }
    FinAllocatorConfig config;
    config.model = parse_airframe_model(airframe);
    config.mixing_matrix = array_parameter(*this, "allocation.mixing_matrix", config.mixing_matrix);
    config.neutral_rad = array_parameter(*this, "allocation.neutral_rad", config.neutral_rad);
    config.direction = array_parameter(*this, "allocation.direction", config.direction);
    config.min_rad = array_parameter(*this, "allocation.min_rad", config.min_rad);
    config.max_rad = array_parameter(*this, "allocation.max_rad", config.max_rad);
    allocator_ = std::make_unique<FinAllocator>(config);
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    command_pub_ = create_publisher<dart_interfaces::msg::FinCommand>("/mcu_node/fin_command", qos);
    guidance_pub_ = create_publisher<dart_interfaces::msg::GuidanceCommand>("~/guidance", qos);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", qos);
    target_sub_ = create_subscription<dart_interfaces::msg::TargetState>(
      target_topic_, qos, [this](dart_interfaces::msg::TargetState::ConstSharedPtr message) {
        latest_target_ = std::move(message);
        last_target_received_ = std::chrono::steady_clock::now();
      });
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz), [this] {control_tick();});
    diagnostics_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {publish_diagnostics();});
  }

private:
  void control_tick()
  {
    dart_interfaces::msg::FinCommand fin;
    fin.header.stamp = now();
    fin.sequence = ++sequence_;
    fin.neutral = true;
    fin.armed = false;
    fin.valid = false;
    fin.deflection_rad = {0.0F, 0.0F, 0.0F, 0.0F};
    dart_interfaces::msg::GuidanceCommand guidance;
    guidance.header = fin.header;
    guidance.sequence = fin.sequence;
    guidance.output_enabled = output_enabled_;
    const auto target_fresh = latest_target_ &&
      last_target_received_ != std::chrono::steady_clock::time_point{} &&
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_target_received_).count() <= target_timeout_s_;
    if (target_fresh && latest_target_->valid && latest_target_->imu_quaternion_valid) {
      const Quaternion quaternion{
        latest_target_->imu_quaternion_w, latest_target_->imu_quaternion_x,
        latest_target_->imu_quaternion_y, latest_target_->imu_quaternion_z};
      const auto adjustment = calculate_attitude_adjustment(
        latest_target_->predicted_bearing_x_rad, latest_target_->predicted_bearing_y_rad,
        quaternion, winding_offset_rad_, minimum_attack_rad_);
      guidance.valid = adjustment.valid;
      guidance.body_roll_rad = static_cast<float>(adjustment.body_roll_rad);
      guidance.target_winding_rad = static_cast<float>(adjustment.target_winding_rad);
      guidance.attack_error_rad = static_cast<float>(adjustment.attack_error_rad);
      guidance.roll_error_rad = static_cast<float>(adjustment.winding_error_rad);
      guidance.attack_control = static_cast<float>(std::clamp(
        attack_gain_ * adjustment.attack_error_rad,
        -attack_control_limit_, attack_control_limit_));
      guidance.roll_control = static_cast<float>(std::clamp(
        winding_gain_ * adjustment.winding_error_rad,
        -winding_control_limit_, winding_control_limit_));
      if (guidance.valid) {
        const auto deflection = allocator_->allocate(
          guidance.attack_control, guidance.roll_control);
        fin.valid = true;
        if (output_enabled_) {
          fin.armed = true;
          fin.neutral = false;
          for (std::size_t index = 0; index < deflection.size(); ++index) {
            fin.deflection_rad[index] = static_cast<float>(deflection[index]);
          }
        }
      }
    }
    guidance_pub_->publish(guidance);
    command_pub_->publish(fin);
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "dart_calculate_node/status";
    status.hardware_id = "dart_fin_framework";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = output_enabled_ ? "attitude_adjustment_enabled" : "output_disabled";
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(std::move(array));
  }

  std::string target_topic_;
  std::string controller_;
  bool output_enabled_{false};
  double target_timeout_s_{0.1};
  double winding_offset_rad_{0.0};
  double minimum_attack_rad_{0.001};
  double attack_gain_{1.0};
  double winding_gain_{1.0};
  double attack_control_limit_{0.35};
  double winding_control_limit_{0.35};
  std::uint64_t sequence_{0};
  std::unique_ptr<FinAllocator> allocator_;
  dart_interfaces::msg::TargetState::ConstSharedPtr latest_target_;
  std::chrono::steady_clock::time_point last_target_received_{};
  rclcpp::Subscription<dart_interfaces::msg::TargetState>::SharedPtr target_sub_;
  rclcpp::Publisher<dart_interfaces::msg::FinCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<dart_interfaces::msg::GuidanceCommand>::SharedPtr guidance_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};
}  // namespace dart_calculate_node

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dart_calculate_node::DartCalculateNode>());
  rclcpp::shutdown();
  return 0;
}
