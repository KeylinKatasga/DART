#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "aimer_node/fin_allocator.hpp"
#include "dart_interfaces/msg/fin_command.hpp"
#include "dart_interfaces/msg/guidance_command.hpp"
#include "dart_interfaces/msg/target_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace aimer_node
{
class AimerNode final : public rclcpp::Node
{
public:
  AimerNode() : Node("aimer_node")
  {
    target_topic_ = declare_parameter<std::string>("target_topic", "/estimate_node/target_state");
    const auto airframe = declare_parameter<std::string>("model.airframe", "four_independent");
    controller_ = declare_parameter<std::string>("model.controller", "pid");
    output_enabled_ = declare_parameter<bool>("safety.output_enabled", false);
    const auto rate_hz = declare_parameter<double>("control_rate_hz", 100.0);
    if (controller_ != "pid" && controller_ != "mpc") {
      throw std::invalid_argument("model.controller must be pid or mpc");
    }
    if (controller_ == "mpc") {
      throw std::invalid_argument("MPC is reserved by the framework and is not implemented yet");
    }
    if (rate_hz <= 0.0 || target_topic_.empty()) {throw std::invalid_argument("invalid aimer_node configuration");}
    FinAllocatorConfig config;
    config.model = parse_airframe_model(airframe);
    allocator_ = std::make_unique<FinAllocator>(config);
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    command_pub_ = create_publisher<dart_interfaces::msg::FinCommand>("/mcu_node/fin_command", qos);
    guidance_pub_ = create_publisher<dart_interfaces::msg::GuidanceCommand>("~/guidance", qos);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", qos);
    target_sub_ = create_subscription<dart_interfaces::msg::TargetState>(
      target_topic_, qos, [this](dart_interfaces::msg::TargetState::ConstSharedPtr message) {
        latest_target_ = std::move(message);
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
    if (latest_target_ && latest_target_->valid) {
      guidance.valid = true;
      guidance.attack_error_rad = -latest_target_->predicted_bearing_y_rad;
      guidance.roll_error_rad = 0.0F;
    }
    guidance_pub_->publish(guidance);
    command_pub_->publish(fin);
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "aimer_node/status";
    status.hardware_id = "dart_fin_framework";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = output_enabled_ ? "framework_neutral_only" : "output_disabled";
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(std::move(array));
  }

  std::string target_topic_;
  std::string controller_;
  bool output_enabled_{false};
  std::uint64_t sequence_{0};
  std::unique_ptr<FinAllocator> allocator_;
  dart_interfaces::msg::TargetState::ConstSharedPtr latest_target_;
  rclcpp::Subscription<dart_interfaces::msg::TargetState>::SharedPtr target_sub_;
  rclcpp::Publisher<dart_interfaces::msg::FinCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<dart_interfaces::msg::GuidanceCommand>::SharedPtr guidance_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};
}  // namespace aimer_node

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aimer_node::AimerNode>());
  rclcpp::shutdown();
  return 0;
}
