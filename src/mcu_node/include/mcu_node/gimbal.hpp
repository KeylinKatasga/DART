#pragma once
#include "mcu_node/protocol.hpp"
#include "mcu_node/serial_port.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "dart_interfaces/msg/fin_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "mcu_node/msg/tx.hpp"
#include "mcu_node/msg/raw_frame.hpp"
#include "mcu_node/msg/rx.hpp"

namespace mcu_node
{
class McuNode final : public rclcpp::Node
{
public:
  McuNode();
  ~McuNode() override;

private:
  using RawFrame = mcu_node::msg::RawFrame;
  SerialPort serial_;
  std::atomic_bool running_{true};
  std::thread read_thread_;
  std::atomic<std::uint64_t> sequence_{0};
  std::uint64_t commands_received_{0};
  std::uint64_t tx_succeeded_{0};
  std::uint64_t tx_failed_{0};
  std::chrono::steady_clock::time_point last_command_{};
  std::chrono::steady_clock::time_point last_tx_success_{};
  std::chrono::steady_clock::time_point last_tx_failure_{};
  double tx_stale_after_s_{2.0};
  std::string protocol_mode_{"gimbal_legacy"};
  bool actuator_output_enabled_{false};
  std::uint64_t fin_commands_received_{0};
  rclcpp::Publisher<mcu_node::msg::Rx>::SharedPtr state_pub_;
  rclcpp::Publisher<mcu_node::msg::Tx>::SharedPtr tx_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<RawFrame>::SharedPtr rx_raw_pub_;
  rclcpp::Publisher<RawFrame>::SharedPtr tx_raw_pub_;
  rclcpp::Subscription<mcu_node::msg::Tx>::SharedPtr command_sub_;
  rclcpp::Subscription<dart_interfaces::msg::FinCommand>::SharedPtr fin_command_sub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  bool publish_raw_;

  static std::string hex(const std::vector<std::uint8_t> & frame);
  void publish_raw(
    const std::vector<std::uint8_t> & frame, std::uint8_t direction,
    const rclcpp::Publisher<RawFrame>::SharedPtr & publisher);
  void read_loop();
  void publish_diagnostics();
};
}  // namespace mcu_node
