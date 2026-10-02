#include "mcu_node/gimbal.hpp"
#include <iomanip>
#include <sstream>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace mcu_node
{
McuNode::McuNode()
: Node("mcu_node"),
  serial_(declare_parameter<std::string>("device", "/dev/ttyACM0"),
    declare_parameter<int>("baud_rate", 115200))
{
  // rosbag2's explicit-topic recorder requests reliable delivery. The MCU
  // stream is local and low bandwidth, so reliable QoS prevents silent bag loss.
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
  state_pub_ = create_publisher<mcu_node::msg::Rx>(
    "/mcu_node/message/rx", qos);
  tx_pub_ = create_publisher<mcu_node::msg::Tx>("/mcu_node/message/tx", qos);
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::QoS(10).reliable());
  publish_raw_ = declare_parameter<bool>("publish_raw", false);
  const double diagnostic_period_s = declare_parameter<double>("diagnostic_period_s", 1.0);
  tx_stale_after_s_ = declare_parameter<double>("tx_stale_after_s", 2.0);
  if (diagnostic_period_s <= 0.0 || tx_stale_after_s_ <= 0.0) {
    throw std::invalid_argument("diagnostic_period_s and tx_stale_after_s must be positive");
  }
  rx_raw_pub_ = create_publisher<mcu_node::msg::RawFrame>(
    "/mcu_node/raw/rx", qos);
  tx_raw_pub_ = create_publisher<mcu_node::msg::RawFrame>(
    "/mcu_node/raw/tx", qos);
  command_sub_ = create_subscription<mcu_node::msg::Tx>(
    "/mcu_node/command", qos,
    [this](const mcu_node::msg::Tx & msg) {
      ++commands_received_;
      last_command_ = std::chrono::steady_clock::now();
      try {
        SendPayload payload{msg.yaw, msg.yaw_vel, msg.yaw_acc, msg.pitch, msg.pitch_vel,
          msg.pitch_acc, msg.enable_lock, msg.enable_shoot, msg.autoaim_living_waterfall_flag};
        const auto frame = encode(payload);
        serial_.write_all(frame);
        publish_raw(frame, mcu_node::msg::RawFrame::TX, tx_raw_pub_);
        auto sent = msg;
        sent.stamp = now();
        tx_pub_->publish(sent);
        ++tx_succeeded_;
        last_tx_success_ = std::chrono::steady_clock::now();
      } catch (const std::exception & e) {
        ++tx_failed_;
        last_tx_failure_ = std::chrono::steady_clock::now();
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000, "serial transmit failed: %s", e.what());
      }
    });
  diagnostics_timer_ = create_wall_timer(
    std::chrono::duration<double>(diagnostic_period_s), [this] {publish_diagnostics();});
  read_thread_ = std::thread([this] {read_loop();});
  RCLCPP_INFO(get_logger(), "mcu_node started: gimbal protocol, 115200 8N1");
}

void McuNode::publish_diagnostics()
{
  const auto current = std::chrono::steady_clock::now();
  const auto age = [current](const std::chrono::steady_clock::time_point time) {
      return time == std::chrono::steady_clock::time_point{} ? -1.0 :
             std::chrono::duration<double>(current - time).count();
    };
  const auto metric = [](const std::string & key, const auto value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = std::to_string(value);
      return item;
    };

  const auto publisher_count = command_sub_->get_publisher_count();
  const auto success_age = age(last_tx_success_);
  const bool latest_event_failed = last_tx_failure_ != std::chrono::steady_clock::time_point{} &&
    (last_tx_success_ == std::chrono::steady_clock::time_point{} ||
    last_tx_failure_ > last_tx_success_);
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "mcu_node/tx";
  status.hardware_id = "gimbal_serial";
  if (latest_event_failed) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "transmit_failed";
  } else if (publisher_count == 0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "no_command_publisher";
  } else if (tx_succeeded_ == 0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "waiting_for_command";
  } else if (success_age > tx_stale_after_s_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "tx_stale";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "tx_live";
  }
  status.values = {
    metric("command_publishers", publisher_count),
    metric("commands_received", commands_received_),
    metric("tx_succeeded", tx_succeeded_),
    metric("tx_failed", tx_failed_),
    metric("last_command_age_s", age(last_command_)),
    metric("last_tx_success_age_s", success_age),
    metric("last_tx_failure_age_s", age(last_tx_failure_)),
  };
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  array.status.push_back(std::move(status));
  diagnostics_pub_->publish(std::move(array));
}

McuNode::~McuNode()
{
  running_ = false;
  serial_.cancel();
  if (read_thread_.joinable()) {read_thread_.join();}
}
std::string McuNode::hex(const std::vector<std::uint8_t> & frame)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto byte : frame) {
    stream << std::setw(2) << static_cast<int>(byte) << ' ';
  }
  return stream.str();
}

void McuNode::publish_raw(
  const std::vector<std::uint8_t> & frame, const std::uint8_t direction,
  const rclcpp::Publisher<RawFrame>::SharedPtr & publisher)
{
  if (!publish_raw_) {return;}
  RawFrame raw;
  raw.stamp = now();
  raw.direction = direction;
  raw.data = frame;
  publisher->publish(raw);
}

void McuNode::read_loop()
{
  while (running_ && rclcpp::ok()) {
    try {
      const auto frame = serial_.read_frame();
      publish_raw(frame, RawFrame::RX, rx_raw_pub_);
      const auto p = decode_receive(frame);
      if (!p) {
        const auto payload_size = frame.size() >= 4 ? frame.size() - 4 : 0;
        const auto expected = frame.size() >= 4 ?
          crc16(std::span<const std::uint8_t>(frame).subspan(2, payload_size)) :
          std::array<std::uint8_t, 2>{0, 0};
        const auto received_low = frame.size() >= 2 ? frame[frame.size() - 2] : 0;
        const auto received_high = frame.size() >= 1 ? frame.back() : 0;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "invalid RX: frame=%zu payload=%zu crc_rx=%02x%02x crc_calc=%02x%02x bytes=[%s]",
          frame.size(), payload_size, received_low, received_high, expected[0], expected[1],
          hex(frame).c_str());
        continue;
      }
      mcu_node::msg::Rx msg;
      msg.stamp = now();
      msg.sequence = ++sequence_;
      msg.device_sample_time_us = p->system_time_point_us;
      msg.imu_yaw = p->imu_yaw;
      msg.imu_pitch = p->imu_pitch;
      msg.imu_roll = p->imu_roll;
      msg.imu_yaw_vel = p->imu_yaw_vel;
      msg.imu_pitch_vel = p->imu_pitch_vel;
      msg.imu_roll_vel = p->imu_roll_vel;
      msg.imu_yaw_acc = p->imu_yaw_acc;
      msg.imu_pitch_acc = p->imu_pitch_acc;
      msg.imu_roll_acc = p->imu_roll_acc;
      msg.imu_quat_w = p->imu_quat_w;
      msg.imu_quat_x = p->imu_quat_x;
      msg.imu_quat_y = p->imu_quat_y;
      msg.imu_quat_z = p->imu_quat_z;
      msg.order = p->order;
      msg.remaining_time_s = p->remaining_time_s;
      msg.sentry_behavior_mode = p->sentry_behavior_mode;
      msg.colour = p->colour;
      state_pub_->publish(msg);
    } catch (const std::exception & e) {
      if (running_) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000, "serial receive failed: %s", e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
  }
}
}  // namespace mcu_node

