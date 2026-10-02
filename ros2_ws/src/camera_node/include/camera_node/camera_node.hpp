#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "camera_info_manager/camera_info_manager.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "camera_node/camera.hpp"

namespace camera_node
{
using CameraFactory = std::function<std::unique_ptr<Camera>(const CameraConfig &)>;

class CameraNode final : public rclcpp::Node
{
public:
  explicit CameraNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    CameraFactory factory = make_daheng_camera);
  ~CameraNode() override;

private:
  CameraConfig read_config();
  void acquisition_loop();
  void connect();
  void publish_frame(CameraFrame && frame);
  void publish_diagnostics();
  rclcpp::Time ros_stamp(const CameraFrame & frame) const;

  CameraConfig config_;
  CameraFactory factory_;
  std::unique_ptr<Camera> camera_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> info_manager_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::thread acquisition_thread_;
  std::atomic_bool running_{true};
  mutable std::mutex state_mutex_;
  std::string frame_id_;
  std::string camera_info_url_;
  std::string connection_state_{"starting"};
  std::string last_error_;
  std::string serial_number_;
  std::string sdk_version_;
  std::string effective_config_;
  std::string time_source_{"host_receive"};
  double reconnect_interval_s_{2.0};
  double publish_rate_limit_hz_{0.0};
  bool calibration_valid_{false};
  std::uint64_t captured_{0};
  std::uint64_t published_{0};
  std::uint64_t timeouts_{0};
  std::uint64_t reconnects_{0};
  std::uint64_t frame_gaps_{0};
  std::uint64_t last_sequence_{0};
  std::uint64_t last_device_ticks_{0};
  double conversion_ms_{0.0};
  double processing_ms_{0.0};
  double jitter_ms_{-1.0};
  std::uint64_t window_captured_{0};
  std::uint64_t window_published_{0};
  std::chrono::steady_clock::time_point stats_window_start_;
  std::chrono::steady_clock::time_point last_capture_;
  std::chrono::steady_clock::time_point last_publish_;
};
}  // namespace camera_node

