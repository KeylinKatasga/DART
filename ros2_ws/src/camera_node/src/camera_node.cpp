#include "camera_node/camera_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace camera_node
{
#ifndef CAMERA_NODE_GIT_REVISION
#define CAMERA_NODE_GIT_REVISION "unknown"
#endif
namespace
{
diagnostic_msgs::msg::KeyValue value(const std::string & key, const auto & data)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = std::to_string(data);
  return item;
}

diagnostic_msgs::msg::KeyValue text_value(const std::string & key, const std::string & data)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = data;
  return item;
}
}  // namespace

CameraNode::CameraNode(const rclcpp::NodeOptions & options, CameraFactory factory)
: Node("camera_node", options), factory_(std::move(factory)),
  stats_window_start_(std::chrono::steady_clock::now())
{
  config_ = read_config();
  validate_config(config_);
  frame_id_ = declare_parameter<std::string>("frame_id", "camera_optical_frame");
  camera_info_url_ = declare_parameter<std::string>("camera_info_url", "");
  reconnect_interval_s_ = declare_parameter<double>("reconnect_interval_s", 2.0);
  publish_rate_limit_hz_ = declare_parameter<double>("publish_rate_limit_hz", 0.0);
  const auto diagnostics_period = declare_parameter<double>("diagnostic_period_s", 1.0);
  const auto queue_depth = declare_parameter<int>("image_queue_depth", 5);
  const auto qos_mode = declare_parameter<std::string>("image_qos", "sensor_data");
  if (frame_id_.empty() || reconnect_interval_s_ <= 0.0 || diagnostics_period <= 0.0 ||
    publish_rate_limit_hz_ < 0.0 || queue_depth <= 0)
  {
    throw std::invalid_argument("invalid frame_id or runtime/publishing parameter");
  }
  if (qos_mode != "sensor_data" && qos_mode != "reliable") {
    throw std::invalid_argument("image_qos must be 'sensor_data' or 'reliable'");
  }
  auto image_qos = rclcpp::QoS(rclcpp::KeepLast(queue_depth));
  if (qos_mode == "sensor_data") {
    image_qos.best_effort().durability_volatile();
  } else {
    image_qos.reliable().durability_volatile();
  }
  image_pub_ = create_publisher<sensor_msgs::msg::Image>("~/image_raw", image_qos);
  info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info", image_qos);
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::QoS(10).reliable());
  if (!camera_info_url_.empty()) {
    info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
      this, get_name(), camera_info_url_);
    calibration_valid_ = info_manager_->isCalibrated();
  }

  diagnostics_timer_ = create_wall_timer(
    std::chrono::duration<double>(diagnostics_period),
    [this] {publish_diagnostics();});
  RCLCPP_INFO(
    get_logger(),
    "requested config: sn=%s roi=%ldx%ld+%ld+%ld exposure=%s/%.1fus gain=%s/%.2f "
    "white_balance=%s flip=%s qos=%s depth=%ld",
    config_.camera_sn.empty() ? "auto" : config_.camera_sn.c_str(),
    config_.roi_width, config_.roi_height, config_.offset_x, config_.offset_y,
    config_.auto_exposure ? "auto" : "manual", config_.exposure_time_us,
    config_.auto_gain ? "auto" : "manual", config_.gain,
    config_.white_balance ? "on" : "off", config_.flip_image ? "180deg" : "off",
    qos_mode.c_str(), queue_depth);
  acquisition_thread_ = std::thread([this] {acquisition_loop();});
}

CameraNode::~CameraNode()
{
  running_ = false;
  if (acquisition_thread_.joinable()) acquisition_thread_.join();
  camera_.reset();
  RCLCPP_INFO(get_logger(), "camera stopped");
}

CameraConfig CameraNode::read_config()
{
  CameraConfig config;
  config.camera_sn = declare_parameter<std::string>("camera_sn", "");
  config.roi_width = declare_parameter<std::int64_t>("roi_width", config.roi_width);
  config.roi_height = declare_parameter<std::int64_t>("roi_height", config.roi_height);
  config.offset_x = declare_parameter<std::int64_t>("offset_x", config.offset_x);
  config.offset_y = declare_parameter<std::int64_t>("offset_y", config.offset_y);
  config.auto_exposure = declare_parameter<bool>("auto_exposure", config.auto_exposure);
  config.exposure_time_us = declare_parameter<double>(
    "exposure_time_us", config.exposure_time_us);
  config.auto_exposure_min_us = declare_parameter<double>(
    "auto_exposure_min_us", config.auto_exposure_min_us);
  config.auto_exposure_max_us = declare_parameter<double>(
    "auto_exposure_max_us", config.auto_exposure_max_us);
  config.auto_gain = declare_parameter<bool>("auto_gain", config.auto_gain);
  config.gain = declare_parameter<double>("gain", config.gain);
  config.auto_gain_min = declare_parameter<double>("auto_gain_min", config.auto_gain_min);
  config.auto_gain_max = declare_parameter<double>("auto_gain_max", config.auto_gain_max);
  config.gray_value = declare_parameter<std::int64_t>("gray_value", config.gray_value);
  config.white_balance = declare_parameter<bool>("white_balance", config.white_balance);
  config.flip_image = declare_parameter<bool>("flip_image", config.flip_image);
  const auto timeout = declare_parameter<int>("grab_timeout_ms", config.grab_timeout_ms);
  if (timeout <= 0) throw std::invalid_argument("grab_timeout_ms must be positive");
  config.grab_timeout_ms = static_cast<std::uint32_t>(timeout);
  return config;
}

void CameraNode::connect()
{
  auto camera = factory_(config_);
  const auto serial = camera->serial_number();
  const auto sdk = camera->sdk_version();
  const auto effective = camera->configuration_summary();
  camera_ = std::move(camera);
  {
    std::lock_guard lock(state_mutex_);
    connection_state_ = "connected";
    last_error_.clear();
    serial_number_ = serial;
    sdk_version_ = sdk;
    effective_config_ = effective;
    ++reconnects_;
    last_sequence_ = 0;
  }
  RCLCPP_INFO(
    get_logger(), "camera connected: serial=%s sdk=%s effective_config={%s}",
    serial.c_str(), sdk.c_str(), effective.c_str());
}

void CameraNode::acquisition_loop()
{
  while (running_) {
    try {
      if (!camera_) connect();
      CameraFrame frame;
      if (camera_->grab(frame) == GrabResult::timeout) {
        std::lock_guard lock(state_mutex_);
        ++timeouts_;
        continue;
      }
      publish_frame(std::move(frame));
    } catch (const std::exception & error) {
      {
        std::lock_guard lock(state_mutex_);
        connection_state_ = "disconnected";
        last_error_ = error.what();
      }
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "camera unavailable: %s", error.what());
      camera_.reset();
      const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(reconnect_interval_s_);
      while (running_ && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
  }
}

rclcpp::Time CameraNode::ros_stamp(const CameraFrame & frame) const
{
  const auto steady_now = std::chrono::steady_clock::now();
  auto age = steady_now - frame.acquisition_time;
  if (age < std::chrono::steady_clock::duration::zero() || age > std::chrono::seconds(10)) {
    age = std::chrono::steady_clock::duration::zero();
  }
  return now() - rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(age));
}

void CameraNode::publish_frame(CameraFrame && frame)
{
  if (frame.width == 0 || frame.height == 0 ||
    frame.bgr.size() != static_cast<std::size_t>(frame.width) * frame.height * 3)
  {
    throw std::runtime_error("camera returned an invalid BGR8 frame");
  }
  const auto current = std::chrono::steady_clock::now();
  const bool limited = publish_rate_limit_hz_ > 0.0 &&
    last_publish_ != std::chrono::steady_clock::time_point{} &&
    current - last_publish_ < std::chrono::duration<double>(1.0 / publish_rate_limit_hz_);
  {
    std::lock_guard lock(state_mutex_);
    ++captured_;
    if (last_sequence_ != 0 && frame.sequence > last_sequence_ + 1) {
      frame_gaps_ += frame.sequence - last_sequence_ - 1;
    }
    last_sequence_ = frame.sequence;
    conversion_ms_ = frame.conversion_ms;
    time_source_ = frame.time_source;
    if (last_capture_ != std::chrono::steady_clock::time_point{} &&
      frame.device_tick_frequency_hz > 0.0 && frame.device_timestamp_ticks > last_device_ticks_ &&
      last_device_ticks_ > 0)
    {
      const double host_ms = std::chrono::duration<double, std::milli>(
        frame.host_receive_time - last_capture_).count();
      const double device_ms = static_cast<double>(
        frame.device_timestamp_ticks - last_device_ticks_) * 1000.0 /
        frame.device_tick_frequency_hz;
      const double sample = std::abs(host_ms - device_ms);
      jitter_ms_ = jitter_ms_ < 0.0 ? sample : 0.9 * jitter_ms_ + 0.1 * sample;
    }
    last_capture_ = frame.host_receive_time;
    last_device_ticks_ = frame.device_timestamp_ticks;
  }
  if (limited) return;

  const auto stamp = ros_stamp(frame);
  sensor_msgs::msg::Image image;
  image.header.stamp = stamp;
  image.header.frame_id = frame_id_;
  image.height = frame.height;
  image.width = frame.width;
  image.encoding = "bgr8";
  image.is_bigendian = false;
  image.step = frame.width * 3;
  image.data = std::move(frame.bgr);

  auto info = info_manager_ ? info_manager_->getCameraInfo() : sensor_msgs::msg::CameraInfo{};
  const bool dimensions_match = info.width == 0 ||
    (info.width == image.width && info.height == image.height);
  bool calibration_valid = false;
  {
    std::lock_guard lock(state_mutex_);
    calibration_valid_ = calibration_valid_ && dimensions_match;
    calibration_valid = calibration_valid_;
  }
  if (!calibration_valid) {
    info = sensor_msgs::msg::CameraInfo{};
    info.width = image.width;
    info.height = image.height;
    info.roi.x_offset = static_cast<std::uint32_t>(config_.offset_x);
    info.roi.y_offset = static_cast<std::uint32_t>(config_.offset_y);
    info.roi.width = image.width;
    info.roi.height = image.height;
    info.roi.do_rectify = false;
  }
  info.header = image.header;
  image_pub_->publish(std::move(image));
  info_pub_->publish(std::move(info));
  {
    std::lock_guard lock(state_mutex_);
    ++published_;
    last_publish_ = current;
    processing_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - frame.host_receive_time).count();
  }
}

void CameraNode::publish_diagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "camera_node/status";
  const auto current = std::chrono::steady_clock::now();
  {
    std::lock_guard lock(state_mutex_);
    const auto elapsed = std::max(
      0.001, std::chrono::duration<double>(current - stats_window_start_).count());
    const auto capture_fps = static_cast<double>(captured_ - window_captured_) / elapsed;
    const auto publish_fps = static_cast<double>(published_ - window_published_) / elapsed;
    window_captured_ = captured_;
    window_published_ = published_;
    stats_window_start_ = current;
    status.hardware_id = serial_number_.empty() ? "daheng:unknown" : "daheng:" + serial_number_;
    status.level = connection_state_ == "connected" ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = connection_state_;
    status.values = {
      text_value("connection", connection_state_),
      text_value("last_error", last_error_),
      text_value("serial_number", serial_number_),
      text_value("sdk_version", sdk_version_),
      text_value("source_revision", CAMERA_NODE_GIT_REVISION),
      text_value("effective_config", effective_config_),
      text_value("time_source", time_source_),
      text_value("calibration", calibration_valid_ ? "valid" : "uncalibrated"),
      value("capture_fps", capture_fps),
      value("publish_fps", publish_fps),
      value("captured_frames", captured_),
      value("published_frames", published_),
      value("timeouts", timeouts_),
      value("frame_gaps", frame_gaps_),
      value("reconnects", reconnects_),
      value("processing_ms", processing_ms_),
      value("conversion_ms", conversion_ms_),
      value("arrival_jitter_ms", jitter_ms_)};
  }
  message.status.push_back(std::move(status));
  diagnostics_pub_->publish(std::move(message));
}
}  // namespace camera_node

