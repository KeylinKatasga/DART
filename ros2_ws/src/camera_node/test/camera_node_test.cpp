#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <thread>

#include "gtest/gtest.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "camera_node/camera_node.hpp"

using namespace std::chrono_literals;

namespace
{
class FakeCamera final : public camera_node::Camera
{
public:
  camera_node::GrabResult grab(camera_node::CameraFrame & frame) override
  {
    std::this_thread::sleep_for(5ms);
    frame.width = 2;
    frame.height = 1;
    frame.bgr = {1, 2, 3, 4, 5, 6};
    frame.sequence = ++sequence_;
    frame.host_receive_time = std::chrono::steady_clock::now();
    frame.acquisition_time = frame.host_receive_time;
    frame.time_source = "host_receive";
    return camera_node::GrabResult::frame;
  }
  std::string serial_number() const override {return "FAKE-001";}
  std::string sdk_version() const override {return "test";}

private:
  std::uint64_t sequence_{0};
};
}  // namespace

TEST(CameraConfig, RejectsInvalidValues)
{
  camera_node::CameraConfig config;
  config.roi_width = 0;
  EXPECT_THROW(camera_node::validate_config(config), std::invalid_argument);
  config.roi_width = 640;
  config.auto_exposure_min_us = 2000.0;
  config.auto_exposure_max_us = 1000.0;
  EXPECT_THROW(camera_node::validate_config(config), std::invalid_argument);
}

TEST(CameraNode, PublishesMatchingStandardMessages)
{
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("roi_width", 2),
    rclcpp::Parameter("roi_height", 1),
    rclcpp::Parameter("white_balance", false),
    rclcpp::Parameter("image_queue_depth", 2)});
  auto camera = std::make_shared<camera_node::CameraNode>(
    options, [](const camera_node::CameraConfig &) {return std::make_unique<FakeCamera>();});
  auto listener = std::make_shared<rclcpp::Node>("camera_test_listener");
  std::promise<sensor_msgs::msg::Image> image_promise;
  std::promise<sensor_msgs::msg::CameraInfo> info_promise;
  std::promise<diagnostic_msgs::msg::DiagnosticArray> diagnostics_promise;
  auto image_future = image_promise.get_future();
  auto info_future = info_promise.get_future();
  auto diagnostics_future = diagnostics_promise.get_future();
  std::atomic_bool got_image{false};
  std::atomic_bool got_info{false};
  std::atomic_bool got_diagnostics{false};
  auto qos = rclcpp::SensorDataQoS();
  auto image_sub = listener->create_subscription<sensor_msgs::msg::Image>(
    "/camera_node/image_raw", qos,
    [&](sensor_msgs::msg::Image::ConstSharedPtr message) {
      if (!got_image.exchange(true)) image_promise.set_value(*message);
    });
  auto info_sub = listener->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_node/camera_info", qos,
    [&](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
      if (!got_info.exchange(true)) info_promise.set_value(*message);
    });
  auto diagnostics_sub = listener->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10,
    [&](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
      if (!got_diagnostics.exchange(true)) diagnostics_promise.set_value(*message);
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(camera);
  executor.add_node(listener);
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while ((image_future.wait_for(0s) != std::future_status::ready ||
    info_future.wait_for(0s) != std::future_status::ready ||
    diagnostics_future.wait_for(0s) != std::future_status::ready) &&
    std::chrono::steady_clock::now() < deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_EQ(image_future.wait_for(0s), std::future_status::ready);
  ASSERT_EQ(info_future.wait_for(0s), std::future_status::ready);
  ASSERT_EQ(diagnostics_future.wait_for(0s), std::future_status::ready);
  const auto image = image_future.get();
  const auto info = info_future.get();
  const auto diagnostics = diagnostics_future.get();
  EXPECT_EQ(image.encoding, "bgr8");
  EXPECT_EQ(image.width, 2U);
  EXPECT_EQ(image.height, 1U);
  EXPECT_EQ(image.step, 6U);
  EXPECT_EQ(image.data, (std::vector<std::uint8_t>{1, 2, 3, 4, 5, 6}));
  EXPECT_EQ(image.header.frame_id, "camera_optical_frame");
  EXPECT_EQ(image.header, info.header);
  EXPECT_EQ(info.width, image.width);
  EXPECT_EQ(info.height, image.height);
  ASSERT_FALSE(diagnostics.status.empty());
  const auto & values = diagnostics.status.front().values;
  const auto has_key = [&values](const std::string & key) {
      return std::any_of(values.begin(), values.end(), [&](const auto & item) {return item.key == key;});
    };
  EXPECT_TRUE(has_key("capture_fps"));
  EXPECT_TRUE(has_key("publish_fps"));
  EXPECT_TRUE(has_key("processing_ms"));
  (void)image_sub;
  (void)info_sub;
  (void)diagnostics_sub;
  executor.remove_node(listener);
  executor.remove_node(camera);
  camera.reset();
  listener.reset();
  rclcpp::shutdown();
}

