#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "dart_interfaces/msg/green_light.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "detect_node/detector.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace detect_node
{
namespace
{
using SteadyClock = std::chrono::steady_clock;

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

struct Calibration
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  std::vector<double> distortion;
  std::uint32_t width{0};
  std::uint32_t height{0};
  bool from_camera_info{false};
};

struct PreviousTarget
{
  cv::Point2f center;
  double bearing_x{0.0};
  double bearing_y{0.0};
  rclcpp::Time stamp;
};
}  // namespace

class GreenLightNode final : public rclcpp::Node
{
public:
  GreenLightNode()
  : Node("detect_node"), detector_(read_detector_config()), stats_start_(SteadyClock::now())
  {
    const auto image_topic = declare_parameter<std::string>(
      "image_topic", "/camera_node/image_raw");
    const auto camera_info_topic = declare_parameter<std::string>(
      "camera_info_topic", "/camera_node/camera_info");
    const auto input_depth = declare_parameter<int>("input_queue_depth", 1);
    debug_image_ = declare_parameter<bool>("debug_image", true);
    max_track_gap_s_ = declare_parameter<double>("max_track_gap_s", 0.2);
    use_camera_info_ = declare_parameter<bool>("use_camera_info", true);
    fallback_.fx = declare_parameter<double>("fallback_intrinsics.fx", 1785.4881526822305);
    fallback_.fy = declare_parameter<double>("fallback_intrinsics.fy", 1785.026019470562);
    fallback_.cx = declare_parameter<double>("fallback_intrinsics.cx", 672.4806478241826);
    fallback_.cy = declare_parameter<double>("fallback_intrinsics.cy", 559.8960322479431);
    fallback_.width = static_cast<std::uint32_t>(
      declare_parameter<int>("fallback_intrinsics.width", 1280));
    fallback_.height = static_cast<std::uint32_t>(
      declare_parameter<int>("fallback_intrinsics.height", 1024));
    const auto diagnostics_period = declare_parameter<double>("diagnostic_period_s", 1.0);
    if (image_topic.empty() || camera_info_topic.empty() || input_depth <= 0 ||
      max_track_gap_s_ <= 0.0 || fallback_.fx <= 0.0 || fallback_.fy <= 0.0 ||
      fallback_.width == 0 || fallback_.height == 0 || diagnostics_period <= 0.0)
    {
      throw std::invalid_argument("invalid green-light node configuration");
    }

    detection_pub_ = create_publisher<dart_interfaces::msg::GreenLight>(
      "~/detection", rclcpp::QoS(10).reliable());
    if (debug_image_) {
      debug_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "~/debug_image", rclcpp::SensorDataQoS());
    }
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10).reliable());
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
        receive_camera_info(*message);
      });
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS().keep_last(input_depth),
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {process(std::move(message));});
    diagnostics_timer_ = create_wall_timer(
      std::chrono::duration<double>(diagnostics_period), [this] {publish_diagnostics();});

    RCLCPP_INFO(
      get_logger(), "started: image=%s camera_info=%s output=%s SN profile=%s",
      image_topic.c_str(), camera_info_topic.c_str(),
      get_fully_qualified_name(), "standard3/KE0210080605");
  }

private:
  DetectorConfig read_detector_config()
  {
    DetectorConfig config;
    config.hue_min = declare_parameter<int>("threshold.hue_min", config.hue_min);
    config.hue_max = declare_parameter<int>("threshold.hue_max", config.hue_max);
    config.saturation_min = declare_parameter<int>(
      "threshold.saturation_min", config.saturation_min);
    config.value_min = declare_parameter<int>("threshold.value_min", config.value_min);
    config.min_area_px = declare_parameter<double>("filter.min_area_px", config.min_area_px);
    config.max_area_px = declare_parameter<double>("filter.max_area_px", config.max_area_px);
    config.min_radius_px = declare_parameter<double>(
      "filter.min_radius_px", config.min_radius_px);
    config.max_radius_px = declare_parameter<double>(
      "filter.max_radius_px", config.max_radius_px);
    config.min_circularity = declare_parameter<double>(
      "filter.min_circularity", config.min_circularity);
    config.min_fill_ratio = declare_parameter<double>(
      "filter.min_fill_ratio", config.min_fill_ratio);
    config.max_aspect_ratio = declare_parameter<double>(
      "filter.max_aspect_ratio", config.max_aspect_ratio);
    config.morphology_kernel = declare_parameter<int>(
      "morphology.kernel", config.morphology_kernel);
    config.open_iterations = declare_parameter<int>(
      "morphology.open_iterations", config.open_iterations);
    config.close_iterations = declare_parameter<int>(
      "morphology.close_iterations", config.close_iterations);
    return config;
  }

  void receive_camera_info(const sensor_msgs::msg::CameraInfo & message)
  {
    if (!use_camera_info_ || message.width == 0 || message.height == 0 ||
      message.k[0] <= 0.0 || message.k[4] <= 0.0)
    {
      return;
    }
    Calibration calibration;
    calibration.fx = message.k[0];
    calibration.fy = message.k[4];
    calibration.cx = message.k[2];
    calibration.cy = message.k[5];
    calibration.distortion = message.d;
    calibration.width = message.width;
    calibration.height = message.height;
    calibration.from_camera_info = true;
    std::lock_guard lock(calibration_mutex_);
    calibration_ = std::move(calibration);
  }

  Calibration calibration_for(const std::uint32_t width, const std::uint32_t height)
  {
    Calibration calibration;
    {
      std::lock_guard lock(calibration_mutex_);
      calibration = calibration_.value_or(fallback_);
    }
    if (calibration.width != width || calibration.height != height) {
      const double scale_x = static_cast<double>(width) / calibration.width;
      const double scale_y = static_cast<double>(height) / calibration.height;
      calibration.fx *= scale_x;
      calibration.cx *= scale_x;
      calibration.fy *= scale_y;
      calibration.cy *= scale_y;
      calibration.width = width;
      calibration.height = height;
    }
    return calibration;
  }

  std::pair<double, double> bearing(
    const cv::Point2f & center, const Calibration & calibration) const
  {
    double normalized_x = (center.x - calibration.cx) / calibration.fx;
    double normalized_y = (center.y - calibration.cy) / calibration.fy;
    if (calibration.from_camera_info && !calibration.distortion.empty()) {
      const cv::Matx33d camera_matrix{
        calibration.fx, 0.0, calibration.cx,
        0.0, calibration.fy, calibration.cy,
        0.0, 0.0, 1.0};
      std::vector<cv::Point2f> source{center};
      std::vector<cv::Point2f> corrected;
      cv::undistortPoints(source, corrected, camera_matrix, calibration.distortion);
      if (!corrected.empty()) {
        normalized_x = corrected.front().x;
        normalized_y = corrected.front().y;
      }
    }
    return {std::atan(normalized_x), std::atan(-normalized_y)};
  }

  void process(sensor_msgs::msg::Image::ConstSharedPtr image)
  {
    const auto started = SteadyClock::now();
    dart_interfaces::msg::GreenLight output;
    output.header = image->header;
    output.sequence = ++sequence_;
    try {
      if (image->encoding != "bgr8" || image->width == 0 || image->height == 0 ||
        image->step < image->width * 3 ||
        image->data.size() < static_cast<std::size_t>(image->step) * image->height)
      {
        throw std::runtime_error("expected a valid bgr8 sensor_msgs/Image");
      }
      const cv::Mat view(
        static_cast<int>(image->height), static_cast<int>(image->width), CV_8UC3,
        const_cast<std::uint8_t *>(image->data.data()), image->step);
      const auto detection = detector_.detect(view);
      output.detected = detection.has_value();
      if (detection) {
        const auto calibration = calibration_for(image->width, image->height);
        const auto [bearing_x, bearing_y] = bearing(detection->center, calibration);
        output.center_x_px = detection->center.x;
        output.center_y_px = detection->center.y;
        output.radius_px = detection->radius;
        output.bearing_x_rad = static_cast<float>(bearing_x);
        output.bearing_y_rad = static_cast<float>(bearing_y);
        output.circularity = detection->circularity;
        output.fill_ratio = detection->fill_ratio;
        output.score = detection->score;

        const rclcpp::Time stamp(image->header.stamp);
        if (previous_) {
          const double elapsed = (stamp - previous_->stamp).seconds();
          if (elapsed > 0.0 && elapsed <= max_track_gap_s_) {
            output.delta_valid = true;
            output.delta_x_px = detection->center.x - previous_->center.x;
            output.delta_y_px = detection->center.y - previous_->center.y;
            output.velocity_x_px_s = static_cast<float>(output.delta_x_px / elapsed);
            output.velocity_y_px_s = static_cast<float>(output.delta_y_px / elapsed);
            output.delta_bearing_x_rad = static_cast<float>(bearing_x - previous_->bearing_x);
            output.delta_bearing_y_rad = static_cast<float>(bearing_y - previous_->bearing_y);
            output.velocity_bearing_x_rad_s = static_cast<float>(
              output.delta_bearing_x_rad / elapsed);
            output.velocity_bearing_y_rad_s = static_cast<float>(
              output.delta_bearing_y_rad / elapsed);
          }
        }
        previous_ = PreviousTarget{detection->center, bearing_x, bearing_y, stamp};
        ++detected_frames_;
      } else {
        previous_.reset();
      }

      output.processing_ms = static_cast<float>(
        std::chrono::duration<double, std::milli>(SteadyClock::now() - started).count());
      detection_pub_->publish(output);
      if (debug_pub_ && debug_pub_->get_subscription_count() > 0) {
        publish_debug(*image, view, detection, output);
      }
      ++processed_frames_;
      last_processing_ms_ = output.processing_ms;
      last_detected_ = output.detected;
      last_error_.clear();
    } catch (const std::exception & error) {
      previous_.reset();
      output.processing_ms = static_cast<float>(
        std::chrono::duration<double, std::milli>(SteadyClock::now() - started).count());
      detection_pub_->publish(output);
      ++processed_frames_;
      ++processing_errors_;
      last_processing_ms_ = output.processing_ms;
      last_detected_ = false;
      last_error_ = error.what();
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
    }
  }

  void publish_debug(
    const sensor_msgs::msg::Image & source, const cv::Mat & view,
    const std::optional<CircleDetection> & detection,
    const dart_interfaces::msg::GreenLight & result)
  {
    cv::Mat overlay = view.clone();
    if (detection) {
      cv::circle(overlay, detection->center, cvRound(detection->radius), {0, 255, 255}, 2);
      cv::drawMarker(overlay, detection->center, {0, 0, 255}, cv::MARKER_CROSS, 16, 2);
      const std::string line = cv::format(
        "u=%.1f v=%.1f r=%.1f dx=%.1f dy=%.1f yaw=%.3f pitch=%.3f",
        result.center_x_px, result.center_y_px, result.radius_px,
        result.delta_x_px, result.delta_y_px,
        result.bearing_x_rad, result.bearing_y_rad);
      cv::putText(
        overlay, line, {10, 32}, cv::FONT_HERSHEY_SIMPLEX, 0.65, {0, 255, 255}, 2);
    } else {
      cv::putText(
        overlay, "green light: not detected", {10, 32},
        cv::FONT_HERSHEY_SIMPLEX, 0.65, {0, 0, 255}, 2);
    }
    sensor_msgs::msg::Image message;
    message.header = source.header;
    message.height = source.height;
    message.width = source.width;
    message.encoding = "bgr8";
    message.is_bigendian = false;
    message.step = source.width * 3;
    message.data.assign(overlay.datastart, overlay.dataend);
    debug_pub_->publish(std::move(message));
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "detect_node/status";
    status.hardware_id = "opencv/standard3/KE0210080605";
    status.level = last_error_.empty() ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = last_error_.empty() ? (last_detected_ ? "target_detected" : "searching") :
      "processing_error";
    const auto current = SteadyClock::now();
    const double elapsed = std::max(
      0.001, std::chrono::duration<double>(current - stats_start_).count());
    status.values = {
      value("processing_fps", static_cast<double>(processed_frames_ - window_processed_) / elapsed),
      value("processed_frames", processed_frames_),
      value("detected_frames", detected_frames_),
      value("processing_errors", processing_errors_),
      value("processing_ms", last_processing_ms_),
      text_value("intrinsics_source", calibration_ ? "camera_info" : "standard3_fallback"),
      text_value("last_error", last_error_)};
    window_processed_ = processed_frames_;
    stats_start_ = current;
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(std::move(array));
  }

  Detector detector_;
  bool debug_image_{true};
  bool use_camera_info_{true};
  double max_track_gap_s_{0.2};
  Calibration fallback_;
  std::optional<Calibration> calibration_;
  std::mutex calibration_mutex_;
  std::optional<PreviousTarget> previous_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<dart_interfaces::msg::GreenLight>::SharedPtr detection_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  SteadyClock::time_point stats_start_;
  std::uint64_t sequence_{0};
  std::uint64_t processed_frames_{0};
  std::uint64_t detected_frames_{0};
  std::uint64_t processing_errors_{0};
  std::uint64_t window_processed_{0};
  double last_processing_ms_{0.0};
  bool last_detected_{false};
  std::string last_error_;
};

}  // namespace detect_node

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<detect_node::GreenLightNode>());
  rclcpp::shutdown();
  return 0;
}
