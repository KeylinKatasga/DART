#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "dart_coordinate_node/circle_pose.hpp"
#include "dart_coordinate_node/coordinate_math.hpp"
#include "dart_interfaces/msg/coordinate_observation.hpp"
#include "dart_interfaces/msg/green_light.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "mcu_node/msg/rx.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace dart_coordinate_node
{
class DartCoordinateNode final : public rclcpp::Node
{
public:
  DartCoordinateNode() : Node("dart_coordinate_node")
  {
    detection_topic_ = declare_parameter<std::string>("detection_topic", "/detect_node/detection");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/mcu_node/message/rx");
    body_frame_id_ = declare_parameter<std::string>("frames.body", "dart_body");
    world_frame_id_ = declare_parameter<std::string>("frames.world", "world");
    quaternion_semantics_ = declare_parameter<std::string>(
      "quaternion_semantics", "body_to_world_hamilton_wxyz");
    solution_mode_ = declare_parameter<std::string>("solution_mode", "bearing_only");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/camera_node/camera_info");
    target_diameter_m_ = declare_parameter<double>("target_diameter_m", 0.1);
    max_reprojection_error_px_ = declare_parameter<double>(
      "max_reprojection_error_px", 3.0);
    max_imu_age_s_ = declare_parameter<double>("max_imu_age_s", 0.05);
    if (solution_mode_ != "bearing_only" && solution_mode_ != "known_circle_pnp") {
      throw std::invalid_argument("solution_mode must be bearing_only or known_circle_pnp");
    }
    if (quaternion_semantics_ != "body_to_world_hamilton_wxyz") {
      throw std::invalid_argument("unsupported quaternion_semantics");
    }
    if (detection_topic_.empty() || imu_topic_.empty() || body_frame_id_.empty() ||
      world_frame_id_.empty() || camera_info_topic_.empty() || max_imu_age_s_ <= 0.0)
    {
      throw std::invalid_argument("invalid dart_coordinate_node configuration");
    }
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
    circle_pose_solver_ = std::make_unique<CirclePoseSolver>(
      target_diameter_m_, max_reprojection_error_px_);
    observation_pub_ = create_publisher<dart_interfaces::msg::CoordinateObservation>(
      "~/observation", qos);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10).reliable());
    imu_sub_ = create_subscription<mcu_node::msg::Rx>(
      imu_topic_, qos, [this](mcu_node::msg::Rx::ConstSharedPtr message) {
        std::lock_guard lock(mutex_);
        latest_imu_ = std::move(message);
      });
    detection_sub_ = create_subscription<dart_interfaces::msg::GreenLight>(
      detection_topic_, qos,
      [this](const dart_interfaces::msg::GreenLight & message) {on_detection(message);});
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::CameraInfo & message) {
        if (message.k[0] <= 0.0 || message.k[4] <= 0.0) return;
        std::array<double, 9> matrix{};
        std::copy(message.k.begin(), message.k.end(), matrix.begin());
        circle_pose_solver_->set_calibration(matrix, message.d);
      });
    diagnostics_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {publish_diagnostics();});
  }

private:
  void on_detection(const dart_interfaces::msg::GreenLight & detection)
  {
    mcu_node::msg::Rx::ConstSharedPtr imu;
    {
      std::lock_guard lock(mutex_);
      imu = latest_imu_;
    }
    dart_interfaces::msg::CoordinateObservation output;
    output.header = detection.header;
    output.sequence = detection.sequence;
    output.target_detected = detection.detected;
    output.body_frame_id = body_frame_id_;
    output.world_frame_id = world_frame_id_;
    output.bearing_x_rad = detection.bearing_x_rad;
    output.bearing_y_rad = detection.bearing_y_rad;
    output.bearing_rate_x_rad_s = detection.velocity_bearing_x_rad_s;
    output.bearing_rate_y_rad_s = detection.velocity_bearing_y_rad_s;
    output.radius_px = detection.radius_px;
    output.imu_age_s = std::numeric_limits<float>::infinity();
    const auto body_los = bearing_to_body(detection.bearing_x_rad, detection.bearing_y_rad);
    output.los_body_unit = {
      static_cast<float>(body_los.x), static_cast<float>(body_los.y),
      static_cast<float>(body_los.z)};
    if (solution_mode_ == "known_circle_pnp" && detection.detected) {
      const auto pose = circle_pose_solver_->solve(
        detection.center_x_px, detection.center_y_px, detection.radius_px);
      if (pose) {
        // OpenCV optical frame is +Z forward, +X right, +Y down. The aligned dart body is
        // +X forward, +Y up, with image-right mapping to body -Z.
        const Vector3 body_position{
          pose->position_camera_m[2], -pose->position_camera_m[1], -pose->position_camera_m[0]};
        output.position_valid = true;
        output.range_m = static_cast<float>(cv::norm(pose->position_camera_m));
        output.reprojection_error_px = static_cast<float>(pose->reprojection_error_px);
        output.position_camera_m = {
          static_cast<float>(pose->position_camera_m[0]),
          static_cast<float>(pose->position_camera_m[1]),
          static_cast<float>(pose->position_camera_m[2])};
        output.position_body_m = {
          static_cast<float>(body_position.x), static_cast<float>(body_position.y),
          static_cast<float>(body_position.z)};
      }
    }
    if (imu) {
      const auto age = std::abs(
        (rclcpp::Time(detection.header.stamp) - rclcpp::Time(imu->stamp)).seconds());
      output.imu_age_s = static_cast<float>(age);
      const auto quaternion = normalized(Quaternion{
        imu->imu_quat_w, imu->imu_quat_x, imu->imu_quat_y, imu->imu_quat_z});
      output.imu_valid = age <= max_imu_age_s_ && quaternion.has_value();
      if (output.imu_valid) {
        output.imu_quaternion_w = static_cast<float>(quaternion->w);
        output.imu_quaternion_x = static_cast<float>(quaternion->x);
        output.imu_quaternion_y = static_cast<float>(quaternion->y);
        output.imu_quaternion_z = static_cast<float>(quaternion->z);
        const auto world_los = rotate_body_to_world(*quaternion, body_los);
        output.los_world_unit = {
          static_cast<float>(world_los.x), static_cast<float>(world_los.y),
          static_cast<float>(world_los.z)};
        if (output.position_valid) {
          const Vector3 body_position{
            output.position_body_m[0], output.position_body_m[1], output.position_body_m[2]};
          const auto world_position = rotate_body_to_world(*quaternion, body_position);
          output.position_world_m = {
            static_cast<float>(world_position.x), static_cast<float>(world_position.y),
            static_cast<float>(world_position.z)};
        }
        output.vehicle_x_vel = imu->vehicle_x_vel;
        output.vehicle_y_vel = imu->vehicle_y_vel;
        output.angle1 = imu->angle1;
        output.angle2 = imu->angle2;
        output.angle3 = imu->angle3;
        output.angle4 = imu->angle4;
      }
    }
    output.valid = output.target_detected && output.imu_valid;
    observation_pub_->publish(output);
    ++observations_;
    last_valid_ = output.valid;
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "dart_coordinate_node/status";
    status.hardware_id = "aligned_camera_imu";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = last_valid_ ? solution_mode_ + "_observation_valid" :
      "waiting_for_valid_observation";
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(std::move(array));
  }

  std::string detection_topic_;
  std::string imu_topic_;
  std::string body_frame_id_;
  std::string world_frame_id_;
  std::string solution_mode_;
  std::string camera_info_topic_;
  std::string quaternion_semantics_;
  double max_imu_age_s_{0.05};
  double target_diameter_m_{0.1};
  double max_reprojection_error_px_{3.0};
  std::unique_ptr<CirclePoseSolver> circle_pose_solver_;
  std::mutex mutex_;
  mcu_node::msg::Rx::ConstSharedPtr latest_imu_;
  std::uint64_t observations_{0};
  bool last_valid_{false};
  rclcpp::Subscription<mcu_node::msg::Rx>::SharedPtr imu_sub_;
  rclcpp::Subscription<dart_interfaces::msg::GreenLight>::SharedPtr detection_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<dart_interfaces::msg::CoordinateObservation>::SharedPtr observation_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};
}  // namespace dart_coordinate_node

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dart_coordinate_node::DartCoordinateNode>());
  rclcpp::shutdown();
  return 0;
}
#include <algorithm>
#include <array>
