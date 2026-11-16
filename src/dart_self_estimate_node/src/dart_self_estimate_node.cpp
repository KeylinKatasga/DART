#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "dart_interfaces/msg/fin_command.hpp"
#include "dart_interfaces/msg/self_state.hpp"
#include "dart_self_estimate_node/self_ekf.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace dart_self_estimate_node
{
namespace
{
template<int Rows, int Columns>
Eigen::Matrix<double, Rows, Columns> matrix_parameter(
  rclcpp::Node & node, const std::string & name,
  const Eigen::Matrix<double, Rows, Columns> & defaults)
{
  std::vector<double> default_values(defaults.data(), defaults.data() + defaults.size());
  const auto values = node.declare_parameter<std::vector<double>>(name, default_values);
  if (values.size() != static_cast<std::size_t>(Rows * Columns)) {
    throw std::invalid_argument(name + " has invalid length");
  }
  Eigen::Matrix<double, Rows, Columns> result;
  for (int row = 0; row < Rows; ++row) {
    for (int column = 0; column < Columns; ++column) {
      result(row, column) = values[static_cast<std::size_t>(row * Columns + column)];
    }
  }
  return result;
}
}  // namespace

class DartSelfEstimateNode final : public rclcpp::Node
{
public:
  DartSelfEstimateNode() : Node("dart_self_estimate_node")
  {
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
    fin_input_topic_ = declare_parameter<std::string>(
      "fin_input_topic", "/mcu_node/message/fin_tx");
    SelfEkfConfig config;
    config.mass_kg = declare_parameter<double>("dynamics.mass_kg", config.mass_kg);
    config.inertia_diagonal = matrix_parameter<3, 1>(
      *this, "dynamics.inertia_diagonal_kg_m2", config.inertia_diagonal);
    config.moment_allocation = matrix_parameter<3, 4>(
      *this, "dynamics.moment_allocation", config.moment_allocation);
    config.force_allocation = matrix_parameter<3, 4>(
      *this, "dynamics.force_allocation", config.force_allocation);
    config.moment_bias = matrix_parameter<3, 1>(
      *this, "dynamics.moment_bias_nm", config.moment_bias);
    config.force_bias = matrix_parameter<3, 1>(
      *this, "dynamics.force_bias_n", config.force_bias);
    config.gravity_world = matrix_parameter<3, 1>(
      *this, "measurement.gravity_world_m_s2", config.gravity_world);
    config.include_gravity_projection = declare_parameter<bool>(
      "measurement.include_gravity_projection", config.include_gravity_projection);
    config.quaternion_process_variance = declare_parameter<double>(
      "noise.quaternion_process_variance", config.quaternion_process_variance);
    config.angular_velocity_process_variance = declare_parameter<double>(
      "noise.angular_velocity_process_variance", config.angular_velocity_process_variance);
    config.accelerometer_variance = declare_parameter<double>(
      "noise.accelerometer_variance", config.accelerometer_variance);
    config.gyroscope_variance = declare_parameter<double>(
      "noise.gyroscope_variance", config.gyroscope_variance);
    config.mahalanobis_limit = declare_parameter<double>(
      "noise.mahalanobis_limit", config.mahalanobis_limit);
    filter_ = std::make_unique<SelfEkf>(config);
    const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
    state_pub_ = create_publisher<dart_interfaces::msg::SelfState>("~/state", reliable_qos);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10).reliable());
    fin_sub_ = create_subscription<dart_interfaces::msg::FinCommand>(
      fin_input_topic_, reliable_qos,
      [this](const dart_interfaces::msg::FinCommand & message) {
        std::lock_guard lock(mutex_);
        if (message.neutral) {
          fin_angles_.fill(0.0);
        } else if (message.valid && message.armed) {
          std::copy(
            message.deflection_rad.begin(), message.deflection_rad.end(), fin_angles_.begin());
        }
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu & message) {on_imu(message);});
    diagnostics_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {publish_diagnostics();});
  }

private:
  void on_imu(const sensor_msgs::msg::Imu & message)
  {
    const Eigen::Vector4d quaternion{
      message.orientation.w, message.orientation.x, message.orientation.y,
      message.orientation.z};
    const Eigen::Vector3d gyroscope{
      message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z};
    const Eigen::Vector3d accelerometer{
      message.linear_acceleration.x, message.linear_acceleration.y,
      message.linear_acceleration.z};
    const rclcpp::Time stamp(message.header.stamp);
    if (!filter_->initialized()) {
      filter_->reset(quaternion, gyroscope);
      last_stamp_ = stamp;
    } else {
      std::array<double, 4> angles;
      {
        std::lock_guard lock(mutex_);
        angles = fin_angles_;
      }
      last_update_accepted_ = filter_->step(
        (stamp - last_stamp_).seconds(), angles, accelerometer, gyroscope);
      last_stamp_ = stamp;
    }
    dart_interfaces::msg::SelfState output;
    output.header = message.header;
    output.valid = filter_->initialized() && last_update_accepted_;
    output.initialized = filter_->initialized();
    output.dt_s = static_cast<float>((stamp - previous_publish_stamp_).seconds());
    previous_publish_stamp_ = stamp;
    const auto & state = filter_->state();
    output.quaternion_w = static_cast<float>(state[0]);
    output.quaternion_x = static_cast<float>(state[1]);
    output.quaternion_y = static_cast<float>(state[2]);
    output.quaternion_z = static_cast<float>(state[3]);
    output.angular_velocity_x_rad_s = static_cast<float>(state[4]);
    output.angular_velocity_y_rad_s = static_cast<float>(state[5]);
    output.angular_velocity_z_rad_s = static_cast<float>(state[6]);
    for (int index = 0; index < 7; ++index) {
      output.covariance_diagonal[index] = static_cast<float>(filter_->covariance()(index, index));
    }
    output.innovation_mahalanobis = static_cast<float>(filter_->last_mahalanobis());
    state_pub_->publish(output);
    ++samples_;
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "dart_self_estimate_node/status";
    status.hardware_id = "rigid_body_ekf";
    status.level = last_update_accepted_ ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = filter_->initialized() ?
      (last_update_accepted_ ? "self_estimate_valid" : "measurement_rejected") :
      "waiting_for_imu";
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(std::move(array));
  }

  std::string imu_topic_;
  std::string fin_input_topic_;
  std::unique_ptr<SelfEkf> filter_;
  std::mutex mutex_;
  std::array<double, 4> fin_angles_{};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time previous_publish_stamp_{0, 0, RCL_ROS_TIME};
  bool last_update_accepted_{true};
  std::uint64_t samples_{0};
  rclcpp::Subscription<dart_interfaces::msg::FinCommand>::SharedPtr fin_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<dart_interfaces::msg::SelfState>::SharedPtr state_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};
}  // namespace dart_self_estimate_node

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dart_self_estimate_node::DartSelfEstimateNode>());
  rclcpp::shutdown();
  return 0;
}
