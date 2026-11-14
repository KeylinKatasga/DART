#include "dart_self_estimate_node/self_ekf.hpp"

#include <ceres/jet.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dart_self_estimate_node
{
namespace
{
template<class T>
void normalize_quaternion(T * state)
{
  const T norm = ceres::sqrt(
    state[0] * state[0] + state[1] * state[1] + state[2] * state[2] +
    state[3] * state[3]);
  for (int index = 0; index < 4; ++index) state[index] /= norm;
}

template<class T>
Eigen::Matrix<T, 3, 1> gravity_in_body(
  const T & w, const T & x, const T & y, const T & z, const Eigen::Vector3d & gravity_world)
{
  Eigen::Matrix<T, 3, 3> rotation;
  rotation <<
    T(1.0) - T(2.0) * (y * y + z * z), T(2.0) * (x * y - z * w),
    T(2.0) * (x * z + y * w),
    T(2.0) * (x * y + z * w), T(1.0) - T(2.0) * (x * x + z * z),
    T(2.0) * (y * z - x * w),
    T(2.0) * (x * z - y * w), T(2.0) * (y * z + x * w),
    T(1.0) - T(2.0) * (x * x + y * y);
  return rotation.transpose() * gravity_world.cast<T>();
}
}  // namespace

SelfEkf::SelfEkf(SelfEkfConfig config) : config_(std::move(config))
{
  if (config_.mass_kg <= 0.0 || (config_.inertia_diagonal.array() <= 0.0).any() ||
    config_.quaternion_process_variance < 0.0 ||
    config_.angular_velocity_process_variance < 0.0 ||
    config_.accelerometer_variance <= 0.0 || config_.gyroscope_variance <= 0.0 ||
    config_.mahalanobis_limit <= 0.0)
  {
    throw std::invalid_argument("invalid self EKF configuration");
  }
}

void SelfEkf::reset(
  const Eigen::Vector4d & quaternion_wxyz, const Eigen::Vector3d & angular_velocity)
{
  const double norm = quaternion_wxyz.norm();
  if (!std::isfinite(norm) || norm < 1e-9 || !angular_velocity.allFinite()) {
    throw std::invalid_argument("invalid self EKF initial state");
  }
  ekf_.x.head<4>() = quaternion_wxyz / norm;
  ekf_.x.tail<3>() = angular_velocity;
  ekf_.x_prior = ekf_.x;
  ekf_.p.setIdentity();
  ekf_.p.topLeftCorner<4, 4>() *= 0.01;
  ekf_.p.bottomRightCorner<3, 3>() *= 0.1;
  initialized_ = true;
}

bool SelfEkf::step(
  double dt_s, const std::array<double, 4> & fin_angles,
  const Eigen::Vector3d & accelerometer, const Eigen::Vector3d & gyroscope)
{
  if (!initialized_ || !accelerometer.allFinite() || !gyroscope.allFinite() ||
    !std::isfinite(dt_s) || dt_s <= 0.0)
  {
    return false;
  }
  dt_s = std::clamp(dt_s, 1e-4, 0.05);
  const Eigen::Vector4d control{fin_angles[0], fin_angles[1], fin_angles[2], fin_angles[3]};
  const Eigen::Vector3d moment = config_.moment_bias + config_.moment_allocation * control;
  const Eigen::Vector3d force = config_.force_bias + config_.force_allocation * control;
  const auto inertia = config_.inertia_diagonal;
  ekf_.predict(
    [dt_s, moment, inertia](const auto * input, auto * output) {
      const auto & qw = input[0];
      const auto & qx = input[1];
      const auto & qy = input[2];
      const auto & qz = input[3];
      const auto & wx = input[4];
      const auto & wy = input[5];
      const auto & wz = input[6];
      output[0] = qw + 0.5 * dt_s * (-qx * wx - qy * wy - qz * wz);
      output[1] = qx + 0.5 * dt_s * (qw * wx + qy * wz - qz * wy);
      output[2] = qy + 0.5 * dt_s * (qw * wy + qz * wx - qx * wz);
      output[3] = qz + 0.5 * dt_s * (qw * wz + qx * wy - qy * wx);
      normalize_quaternion(output);
      const auto cross_x = wy * (inertia.z() * wz) - wz * (inertia.y() * wy);
      const auto cross_y = wz * (inertia.x() * wx) - wx * (inertia.z() * wz);
      const auto cross_z = wx * (inertia.y() * wy) - wy * (inertia.x() * wx);
      output[4] = wx + dt_s * (moment.x() - cross_x) / inertia.x();
      output[5] = wy + dt_s * (moment.y() - cross_y) / inertia.y();
      output[6] = wz + dt_s * (moment.z() - cross_z) / inertia.z();
    },
    (Eigen::Matrix<double, 7, 1>() <<
      config_.quaternion_process_variance, config_.quaternion_process_variance,
      config_.quaternion_process_variance, config_.quaternion_process_variance,
      config_.angular_velocity_process_variance, config_.angular_velocity_process_variance,
      config_.angular_velocity_process_variance).finished().asDiagonal() * dt_s);
  Eigen::Matrix<double, 6, 1> measurement;
  measurement << accelerometer, gyroscope;
  Eigen::Matrix<double, 6, 6> measurement_noise = Eigen::Matrix<double, 6, 6>::Zero();
  measurement_noise.diagonal() <<
    config_.accelerometer_variance, config_.accelerometer_variance,
    config_.accelerometer_variance, config_.gyroscope_variance,
    config_.gyroscope_variance, config_.gyroscope_variance;
  const bool accepted = ekf_.update(
    [this, force](const auto * state, auto * output) {
      using T = std::remove_cvref_t<decltype(state[0])>;
      Eigen::Matrix<T, 3, 1> predicted = (force / config_.mass_kg).cast<T>();
      if (config_.include_gravity_projection) {
        predicted += gravity_in_body(
          state[0], state[1], state[2], state[3], config_.gravity_world);
      }
      output[0] = predicted[0];
      output[1] = predicted[1];
      output[2] = predicted[2];
      output[3] = state[4];
      output[4] = state[5];
      output[5] = state[6];
    }, measurement, measurement_noise,
    [](const auto & prior, const auto & increment) {
      Eigen::Matrix<double, 7, 1> result = prior + increment;
      result.head(4).normalize();
      return result;
    }, config_.mahalanobis_limit);
  return accepted;
}
}  // namespace dart_self_estimate_node
