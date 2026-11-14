#include "dart_estimate_node/singer_filter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dart_estimate_node
{
SingerFilter::SingerFilter(SingerConfig config) : config_(config)
{
  if (config_.tau_s <= 0.0 || config_.acceleration_spectral_density < 0.0 ||
    config_.position_variance <= 0.0 || config_.initial_position_variance <= 0.0 ||
    config_.initial_velocity_variance <= 0.0 || config_.initial_acceleration_variance <= 0.0)
  {
    throw std::invalid_argument("invalid Singer filter configuration");
  }
}

Eigen::Matrix3d SingerFilter::transition_axis(double dt_s, double tau_s)
{
  const double alpha = 1.0 / tau_s;
  const double decay = std::exp(-alpha * dt_s);
  Eigen::Matrix3d transition = Eigen::Matrix3d::Identity();
  transition(0, 1) = dt_s;
  transition(0, 2) = (decay + alpha * dt_s - 1.0) / (alpha * alpha);
  transition(1, 2) = (1.0 - decay) / alpha;
  transition(2, 2) = decay;
  return transition;
}

Eigen::Matrix3d SingerFilter::process_noise_axis(
  double dt_s, double tau_s, double spectral_density)
{
  const double a = 1.0 / tau_s;
  const double t = dt_s;
  const double e = std::exp(-a * t);
  const double e2 = e * e;
  const double a2 = a * a;
  const double a3 = a2 * a;
  const double a4 = a3 * a;
  const double a5 = a4 * a;
  Eigen::Matrix3d q;
  q(0, 0) = t * t * t / (3.0 * a2) - t * t / a3 + t / a4 +
    (0.5 - 2.0 * a * t * e - 0.5 * e2) / a5;
  q(0, 1) = t * t / (2.0 * a2) - t / a3 +
    (0.5 - e + a * t * e + 0.5 * e2) / a4;
  q(0, 2) = (0.5 - a * t * e - 0.5 * e2) / a3;
  q(1, 1) = (t - 2.0 * (1.0 - e) / a + (1.0 - e2) / (2.0 * a)) / a2;
  q(1, 2) = (1.0 - 2.0 * e + e2) / (2.0 * a2);
  q(2, 2) = (1.0 - e2) / (2.0 * a);
  q(1, 0) = q(0, 1);
  q(2, 0) = q(0, 2);
  q(2, 1) = q(1, 2);
  return spectral_density * 0.5 * (q + q.transpose());
}

void SingerFilter::reset(const Eigen::Vector3d & position)
{
  x_.setZero();
  x_.segment<3>(0) = position;
  p_.setZero();
  p_.diagonal() <<
    config_.initial_position_variance, config_.initial_position_variance,
    config_.initial_position_variance, config_.initial_velocity_variance,
    config_.initial_velocity_variance, config_.initial_velocity_variance,
    config_.initial_acceleration_variance, config_.initial_acceleration_variance,
    config_.initial_acceleration_variance;
  initialized_ = true;
  last_mahalanobis_ = 0.0;
}

void SingerFilter::predict(double dt_s)
{
  if (!initialized_ || !std::isfinite(dt_s) || dt_s <= 0.0) return;
  dt_s = std::clamp(dt_s, 1e-4, 0.2);
  const auto axis_transition = transition_axis(dt_s, config_.tau_s);
  const auto axis_noise = process_noise_axis(
    dt_s, config_.tau_s, config_.acceleration_spectral_density);
  Eigen::Matrix<double, 9, 9> transition = Eigen::Matrix<double, 9, 9>::Zero();
  Eigen::Matrix<double, 9, 9> noise = Eigen::Matrix<double, 9, 9>::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    const std::array<int, 3> indices{axis, axis + 3, axis + 6};
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        transition(indices[row], indices[column]) = axis_transition(row, column);
        noise(indices[row], indices[column]) = axis_noise(row, column);
      }
    }
  }
  x_ = transition * x_;
  p_ = transition * p_ * transition.transpose() + noise;
  p_ = 0.5 * (p_ + p_.transpose());
}

bool SingerFilter::update_position(
  const Eigen::Vector3d & position, double mahalanobis_limit)
{
  if (!initialized_ || !position.allFinite()) return false;
  Eigen::Matrix<double, 3, 9> h = Eigen::Matrix<double, 3, 9>::Zero();
  h.block<3, 3>(0, 0).setIdentity();
  const Eigen::Matrix3d r = config_.position_variance * Eigen::Matrix3d::Identity();
  const Eigen::Vector3d innovation = position - h * x_;
  const Eigen::Matrix3d s = h * p_ * h.transpose() + r;
  const Eigen::LDLT<Eigen::Matrix3d> solver(s);
  if (solver.info() != Eigen::Success) return false;
  last_mahalanobis_ = innovation.dot(solver.solve(innovation));
  if (!std::isfinite(last_mahalanobis_) || last_mahalanobis_ > mahalanobis_limit) return false;
  const Eigen::Matrix<double, 9, 3> gain = solver.solve(h * p_).transpose();
  x_ += gain * innovation;
  const auto identity = Eigen::Matrix<double, 9, 9>::Identity();
  const auto residual = identity - gain * h;
  p_ = residual * p_ * residual.transpose() + gain * r * gain.transpose();
  p_ = 0.5 * (p_ + p_.transpose());
  return x_.allFinite() && p_.allFinite();
}

Eigen::Vector3d SingerFilter::predicted_position(double horizon_s) const
{
  if (!initialized_ || horizon_s <= 0.0) return x_.segment<3>(0);
  Eigen::Vector3d result;
  const auto transition = transition_axis(horizon_s, config_.tau_s);
  for (int axis = 0; axis < 3; ++axis) {
    const Eigen::Vector3d state{x_[axis], x_[axis + 3], x_[axis + 6]};
    result[axis] = (transition * state)[0];
  }
  return result;
}
}  // namespace dart_estimate_node
