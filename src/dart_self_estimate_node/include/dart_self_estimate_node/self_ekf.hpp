#pragma once

#include <Eigen/Dense>

#include <array>

#include "dart_self_estimate_node/ekf.hpp"

namespace dart_self_estimate_node
{
struct SelfEkfConfig
{
  double mass_kg{1.0};
  Eigen::Vector3d inertia_diagonal{Eigen::Vector3d::Ones()};
  Eigen::Matrix<double, 3, 4> moment_allocation{Eigen::Matrix<double, 3, 4>::Zero()};
  Eigen::Matrix<double, 3, 4> force_allocation{Eigen::Matrix<double, 3, 4>::Zero()};
  Eigen::Vector3d moment_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d force_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gravity_world{0.0, 0.0, -9.80665};
  bool include_gravity_projection{true};
  double quaternion_process_variance{1e-6};
  double angular_velocity_process_variance{1e-3};
  double accelerometer_variance{0.25};
  double gyroscope_variance{1e-3};
  double mahalanobis_limit{30.0};
};

class SelfEkf
{
public:
  explicit SelfEkf(SelfEkfConfig config);
  void reset(const Eigen::Vector4d & quaternion_wxyz, const Eigen::Vector3d & angular_velocity);
  bool step(
    double dt_s, const std::array<double, 4> & fin_angles,
    const Eigen::Vector3d & accelerometer, const Eigen::Vector3d & gyroscope);
  [[nodiscard]] const Eigen::Matrix<double, 7, 1> & state() const noexcept {return ekf_.x;}
  [[nodiscard]] const Eigen::Matrix<double, 7, 7> & covariance() const noexcept {return ekf_.p;}
  [[nodiscard]] double last_mahalanobis() const noexcept {return ekf_.last_mahalanobis;}
  [[nodiscard]] bool initialized() const noexcept {return initialized_;}

private:
  SelfEkfConfig config_;
  Ekf<7, 6> ekf_;
  bool initialized_{false};
};
}  // namespace dart_self_estimate_node
