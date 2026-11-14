#pragma once

#include <Eigen/Dense>

#include <array>

namespace dart_estimate_node
{
struct SingerConfig
{
  double tau_s{0.5};
  double acceleration_spectral_density{1.0};
  double position_variance{0.01};
  double initial_position_variance{0.25};
  double initial_velocity_variance{4.0};
  double initial_acceleration_variance{16.0};
};

class SingerFilter
{
public:
  explicit SingerFilter(SingerConfig config);

  void reset(const Eigen::Vector3d & position);
  void predict(double dt_s);
  bool update_position(const Eigen::Vector3d & position, double mahalanobis_limit);
  [[nodiscard]] Eigen::Vector3d predicted_position(double horizon_s) const;
  [[nodiscard]] const Eigen::Matrix<double, 9, 1> & state() const noexcept {return x_;}
  [[nodiscard]] const Eigen::Matrix<double, 9, 9> & covariance() const noexcept {return p_;}
  [[nodiscard]] bool initialized() const noexcept {return initialized_;}
  [[nodiscard]] double last_mahalanobis() const noexcept {return last_mahalanobis_;}

  static Eigen::Matrix3d transition_axis(double dt_s, double tau_s);
  static Eigen::Matrix3d process_noise_axis(
    double dt_s, double tau_s, double spectral_density);

private:
  SingerConfig config_;
  Eigen::Matrix<double, 9, 1> x_{Eigen::Matrix<double, 9, 1>::Zero()};
  Eigen::Matrix<double, 9, 9> p_{Eigen::Matrix<double, 9, 9>::Identity()};
  bool initialized_{false};
  double last_mahalanobis_{0.0};
};
}  // namespace dart_estimate_node
