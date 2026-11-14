#pragma once

#include <Eigen/Dense>
#include <ceres/jet.h>

#include <array>
#include <cmath>
#include <stdexcept>

namespace dart_self_estimate_node
{
template<int DimX, int DimY>
class Ekf
{
public:
  using MatXX = Eigen::Matrix<double, DimX, DimX>;
  using MatYY = Eigen::Matrix<double, DimY, DimY>;
  using VecX = Eigen::Matrix<double, DimX, 1>;
  using VecY = Eigen::Matrix<double, DimY, 1>;

  VecX x{VecX::Zero()};
  VecX x_prior{VecX::Zero()};
  MatXX p{MatXX::Identity()};
  double last_mahalanobis{0.0};

  template<class Transition>
  void predict(Transition && transition, const MatXX & process_noise)
  {
    using Jet = ceres::Jet<double, DimX>;
    std::array<Jet, DimX> input;
    std::array<Jet, DimX> output;
    for (int index = 0; index < DimX; ++index) {
      input[index].a = x[index];
      input[index].v.setZero();
      input[index].v[index] = 1.0;
    }
    transition(input.data(), output.data());
    MatXX jacobian;
    for (int row = 0; row < DimX; ++row) {
      x_prior[row] = output[row].a;
      jacobian.row(row) = output[row].v.transpose();
    }
    p = jacobian * p * jacobian.transpose() + process_noise;
    p = 0.5 * (p + p.transpose());
    x = x_prior;
    require_finite();
  }

  template<class Observation, class Add>
  bool update(
    Observation && observation, const VecY & measurement, const MatYY & measurement_noise,
    Add && add, double mahalanobis_limit)
  {
    using Jet = ceres::Jet<double, DimX>;
    std::array<Jet, DimX> input;
    std::array<Jet, DimY> output;
    for (int index = 0; index < DimX; ++index) {
      input[index].a = x_prior[index];
      input[index].v.setZero();
      input[index].v[index] = 1.0;
    }
    observation(input.data(), output.data());
    VecY predicted;
    Eigen::Matrix<double, DimY, DimX> h;
    for (int row = 0; row < DimY; ++row) {
      predicted[row] = output[row].a;
      h.row(row) = output[row].v.transpose();
    }
    const VecY innovation = measurement - predicted;
    const MatYY innovation_covariance = h * p * h.transpose() + measurement_noise;
    const Eigen::LDLT<MatYY> solver(innovation_covariance);
    if (solver.info() != Eigen::Success) return false;
    last_mahalanobis = innovation.dot(solver.solve(innovation));
    if (!std::isfinite(last_mahalanobis) || last_mahalanobis > mahalanobis_limit) return false;
    const Eigen::Matrix<double, DimX, DimY> gain = solver.solve(h * p).transpose();
    x = add(x_prior, gain * innovation);
    const MatXX identity = MatXX::Identity();
    const MatXX ikh = identity - gain * h;
    p = ikh * p * ikh.transpose() + gain * measurement_noise * gain.transpose();
    p = 0.5 * (p + p.transpose());
    require_finite();
    return true;
  }

private:
  void require_finite() const
  {
    if (!x.allFinite() || !p.allFinite()) throw std::runtime_error("self EKF is not finite");
  }
};
}  // namespace dart_self_estimate_node
