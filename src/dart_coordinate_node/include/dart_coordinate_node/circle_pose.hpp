#pragma once

#include <array>
#include <optional>
#include <vector>

#include <opencv2/core.hpp>

namespace dart_coordinate_node
{
struct CirclePose
{
  cv::Vec3d position_camera_m{};
  double reprojection_error_px{0.0};
};

class CirclePoseSolver
{
public:
  CirclePoseSolver(double diameter_m, double max_reprojection_error_px);

  void set_calibration(const std::array<double, 9> & camera_matrix, const std::vector<double> & distortion);
  [[nodiscard]] bool calibrated() const noexcept {return calibrated_;}
  [[nodiscard]] std::optional<CirclePose> solve(
    double center_x_px, double center_y_px, double radius_px) const;

private:
  double diameter_m_{};
  double max_reprojection_error_px_{};
  cv::Mat camera_matrix_;
  cv::Mat distortion_;
  bool calibrated_{false};
};
}  // namespace dart_coordinate_node
