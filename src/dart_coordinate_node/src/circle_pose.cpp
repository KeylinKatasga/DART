#include "dart_coordinate_node/circle_pose.hpp"

#include <cmath>
#include <stdexcept>

#include <opencv2/calib3d.hpp>

namespace dart_coordinate_node
{
CirclePoseSolver::CirclePoseSolver(double diameter_m, double max_reprojection_error_px)
: diameter_m_(diameter_m), max_reprojection_error_px_(max_reprojection_error_px)
{
  if (diameter_m_ <= 0.0 || max_reprojection_error_px_ <= 0.0) {
    throw std::invalid_argument("invalid known-circle pose configuration");
  }
}

void CirclePoseSolver::set_calibration(
  const std::array<double, 9> & camera_matrix, const std::vector<double> & distortion)
{
  if (camera_matrix[0] <= 0.0 || camera_matrix[4] <= 0.0) {
    throw std::invalid_argument("camera focal lengths must be positive");
  }
  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      camera_matrix_.at<double>(row, column) = camera_matrix[row * 3 + column];
    }
  }
  distortion_ = cv::Mat::zeros(1, static_cast<int>(distortion.size()), CV_64F);
  for (std::size_t index = 0; index < distortion.size(); ++index) {
    distortion_.at<double>(0, static_cast<int>(index)) = distortion[index];
  }
  calibrated_ = true;
}

std::optional<CirclePose> CirclePoseSolver::solve(
  double center_x_px, double center_y_px, double radius_px) const
{
  if (!calibrated_ || !std::isfinite(center_x_px) || !std::isfinite(center_y_px) ||
    !std::isfinite(radius_px) || radius_px <= 0.0)
  {
    return std::nullopt;
  }
  const float radius_m = static_cast<float>(0.5 * diameter_m_);
  const std::vector<cv::Point3f> object_points{
    {-radius_m, 0.0F, 0.0F}, {0.0F, -radius_m, 0.0F},
    {radius_m, 0.0F, 0.0F}, {0.0F, radius_m, 0.0F}};
  const float cx = static_cast<float>(center_x_px);
  const float cy = static_cast<float>(center_y_px);
  const float r = static_cast<float>(radius_px);
  const std::vector<cv::Point2f> image_points{
    {cx - r, cy}, {cx, cy - r}, {cx + r, cy}, {cx, cy + r}};
  cv::Mat rotation_vector;
  cv::Mat translation_vector;
  const bool solved = cv::solvePnP(
    object_points, image_points, camera_matrix_, distortion_, rotation_vector,
    translation_vector, false, cv::SOLVEPNP_ITERATIVE);
  if (!solved || translation_vector.rows != 3) return std::nullopt;
  const cv::Vec3d translation{
    translation_vector.at<double>(0), translation_vector.at<double>(1),
    translation_vector.at<double>(2)};
  if (!std::isfinite(translation[0]) || !std::isfinite(translation[1]) ||
    !std::isfinite(translation[2]) || translation[2] <= 0.0)
  {
    return std::nullopt;
  }
  std::vector<cv::Point2f> reprojected;
  cv::projectPoints(
    object_points, rotation_vector, translation_vector, camera_matrix_, distortion_, reprojected);
  double squared_error = 0.0;
  for (std::size_t index = 0; index < image_points.size(); ++index) {
    const auto difference = reprojected[index] - image_points[index];
    squared_error += difference.dot(difference);
  }
  const double rms = std::sqrt(squared_error / static_cast<double>(image_points.size()));
  if (!std::isfinite(rms) || rms > max_reprojection_error_px_) return std::nullopt;
  return CirclePose{translation, rms};
}
}  // namespace dart_coordinate_node
