#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>

namespace dart_calculate_node
{
struct Quaternion
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct AttitudeAdjustment
{
  bool valid{false};
  double body_roll_rad{0.0};
  double target_winding_rad{0.0};
  double attack_error_rad{0.0};
  double winding_error_rad{0.0};
};

inline double wrap_angle(double value)
{
  return std::remainder(value, 2.0 * std::numbers::pi);
}

inline AttitudeAdjustment calculate_attitude_adjustment(
  const double bearing_x_rad, const double bearing_y_rad, const Quaternion quaternion,
  const double winding_offset_rad, const double minimum_attack_rad)
{
  AttitudeAdjustment result;
  const auto norm = std::sqrt(
    quaternion.w * quaternion.w + quaternion.x * quaternion.x +
    quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  if (!std::isfinite(norm) || norm <= 1.0e-8 || !std::isfinite(bearing_x_rad) ||
    !std::isfinite(bearing_y_rad))
  {
    return result;
  }
  const auto w = quaternion.w / norm;
  const auto x = quaternion.x / norm;
  const auto y = quaternion.y / norm;
  const auto z = quaternion.z / norm;
  const auto sin_roll_cos_pitch = 2.0 * (w * x + y * z);
  const auto cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y);
  result.body_roll_rad = std::atan2(sin_roll_cos_pitch, cos_roll_cos_pitch);

  // Detection bearings form a polar correction around the optical axis. The radial magnitude
  // is the attack correction; its azimuth, compensated by IMU roll, is the winding correction.
  const auto tangent_x = std::tan(bearing_x_rad);
  const auto tangent_y = std::tan(bearing_y_rad);
  result.attack_error_rad = std::atan(std::hypot(tangent_x, tangent_y));
  if (result.attack_error_rad >= minimum_attack_rad) {
    result.target_winding_rad = std::atan2(tangent_y, tangent_x);
    result.winding_error_rad = wrap_angle(
      result.target_winding_rad - result.body_roll_rad + winding_offset_rad);
  }
  result.valid = std::isfinite(result.attack_error_rad) &&
    std::isfinite(result.winding_error_rad);
  return result;
}
}  // namespace dart_calculate_node
