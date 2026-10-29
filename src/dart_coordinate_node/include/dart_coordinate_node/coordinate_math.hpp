#pragma once

#include <cmath>
#include <optional>

namespace dart_coordinate_node
{
struct Vector3
{
  double x{};
  double y{};
  double z{};
};

struct Quaternion
{
  double w{1.0};
  double x{};
  double y{};
  double z{};
};

inline std::optional<Quaternion> normalized(Quaternion value)
{
  const auto norm = std::sqrt(
    value.w * value.w + value.x * value.x + value.y * value.y + value.z * value.z);
  if (!std::isfinite(norm) || norm <= 1.0e-8) {return std::nullopt;}
  value.w /= norm;
  value.x /= norm;
  value.y /= norm;
  value.z /= norm;
  return value;
}

inline Vector3 normalized(Vector3 value)
{
  const auto norm = std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
  if (!std::isfinite(norm) || norm <= 1.0e-8) {return {};}
  value.x /= norm;
  value.y /= norm;
  value.z /= norm;
  return value;
}

inline Vector3 bearing_to_body(const double bearing_x_rad, const double bearing_y_rad)
{
  // ROS optical: +x right, +y down, +z forward. DART body: +x nose, +y up,
  // +z completes the right-handed frame. Aligned optical axes therefore map
  // camera +z -> body +x, camera -y -> body +y, camera -x -> body +z.
  return normalized(Vector3{1.0, std::tan(bearing_y_rad), -std::tan(bearing_x_rad)});
}

inline Vector3 rotate_body_to_world(const Quaternion & q, const Vector3 & vector)
{
  const Vector3 u{q.x, q.y, q.z};
  const auto dot = u.x * vector.x + u.y * vector.y + u.z * vector.z;
  const Vector3 cross{
    u.y * vector.z - u.z * vector.y,
    u.z * vector.x - u.x * vector.z,
    u.x * vector.y - u.y * vector.x};
  const auto uu = u.x * u.x + u.y * u.y + u.z * u.z;
  return Vector3{
    2.0 * dot * u.x + (q.w * q.w - uu) * vector.x + 2.0 * q.w * cross.x,
    2.0 * dot * u.y + (q.w * q.w - uu) * vector.y + 2.0 * q.w * cross.y,
    2.0 * dot * u.z + (q.w * q.w - uu) * vector.z + 2.0 * q.w * cross.z};
}
}  // namespace dart_coordinate_node
