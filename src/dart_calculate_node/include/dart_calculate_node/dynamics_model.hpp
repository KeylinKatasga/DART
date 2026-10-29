#pragma once

#include <array>
#include <cstddef>

namespace dart_calculate_node
{
inline constexpr std::size_t kFinCount = 4;

enum class FinIndex : std::size_t
{
  upper_left = 0,
  upper_right = 1,
  lower_right = 2,
  lower_left = 3,
};

struct BodyVector
{
  double x{};
  double y{};
  double z{};
};

struct BodyWrench
{
  BodyVector force_newton;
  BodyVector moment_newton_meter;
};

struct FinGeometry
{
  BodyVector arm_from_center_m;
  BodyVector zero_normal_body;
  BodyVector hinge_axis_body;
  double area_m2{};
  double chord_m{};
};

struct DynamicsParameters
{
  double mass_kg{};
  BodyVector inertia_diagonal_kg_m2;
  double air_density_kg_m3{};
  std::array<FinGeometry, kFinCount> fins;
};

struct RigidBodyState
{
  BodyVector velocity_body_mps;
  BodyVector angular_velocity_body_rad_s;
  std::array<double, 4> quaternion_body_to_world_wxyz{1.0, 0.0, 0.0, 0.0};
};

class AerodynamicModel
{
public:
  virtual ~AerodynamicModel() = default;
  virtual BodyWrench evaluate(
    const DynamicsParameters & parameters, const RigidBodyState & state,
    const std::array<double, kFinCount> & fin_angle_rad) const = 0;
};
}  // namespace dart_calculate_node
