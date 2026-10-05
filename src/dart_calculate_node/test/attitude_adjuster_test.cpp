#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

#include "dart_calculate_node/attitude_adjuster.hpp"

using dart_calculate_node::Quaternion;
using dart_calculate_node::calculate_attitude_adjustment;

TEST(AttitudeAdjuster, IdentityQuaternionPreservesTargetWinding)
{
  const auto adjustment = calculate_attitude_adjustment(0.1, 0.0, {}, 0.0, 0.001);
  ASSERT_TRUE(adjustment.valid);
  EXPECT_NEAR(adjustment.body_roll_rad, 0.0, 1.0e-9);
  EXPECT_NEAR(adjustment.attack_error_rad, 0.1, 1.0e-9);
  EXPECT_NEAR(adjustment.winding_error_rad, 0.0, 1.0e-9);
}

TEST(AttitudeAdjuster, CompensatesBodyRollFromQuaternion)
{
  const auto half = std::numbers::pi / 4.0;
  const Quaternion rolled{std::cos(half), std::sin(half), 0.0, 0.0};
  const auto adjustment = calculate_attitude_adjustment(0.1, 0.0, rolled, 0.0, 0.001);
  ASSERT_TRUE(adjustment.valid);
  EXPECT_NEAR(adjustment.body_roll_rad, std::numbers::pi / 2.0, 1.0e-9);
  EXPECT_NEAR(adjustment.winding_error_rad, -std::numbers::pi / 2.0, 1.0e-9);
}

TEST(AttitudeAdjuster, SuppressesUndefinedWindingNearOpticalAxis)
{
  const auto adjustment = calculate_attitude_adjustment(0.0, 0.0, {}, 0.2, 0.001);
  ASSERT_TRUE(adjustment.valid);
  EXPECT_DOUBLE_EQ(adjustment.attack_error_rad, 0.0);
  EXPECT_DOUBLE_EQ(adjustment.winding_error_rad, 0.0);
}

TEST(AttitudeAdjuster, RejectsInvalidQuaternion)
{
  const auto adjustment = calculate_attitude_adjustment(0.1, 0.1, {0.0, 0.0, 0.0, 0.0}, 0.0, 0.001);
  EXPECT_FALSE(adjustment.valid);
}
