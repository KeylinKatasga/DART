#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

#include "dart_coordinate_node/coordinate_math.hpp"

TEST(CoordinateMath, OpticalAxisMapsToBodyPositiveX)
{
  const auto value = dart_coordinate_node::bearing_to_body(0.0, 0.0);
  EXPECT_DOUBLE_EQ(value.x, 1.0);
  EXPECT_DOUBLE_EQ(value.y, 0.0);
  EXPECT_DOUBLE_EQ(value.z, 0.0);
}

TEST(CoordinateMath, ImageRightMapsToBodyNegativeZ)
{
  const auto value = dart_coordinate_node::bearing_to_body(0.1, 0.0);
  EXPECT_GT(value.x, 0.0);
  EXPECT_DOUBLE_EQ(value.y, 0.0);
  EXPECT_LT(value.z, 0.0);
}

TEST(CoordinateMath, QuaternionRotatesBodyLineOfSightToWorld)
{
  const auto half = std::numbers::pi / 4.0;
  const auto quaternion = dart_coordinate_node::normalized(
    dart_coordinate_node::Quaternion{std::cos(half), 0.0, 0.0, std::sin(half)});
  ASSERT_TRUE(quaternion.has_value());
  const auto value = dart_coordinate_node::rotate_body_to_world(
    *quaternion, dart_coordinate_node::Vector3{1.0, 0.0, 0.0});
  EXPECT_NEAR(value.x, 0.0, 1.0e-9);
  EXPECT_NEAR(value.y, 1.0, 1.0e-9);
  EXPECT_NEAR(value.z, 0.0, 1.0e-9);
}
