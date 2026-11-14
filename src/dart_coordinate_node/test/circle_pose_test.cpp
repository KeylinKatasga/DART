#include "dart_coordinate_node/circle_pose.hpp"

#include <array>
#include <vector>

#include <gtest/gtest.h>

TEST(CirclePose, RecoversFrontalKnownCircle)
{
  dart_coordinate_node::CirclePoseSolver solver(0.1, 1.0);
  solver.set_calibration(
    std::array<double, 9>{1000.0, 0.0, 640.0, 0.0, 1000.0, 512.0, 0.0, 0.0, 1.0},
    std::vector<double>(5, 0.0));
  const auto pose = solver.solve(640.0, 512.0, 50.0);
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->position_camera_m[0], 0.0, 1e-5);
  EXPECT_NEAR(pose->position_camera_m[1], 0.0, 1e-5);
  EXPECT_NEAR(pose->position_camera_m[2], 1.0, 1e-5);
  EXPECT_LT(pose->reprojection_error_px, 1e-5);
}
