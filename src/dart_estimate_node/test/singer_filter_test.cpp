#include "dart_estimate_node/singer_filter.hpp"

#include <gtest/gtest.h>

TEST(SingerFilter, TracksStationaryPosition)
{
  dart_estimate_node::SingerFilter filter({});
  filter.reset(Eigen::Vector3d{1.0, 2.0, 3.0});
  for (int index = 0; index < 100; ++index) {
    filter.predict(0.01);
    ASSERT_TRUE(filter.update_position(Eigen::Vector3d{1.0, 2.0, 3.0}, 100.0));
  }
  EXPECT_NEAR(filter.state()[0], 1.0, 1e-3);
  EXPECT_NEAR(filter.state()[1], 2.0, 1e-3);
  EXPECT_NEAR(filter.state()[2], 3.0, 1e-3);
  EXPECT_LT(filter.state().segment<3>(3).norm(), 1e-2);
}

TEST(SingerFilter, ProcessNoiseIsSymmetricPositiveSemidefinite)
{
  const auto noise = dart_estimate_node::SingerFilter::process_noise_axis(0.01, 0.5, 2.0);
  EXPECT_TRUE(noise.isApprox(noise.transpose(), 1e-12));
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(noise);
  ASSERT_EQ(solver.info(), Eigen::Success);
  EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-10);
}
