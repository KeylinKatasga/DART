#include "dart_self_estimate_node/self_ekf.hpp"

#include <gtest/gtest.h>

TEST(SelfEkf, MaintainsNormalizedStationaryAttitude)
{
  dart_self_estimate_node::SelfEkfConfig config;
  config.include_gravity_projection = true;
  config.mahalanobis_limit = 1000.0;
  dart_self_estimate_node::SelfEkf filter(config);
  filter.reset(Eigen::Vector4d{1.0, 0.0, 0.0, 0.0}, Eigen::Vector3d::Zero());
  for (int index = 0; index < 50; ++index) {
    ASSERT_TRUE(filter.step(
      0.01, std::array<double, 4>{}, Eigen::Vector3d{0.0, 0.0, -9.80665},
      Eigen::Vector3d::Zero()));
  }
  EXPECT_NEAR(filter.state().head<4>().norm(), 1.0, 1e-9);
  EXPECT_LT(filter.state().tail<3>().norm(), 1e-6);
}

TEST(SelfEkf, FinMomentChangesAngularVelocity)
{
  dart_self_estimate_node::SelfEkfConfig config;
  config.include_gravity_projection = false;
  config.moment_allocation(0, 0) = 1.0;
  config.mahalanobis_limit = 1e9;
  dart_self_estimate_node::SelfEkf filter(config);
  filter.reset(Eigen::Vector4d{1.0, 0.0, 0.0, 0.0}, Eigen::Vector3d::Zero());
  filter.step(
    0.01, std::array<double, 4>{0.1, 0.0, 0.0, 0.0}, Eigen::Vector3d::Zero(),
    Eigen::Vector3d{0.001, 0.0, 0.0});
  EXPECT_GT(filter.state()[4], 0.0);
}
