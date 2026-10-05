#include <gtest/gtest.h>

#include "aimer_node/fin_allocator.hpp"

TEST(FinAllocator, HoldsFixedFinsAtNeutral)
{
  aimer_node::FinAllocatorConfig config;
  config.movable = {true, false, true, false};
  config.neutral_rad = {0.0, 0.1, 0.0, -0.1};
  aimer_node::FinAllocator allocator(config);
  const auto output = allocator.allocate(0.2, -0.1);
  EXPECT_DOUBLE_EQ(output[1], 0.1);
  EXPECT_DOUBLE_EQ(output[3], -0.1);
}

TEST(FinAllocator, LimitsDeflection)
{
  aimer_node::FinAllocatorConfig config;
  config.min_rad = {-0.05, -0.05, -0.05, -0.05};
  config.max_rad = {0.05, 0.05, 0.05, 0.05};
  aimer_node::FinAllocator allocator(config);
  for (const auto value : allocator.allocate(10.0, 10.0)) {
    EXPECT_LE(value, 0.05);
    EXPECT_GE(value, -0.05);
  }
}
