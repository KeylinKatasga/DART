#include <gtest/gtest.h>

#include "dart_calculate_node/fin_allocator.hpp"

TEST(FinAllocator, HoldsFixedFinsAtNeutral)
{
  dart_calculate_node::FinAllocatorConfig config;
  config.movable = {true, false, true, false};
  config.neutral_rad = {0.0, 0.1, 0.0, -0.1};
  dart_calculate_node::FinAllocator allocator(config);
  const auto output = allocator.allocate(0.2, -0.1);
  EXPECT_DOUBLE_EQ(output[1], 0.1);
  EXPECT_DOUBLE_EQ(output[3], -0.1);
}

TEST(FinAllocator, LimitsDeflection)
{
  dart_calculate_node::FinAllocatorConfig config;
  config.min_rad = {-0.05, -0.05, -0.05, -0.05};
  config.max_rad = {0.05, 0.05, 0.05, 0.05};
  dart_calculate_node::FinAllocator allocator(config);
  for (const auto value : allocator.allocate(10.0, 10.0)) {
    EXPECT_LE(value, 0.05);
    EXPECT_GE(value, -0.05);
  }
}
