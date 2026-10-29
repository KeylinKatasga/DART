#include <gtest/gtest.h>

#include "dart_calculate_node/dynamics_model.hpp"

TEST(DynamicsModelFramework, UsesClockwiseXFinNumbering)
{
  using dart_calculate_node::FinIndex;
  EXPECT_EQ(static_cast<std::size_t>(FinIndex::upper_left), 0U);
  EXPECT_EQ(static_cast<std::size_t>(FinIndex::upper_right), 1U);
  EXPECT_EQ(static_cast<std::size_t>(FinIndex::lower_right), 2U);
  EXPECT_EQ(static_cast<std::size_t>(FinIndex::lower_left), 3U);
  EXPECT_EQ(dart_calculate_node::kFinCount, 4U);
}
