#include <gtest/gtest.h>

#include <navigation/clearance_oracle.hpp>

namespace
{
constexpr double kRadius = 0.5656854249;

TEST(ClearanceOracle, FlatCorridorRejectsBelowCircumscribedDiameter)
{
  // Half of a 1.0 m corridor is inside R=0.566: either wall blocks its centreline.
  EXPECT_TRUE(navigation::footprintBlocksObstacle(
    Eigen::Vector2d(0.0, 0.50), 0, Eigen::Vector2d::Zero(), kRadius, 0.4, 0.4));
}

TEST(ClearanceOracle, FlatCorridorAcceptsAboveCircumscribedDiameter)
{
  // A 1.20 m corridor leaves 0.60 m to either wall.
  EXPECT_FALSE(navigation::footprintBlocksObstacle(
    Eigen::Vector2d(0.0, 0.60), 0, Eigen::Vector2d::Zero(), kRadius, 0.4, 0.4));
}

TEST(ClearanceOracle, DirectionalTerrainUsesAlignedSquare)
{
  const uint8_t slope = static_cast<uint8_t>(
    navigation::terrain_core::TerrainType::SLOPE);
  EXPECT_FALSE(navigation::footprintBlocksObstacle(
    Eigen::Vector2d(0.0, 0.50), slope, Eigen::Vector2d::UnitX(), kRadius, 0.4, 0.4));
  EXPECT_TRUE(navigation::footprintBlocksObstacle(
    Eigen::Vector2d(0.0, 0.39), slope, Eigen::Vector2d::UnitX(), kRadius, 0.4, 0.4));
}
}  // namespace
