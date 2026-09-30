#pragma once

#include <cmath>
#include <cstdint>

#include <Eigen/Core>

#include <navigation/terrain_core/common/environment/nav_map.hpp>

namespace navigation
{

inline bool footprintBlocksObstacle(
  const Eigen::Vector2d & center_to_obstacle, const uint8_t terrain_label,
  const Eigen::Vector2d & terrain_direction, const double flat_radius,
  const double terrain_half_length, const double terrain_half_width)
{
  if (terrain_label >= static_cast<uint8_t>(terrain_core::TerrainType::SLOPE) &&
    terrain_direction.norm() > 0.1)
  {
    const Eigen::Vector2d along = terrain_direction.normalized();
    const Eigen::Vector2d across(-along.y(), along.x());
    return std::abs(center_to_obstacle.dot(along)) <= terrain_half_length &&
           std::abs(center_to_obstacle.dot(across)) <= terrain_half_width;
  }
  return center_to_obstacle.squaredNorm() <= flat_radius * flat_radius;
}

}  // namespace navigation
