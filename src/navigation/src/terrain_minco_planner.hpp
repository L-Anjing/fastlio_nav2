#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <chrono>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <navigation/msg/plan_meta.hpp>

#include <navigation/terrain_core/common/environment/nav_map.hpp>
#include <navigation/terrain_core/common/trajectory/minco_trajectory.hpp>
#include <navigation/terrain_core/path_planner/search/spatial_grid_astar.hpp>
#include <navigation/terrain_core/path_planner/trajectory/minco_optimizer.hpp>

namespace navigation
{

class TerrainMincoPlanner : public nav2_core::GlobalPlanner
{
public:
  TerrainMincoPlanner() = default;
  ~TerrainMincoPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  struct PlanningSnapshot
  {
    std::shared_ptr<terrain_core::CostMap> cost;
    std::shared_ptr<terrain_core::DirectionMap> direction;
    terrain_core::TerrainTraversalConstraints terrain;
    size_t lethal_source_count{0};
  };

  PlanningSnapshot makeSnapshot() const;
  bool lineOfSight(
    const Eigen::Vector2d & from, const Eigen::Vector2d & to,
    const PlanningSnapshot & snapshot) const;
  bool cachedPathSafe(
    const Eigen::Vector2d & start, const Eigen::Vector2d & goal,
    const PlanningSnapshot & snapshot, size_t & nearest_index,
    std::string & invalid_reason) const;
  std::vector<Eigen::Vector2d> simplifyRoute(
    const terrain_core::SpatialRoute & route,
    const Eigen::Vector2d & start, const Eigen::Vector2d & goal,
    const PlanningSnapshot & snapshot) const;
  std::vector<terrain_core::GeometricBoundary> makeMincoSeed(
    const std::vector<Eigen::Vector2d> & anchors,
    const terrain_core::DirectionMap & direction) const;
  bool validateTrajectory(
    const terrain_core::MincoTrajectory & trajectory,
    const PlanningSnapshot & snapshot, std::string & reason) const;
  nav_msgs::msg::Path trajectoryToPath(
    const terrain_core::MincoTrajectory & trajectory,
    const std_msgs::msg::Header & header,
    const geometry_msgs::msg::Quaternion & goal_orientation) const;
  nav_msgs::msg::Path trimmedCachedPath(
    const geometry_msgs::msg::PoseStamped & start, size_t nearest_index) const;
  void publishPlanMeta(
    const std_msgs::msg::Header & header, bool replanned,
    float path_start_s, const std::string & reason);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr terrain_type_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr terrain_direction_sub_;
  rclcpp::Publisher<navigation::msg::PlanMeta>::SharedPtr plan_meta_pub_;
  mutable std::mutex terrain_mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr terrain_type_;
  nav_msgs::msg::OccupancyGrid::SharedPtr terrain_direction_;

  int occupied_cost_threshold_{253};
  int unknown_traversal_cost_{50};
  int max_astar_expansions_{1000000};
  int minco_max_iterations_{400};
  double flat_clearance_radius_{0.5656854249};
  double terrain_half_length_{0.4};
  double terrain_half_width_{0.4};
  double obstacle_weight_{1.0};
  double min_terrain_alignment_cosine_{0.9659258263};
  double seed_spacing_{0.45};
  double output_spacing_{0.05};
  double validation_spacing_{0.025};
  double max_path_deviation_{0.35};
  double goal_cache_tolerance_{0.05};
  double curvature_max_{4.0};
  double curvature_rate_max_{8.0};
  double minco_obstacle_weight_{1000.0};
  double minco_curvature_weight_{100.0};
  double minco_curvature_rate_weight_{100.0};
  double minco_alignment_weight_{2000.0};
  double optimization_check_frequency_{1.0};
  double costmap_stable_time_{0.8};
  double optimization_cooldown_{1.5};
  double minimum_length_improvement_ratio_{0.08};

  nav_msgs::msg::Path cached_path_;
  std::vector<double> cached_path_s_;
  Eigen::Vector2d cached_goal_{Eigen::Vector2d::Zero()};
  bool have_cached_path_{false};
  uint64_t path_id_{0};
  uint64_t publish_seq_{0};
  size_t last_lethal_source_count_{0};
  bool have_lethal_source_count_{false};
  bool optimization_pending_{false};
  std::chrono::steady_clock::time_point optimization_pending_since_{};
  std::chrono::steady_clock::time_point last_optimization_attempt_{};
  std::chrono::steady_clock::time_point last_accepted_replan_{};
};

}  // namespace navigation
