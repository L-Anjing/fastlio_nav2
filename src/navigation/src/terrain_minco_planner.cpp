#include "terrain_minco_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <utility>

#include <nav2_core/exceptions.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_util/node_utils.hpp>
#include <opencv2/imgproc.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <navigation/terrain_core/path_planner/search/grid_utils.hpp>
#include <navigation/clearance_oracle.hpp>

namespace navigation
{
namespace
{

template<typename T>
T parameter(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name, const std::string & key, const T & default_value)
{
  const std::string full_name = name + "." + key;
  nav2_util::declare_parameter_if_not_declared(
    node, full_name, rclcpp::ParameterValue(default_value));
  T value{};
  node->get_parameter(full_name, value);
  return value;
}

bool identityOrientation(const geometry_msgs::msg::Quaternion & q)
{
  return std::abs(q.x) < 1.0e-6 && std::abs(q.y) < 1.0e-6 &&
         std::abs(q.z) < 1.0e-6 && std::abs(std::abs(q.w) - 1.0) < 1.0e-6;
}

std::optional<int8_t> sampleGrid(
  const nav_msgs::msg::OccupancyGrid & grid, const Eigen::Vector2d & point)
{
  if (!identityOrientation(grid.info.origin.orientation) || grid.info.resolution <= 0.0F) {
    return std::nullopt;
  }
  const double gx = (point.x() - grid.info.origin.position.x) / grid.info.resolution;
  const double gy = (point.y() - grid.info.origin.position.y) / grid.info.resolution;
  const int x = static_cast<int>(std::floor(gx));
  const int y = static_cast<int>(std::floor(gy));
  if (x < 0 || y < 0 || x >= static_cast<int>(grid.info.width) ||
    y >= static_cast<int>(grid.info.height))
  {
    return std::nullopt;
  }
  const size_t index = static_cast<size_t>(y) * grid.info.width + static_cast<size_t>(x);
  if (index >= grid.data.size()) {
    return std::nullopt;
  }
  return grid.data[index];
}

double pointDistanceSquared(const geometry_msgs::msg::PoseStamped & pose, const Eigen::Vector2d & p)
{
  const double dx = pose.pose.position.x - p.x();
  const double dy = pose.pose.position.y - p.y();
  return dx * dx + dy * dy;
}

geometry_msgs::msg::Quaternion yawQuaternion(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

}  // namespace

void TerrainMincoPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer>,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = std::move(name);
  costmap_ros_ = std::move(costmap_ros);
  costmap_ = costmap_ros_->getCostmap();
  const auto node = node_.lock();
  if (!node || !costmap_) {
    throw nav2_core::PlannerException("TerrainMincoPlanner configure failed");
  }

  occupied_cost_threshold_ = parameter(node, name_, "occupied_cost_threshold", 253);
  unknown_traversal_cost_ = parameter(node, name_, "unknown_traversal_cost", 50);
  max_astar_expansions_ = parameter(node, name_, "max_astar_expansions", 1000000);
  minco_max_iterations_ = parameter(node, name_, "minco_max_iterations", 400);
  flat_clearance_radius_ = parameter(node, name_, "flat_clearance_radius", 0.5656854249);
  terrain_half_length_ = parameter(node, name_, "terrain_half_length", 0.4);
  terrain_half_width_ = parameter(node, name_, "terrain_half_width", 0.4);
  obstacle_weight_ = parameter(node, name_, "obstacle_weight", 1.0);
  min_terrain_alignment_cosine_ = parameter(
    node, name_, "min_terrain_alignment_cosine", 0.9659258263);
  seed_spacing_ = parameter(node, name_, "seed_spacing", 0.45);
  output_spacing_ = parameter(node, name_, "output_spacing", 0.05);
  validation_spacing_ = parameter(node, name_, "validation_spacing", 0.025);
  max_path_deviation_ = parameter(node, name_, "max_path_deviation", 0.35);
  goal_cache_tolerance_ = parameter(node, name_, "goal_cache_tolerance", 0.05);
  curvature_max_ = parameter(node, name_, "curvature_max", 4.0);
  curvature_rate_max_ = parameter(node, name_, "curvature_rate_max", 8.0);
  minco_obstacle_weight_ = parameter(node, name_, "minco_obstacle_weight", 1000.0);
  minco_curvature_weight_ = parameter(node, name_, "minco_curvature_weight", 100.0);
  minco_curvature_rate_weight_ = parameter(
    node, name_, "minco_curvature_rate_weight", 100.0);
  minco_alignment_weight_ = parameter(node, name_, "minco_alignment_weight", 2000.0);
  optimization_check_frequency_ = parameter(
    node, name_, "optimization_check_frequency", 1.0);
  costmap_stable_time_ = parameter(node, name_, "costmap_stable_time", 0.8);
  optimization_cooldown_ = parameter(node, name_, "optimization_cooldown", 1.5);
  minimum_length_improvement_ratio_ = parameter(
    node, name_, "minimum_length_improvement_ratio", 0.08);
  const std::string terrain_type_topic = parameter(
    node, name_, "terrain_type_topic", std::string("/terrain/type"));
  const std::string terrain_direction_topic = parameter(
    node, name_, "terrain_direction_topic", std::string("/terrain/direction"));

  if (occupied_cost_threshold_ <= 0 || occupied_cost_threshold_ > 255 ||
    seed_spacing_ <= 0.0 || output_spacing_ <= 0.0 || validation_spacing_ <= 0.0 ||
    curvature_max_ <= 0.0 || curvature_rate_max_ <= 0.0 ||
    flat_clearance_radius_ <= 0.0 || terrain_half_length_ <= 0.0 ||
    terrain_half_width_ <= 0.0 ||
    optimization_check_frequency_ <= 0.0 || costmap_stable_time_ < 0.0 ||
    optimization_cooldown_ < 0.0 || minimum_length_improvement_ratio_ < 0.0 ||
    minimum_length_improvement_ratio_ >= 1.0 ||
    min_terrain_alignment_cosine_ <= 0.0 || min_terrain_alignment_cosine_ > 1.0)
  {
    throw nav2_core::PlannerException("TerrainMincoPlanner parameters are invalid");
  }

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  terrain_type_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    terrain_type_topic, qos,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(terrain_mutex_);
      terrain_type_ = msg;
    });
  terrain_direction_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    terrain_direction_topic, qos,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(terrain_mutex_);
      terrain_direction_ = msg;
    });
  plan_meta_pub_ = node->create_publisher<navigation::msg::PlanMeta>(
    "/terrain_minco/plan_meta", rclcpp::QoS(10).reliable());

  RCLCPP_INFO(
    node->get_logger(),
    "Configured %s: semantic A* + quintic MINCO, flat clearance %.3f m, terrain topics [%s, %s]",
    name_.c_str(), flat_clearance_radius_, terrain_type_topic.c_str(),
    terrain_direction_topic.c_str());
}

void TerrainMincoPlanner::cleanup()
{
  terrain_type_sub_.reset();
  terrain_direction_sub_.reset();
  plan_meta_pub_.reset();
  costmap_ = nullptr;
  costmap_ros_.reset();
  cached_path_.poses.clear();
  cached_path_s_.clear();
  have_cached_path_ = false;
  have_lethal_source_count_ = false;
  optimization_pending_ = false;
}

void TerrainMincoPlanner::activate()
{
  if (const auto node = node_.lock()) {
    RCLCPP_INFO(node->get_logger(), "Activating %s", name_.c_str());
  }
}

void TerrainMincoPlanner::deactivate()
{
  if (const auto node = node_.lock()) {
    RCLCPP_INFO(node->get_logger(), "Deactivating %s", name_.c_str());
  }
}

TerrainMincoPlanner::PlanningSnapshot TerrainMincoPlanner::makeSnapshot() const
{
  if (!costmap_) {
    throw nav2_core::PlannerException("costmap is unavailable");
  }

  unsigned int width = 0;
  unsigned int height = 0;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::vector<uint8_t> costs;
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
    width = costmap_->getSizeInCellsX();
    height = costmap_->getSizeInCellsY();
    resolution = costmap_->getResolution();
    origin_x = costmap_->getOriginX();
    origin_y = costmap_->getOriginY();
    const unsigned char * source = costmap_->getCharMap();
    costs.assign(source, source + static_cast<size_t>(width) * height);
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr terrain_type;
  nav_msgs::msg::OccupancyGrid::SharedPtr terrain_direction;
  {
    std::lock_guard<std::mutex> lock(terrain_mutex_);
    terrain_type = terrain_type_;
    terrain_direction = terrain_direction_;
  }

  terrain_core::GridGeometry geometry(
    static_cast<int>(width), static_cast<int>(height), resolution,
    Eigen::Vector2d(origin_x, origin_y));
  const std::vector<uint8_t> composite_costs = costs;
  std::vector<Eigen::Vector2d> directions(costs.size(), Eigen::Vector2d::Zero());
  std::vector<uint8_t> terrain_labels(costs.size(), 0);

  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      const size_t index = static_cast<size_t>(y) * width + x;
      if (costs[index] == nav2_costmap_2d::NO_INFORMATION) {
        costs[index] = static_cast<uint8_t>(std::clamp(unknown_traversal_cost_, 0, 252));
      } else if (costs[index] != nav2_costmap_2d::LETHAL_OBSTACLE) {
        // InflationLayer costs remain a soft A* preference.  They are capped below
        // the hard threshold because footprint clearance is rebuilt exactly once
        // from true lethal cells below (no double inflation).
        costs[index] = std::min<uint8_t>(costs[index], 252);
      }
      if (!terrain_type) {
        continue;
      }
      const Eigen::Vector2d point = geometry.cell_center(
        Eigen::Vector2i(static_cast<int>(x), static_cast<int>(y)));
      const auto raw_label = sampleGrid(*terrain_type, point);
      if (!raw_label || *raw_label < 0) {
        continue;
      }
      // This project exposes only four terrain classes. Do not reinterpret
      // labels from the upstream sentry project; unknown classes are blocked.
      const int label = *raw_label <= 3 ? static_cast<int>(*raw_label) :
        static_cast<int>(terrain_core::TerrainType::OBSTACLE);
      terrain_labels[index] = static_cast<uint8_t>(label);
      if (label == static_cast<int>(terrain_core::TerrainType::OBSTACLE)) {
        costs[index] = nav2_costmap_2d::LETHAL_OBSTACLE;
        continue;
      }
      if (label < static_cast<int>(terrain_core::TerrainType::SLOPE)) {
        continue;
      }
      const auto encoded_direction = terrain_direction ?
        sampleGrid(*terrain_direction, point) : std::nullopt;
      if (!encoded_direction || *encoded_direction < 0 || *encoded_direction > 100) {
        // A directional terrain without a direction is unsafe, so make it impassable.
        costs[index] = nav2_costmap_2d::LETHAL_OBSTACLE;
        continue;
      }
      const double angle = 2.0 * std::numbers::pi *
        static_cast<double>(*encoded_direction) / 100.0;
      directions[index] = Eigen::Vector2d(std::cos(angle), std::sin(angle));
    }
  }

  // Build the one hard collision oracle used by A*, LOS, cache validation and
  // MINCO. Flat cells use the circumscribed circle (yaw-independent). Directional
  // terrain uses the 0.8 m square aligned with its terrain axis, so a 1.0 m stair
  // is not incorrectly rejected by the 1.131 m circle diameter.
  const int radius_cells = static_cast<int>(std::ceil(
    std::max(flat_clearance_radius_, std::hypot(terrain_half_length_, terrain_half_width_)) /
    resolution));
  cv::Mat free_mask(static_cast<int>(height), static_cast<int>(width), CV_8UC1, cv::Scalar(255));
  bool have_lethal_cell = false;
  size_t lethal_source_count = 0;
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      const size_t index = static_cast<size_t>(y) * width + x;
      if (composite_costs[index] == nav2_costmap_2d::LETHAL_OBSTACLE ||
        terrain_labels[index] == static_cast<uint8_t>(terrain_core::TerrainType::OBSTACLE) ||
        costs[index] == nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        costs[index] = nav2_costmap_2d::LETHAL_OBSTACLE;
        free_mask.at<uint8_t>(static_cast<int>(y), static_cast<int>(x)) = 0;
        have_lethal_cell = true;
        ++lethal_source_count;
      }
    }
  }

  // Exact Euclidean distance transform is O(width*height), unlike expanding
  // every lethal cell over the footprint neighborhood. This runs on every 5 Hz
  // cache check, so the distinction is material on a map with long static walls.
  cv::Mat distance_cells;
  if (have_lethal_cell) {
    cv::distanceTransform(free_mask, distance_cells, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  } else {
    distance_cells = cv::Mat(
      static_cast<int>(height), static_cast<int>(width), CV_32FC1,
      cv::Scalar(std::numeric_limits<float>::infinity()));
  }
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      const size_t index = static_cast<size_t>(y) * width + x;
      if (terrain_labels[index] < static_cast<uint8_t>(terrain_core::TerrainType::SLOPE) &&
        static_cast<double>(distance_cells.at<float>(static_cast<int>(y), static_cast<int>(x))) *
        resolution <= flat_clearance_radius_)
      {
        costs[index] = nav2_costmap_2d::LETHAL_OBSTACLE;
      }
    }
  }

  // Directional terrain occupies only a small fraction of the map. Check its
  // aligned square footprint target-by-target instead of applying the flat
  // circumscribed circle, which would reject a valid 1.0 m stair passage.
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      const size_t index = static_cast<size_t>(y) * width + x;
      if (terrain_labels[index] < static_cast<uint8_t>(terrain_core::TerrainType::SLOPE) ||
        directions[index].norm() <= 0.1)
      {
        continue;
      }
      const Eigen::Vector2i cell(static_cast<int>(x), static_cast<int>(y));
      const Eigen::Vector2d center = geometry.cell_center(cell);
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        bool blocked = false;
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
          const Eigen::Vector2i obstacle = cell + Eigen::Vector2i(dx, dy);
          if (!geometry.contains_cell(obstacle) ||
            free_mask.at<uint8_t>(obstacle.y(), obstacle.x()) != 0)
          {
            continue;
          }
          if (footprintBlocksObstacle(
              center - geometry.cell_center(obstacle), terrain_labels[index], directions[index],
              flat_clearance_radius_, terrain_half_length_, terrain_half_width_))
          {
            blocked = true;
            break;
          }
        }
        if (blocked) {
          costs[index] = nav2_costmap_2d::LETHAL_OBSTACLE;
          break;
        }
      }
    }
  }

  PlanningSnapshot snapshot;
  snapshot.cost = std::make_shared<terrain_core::CostMap>(geometry, std::move(costs));
  snapshot.direction = std::make_shared<terrain_core::DirectionMap>(
    geometry, std::move(directions), std::move(terrain_labels));
  snapshot.lethal_source_count = lethal_source_count;

  for (uint8_t label = static_cast<uint8_t>(terrain_core::TerrainType::SLOPE);
    label <= static_cast<uint8_t>(terrain_core::TerrainType::STEP); ++label)
  {
    terrain_core::TraversalMode mode;
    mode.name = "geometric";
    mode.velocity_window = {0.0, 100.0};
    mode.run_up = label == static_cast<uint8_t>(terrain_core::TerrainType::SLOPE) ? 0.2 : 0.5;
    snapshot.terrain.selected_modes[label - 2].up = mode;
    snapshot.terrain.selected_modes[label - 2].down = mode;
    snapshot.terrain.rules[label] = {true, true};
  }
  return snapshot;
}

bool TerrainMincoPlanner::lineOfSight(
  const Eigen::Vector2d & from, const Eigen::Vector2d & to,
  const PlanningSnapshot & snapshot) const
{
  const auto from_cell = snapshot.cost->geometry.containing_cell(from);
  const auto to_cell = snapshot.cost->geometry.containing_cell(to);
  if (!from_cell || !to_cell) {
    return false;
  }
  const double length = (to - from).norm();
  const int samples = std::max(1, static_cast<int>(std::ceil(
    length / (0.5 * snapshot.cost->geometry.resolution()))));
  for (int i = 0; i <= samples; ++i) {
    const Eigen::Vector2d point = from +
      (static_cast<double>(i) / samples) * (to - from);
    const auto cost = snapshot.cost->sample_map(point);
    if (!cost || cost->value >= occupied_cost_threshold_) {
      return false;
    }
  }
  Eigen::Vector2i current = *from_cell;
  for (const auto & crossing : terrain_core::trace_grid_crossings(
      snapshot.cost->geometry, from, to))
  {
    if (!terrain_core::grid_cell_traversable(
        *snapshot.cost, crossing.to, occupied_cost_threshold_) ||
      !terrain_core::grid_edge_avoids_corner_cutting(
        *snapshot.cost, crossing.from, crossing.to, occupied_cost_threshold_) ||
      !terrain_core::directed_terrain_edge_allowed(
        *snapshot.direction, snapshot.terrain, crossing.from, crossing.to,
        min_terrain_alignment_cosine_))
    {
      return false;
    }
    current = crossing.to;
  }
  return terrain_core::same_cell(current, *to_cell);
}

bool TerrainMincoPlanner::cachedPathSafe(
  const Eigen::Vector2d & start, const Eigen::Vector2d & goal,
  const PlanningSnapshot & snapshot, size_t & nearest_index,
  std::string & invalid_reason) const
{
  if (!have_cached_path_ || cached_path_.poses.size() < 2) {
    invalid_reason = "no_cached_path";
    return false;
  }
  if ((goal - cached_goal_).norm() > goal_cache_tolerance_) {
    invalid_reason = "goal_changed";
    return false;
  }
  nearest_index = 0;
  double nearest_sq = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < cached_path_.poses.size(); ++i) {
    const double d2 = pointDistanceSquared(cached_path_.poses[i], start);
    if (d2 < nearest_sq) {
      nearest_sq = d2;
      nearest_index = i;
    }
  }
  if (std::sqrt(nearest_sq) > max_path_deviation_) {
    invalid_reason = "robot_deviated";
    return false;
  }
  const size_t join = std::min(nearest_index + 1, cached_path_.poses.size() - 1);
  const Eigen::Vector2d join_point(
    cached_path_.poses[join].pose.position.x,
    cached_path_.poses[join].pose.position.y);
  if (!lineOfSight(start, join_point, snapshot)) {
    invalid_reason = "cache_join_blocked";
    return false;
  }
  for (size_t i = join; i + 1 < cached_path_.poses.size(); ++i) {
    const Eigen::Vector2d a(
      cached_path_.poses[i].pose.position.x, cached_path_.poses[i].pose.position.y);
    const Eigen::Vector2d b(
      cached_path_.poses[i + 1].pose.position.x,
      cached_path_.poses[i + 1].pose.position.y);
    if (!lineOfSight(a, b, snapshot)) {
      invalid_reason = "obstacle_entered_corridor";
      return false;
    }
  }
  return true;
}

std::vector<Eigen::Vector2d> TerrainMincoPlanner::simplifyRoute(
  const terrain_core::SpatialRoute & route,
  const Eigen::Vector2d & start, const Eigen::Vector2d & goal,
  const PlanningSnapshot & snapshot) const
{
  std::vector<Eigen::Vector2d> raw;
  raw.reserve(route.raw_path.size());
  for (const auto & cell : route.raw_path) {
    raw.push_back(snapshot.cost->geometry.cell_center(cell));
  }
  raw.front() = start;
  raw.back() = goal;

  std::vector<Eigen::Vector2d> anchors{raw.front()};
  size_t current = 0;
  while (current + 1 < raw.size()) {
    size_t next = raw.size() - 1;
    while (next > current + 1 && !lineOfSight(raw[current], raw[next], snapshot)) {
      --next;
    }
    anchors.push_back(raw[next]);
    current = next;
  }

  std::vector<Eigen::Vector2d> seeded{anchors.front()};
  for (size_t i = 0; i + 1 < anchors.size(); ++i) {
    const double distance = (anchors[i + 1] - anchors[i]).norm();
    const int pieces = std::max(1, static_cast<int>(std::ceil(distance / seed_spacing_)));
    for (int k = 1; k <= pieces; ++k) {
      seeded.push_back(anchors[i] +
        (static_cast<double>(k) / pieces) * (anchors[i + 1] - anchors[i]));
    }
  }
  return seeded;
}

std::vector<terrain_core::GeometricBoundary> TerrainMincoPlanner::makeMincoSeed(
  const std::vector<Eigen::Vector2d> & anchors,
  const terrain_core::DirectionMap & direction) const
{
  std::vector<terrain_core::GeometricBoundary> boundaries(anchors.size());
  for (size_t i = 0; i < anchors.size(); ++i) {
    Eigen::Vector2d tangent;
    if (i == 0) {
      tangent = anchors[1] - anchors[0];
    } else if (i + 1 == anchors.size()) {
      tangent = anchors[i] - anchors[i - 1];
    } else {
      tangent = anchors[i + 1] - anchors[i - 1];
    }
    if (tangent.norm() <= 1.0e-9) {
      tangent = Eigen::Vector2d::UnitX();
    }
    tangent.normalize();
    if (const auto terrain_direction = direction.sample_map(anchors[i]);
      terrain_direction && terrain_direction->value.norm() > 0.1)
    {
      Eigen::Vector2d aligned = terrain_direction->value.normalized();
      if (aligned.dot(tangent) < 0.0) {
        aligned = -aligned;
      }
      tangent = aligned;
    }
    boundaries[i] = {anchors[i], tangent, 0.0};
  }
  return boundaries;
}

bool TerrainMincoPlanner::validateTrajectory(
  const terrain_core::MincoTrajectory & trajectory,
  const PlanningSnapshot & snapshot, std::string & reason) const
{
  if (trajectory.empty() || !std::isfinite(trajectory.total_arc_length()) ||
    trajectory.total_arc_length() <= 0.0)
  {
    reason = "empty or non-finite trajectory";
    return false;
  }
  const int intervals = std::max(1, static_cast<int>(std::ceil(
    trajectory.total_arc_length() / validation_spacing_)));
  for (int i = 0; i <= intervals; ++i) {
    const double s = trajectory.total_arc_length() * i / intervals;
    const auto sample = trajectory.eval_arc_length(s);
    if (!sample.p.allFinite() || !std::isfinite(sample.kappa) ||
      !std::isfinite(sample.kappa_rate))
    {
      reason = "trajectory contains a non-finite sample";
      return false;
    }
    const auto cost = snapshot.cost->sample_map(sample.p);
    if (!cost || cost->value >= occupied_cost_threshold_) {
      reason = "trajectory intersects an occupied cell";
      return false;
    }
    if (std::abs(sample.kappa) > curvature_max_ * 1.02 ||
      std::abs(sample.kappa_rate) > curvature_rate_max_ * 1.02)
    {
      reason = "trajectory exceeds the configured geometric limits";
      return false;
    }
    const auto cell = snapshot.direction->geometry.containing_cell(sample.p);
    if (!cell || !snapshot.direction->is_terrain_body_cell(*cell)) {
      continue;
    }
    const Eigen::Vector2d terrain = snapshot.direction->raw_direction_at_cell(*cell);
    const Eigen::Vector2d heading(std::cos(sample.theta), std::sin(sample.theta));
    if (terrain.norm() <= 1.0e-9 ||
      std::abs(heading.dot(terrain.normalized())) < min_terrain_alignment_cosine_)
    {
      reason = "trajectory crosses directional terrain sideways";
      return false;
    }
    const bool going_up = heading.dot(terrain.normalized()) >= 0.0;
    if (!snapshot.terrain.selected_mode(
        snapshot.direction->terrain_label_at_cell(*cell), going_up))
    {
      reason = "trajectory uses a prohibited terrain direction";
      return false;
    }
  }
  return true;
}

nav_msgs::msg::Path TerrainMincoPlanner::trajectoryToPath(
  const terrain_core::MincoTrajectory & trajectory,
  const std_msgs::msg::Header & header,
  const geometry_msgs::msg::Quaternion & goal_orientation) const
{
  nav_msgs::msg::Path path;
  path.header = header;
  const int intervals = std::max(1, static_cast<int>(std::ceil(
    trajectory.total_arc_length() / output_spacing_)));
  path.poses.reserve(static_cast<size_t>(intervals) + 1);
  for (int i = 0; i <= intervals; ++i) {
    const auto sample = trajectory.eval_arc_length(
      trajectory.total_arc_length() * i / intervals);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = sample.p.x();
    pose.pose.position.y = sample.p.y();
    pose.pose.orientation = yawQuaternion(sample.theta);
    path.poses.push_back(std::move(pose));
  }
  // Pose headings describe the geometric tangent except at the goal, where the
  // caller's requested final orientation is part of the NavigateToPose contract.
  path.poses.back().pose.orientation = goal_orientation;
  return path;
}

nav_msgs::msg::Path TerrainMincoPlanner::trimmedCachedPath(
  const geometry_msgs::msg::PoseStamped & start, const size_t nearest_index) const
{
  nav_msgs::msg::Path result;
  result.header = cached_path_.header;
  if (const auto node = node_.lock()) {
    result.header.stamp = node->now();
  }
  geometry_msgs::msg::PoseStamped first = start;
  first.header = result.header;
  const size_t next = std::min(nearest_index + 1, cached_path_.poses.size() - 1);
  const double dx = cached_path_.poses[next].pose.position.x - first.pose.position.x;
  const double dy = cached_path_.poses[next].pose.position.y - first.pose.position.y;
  if (std::hypot(dx, dy) > 1.0e-6) {
    first.pose.orientation = yawQuaternion(std::atan2(dy, dx));
  }
  result.poses.push_back(first);
  for (size_t i = next; i < cached_path_.poses.size(); ++i) {
    auto pose = cached_path_.poses[i];
    pose.header = result.header;
    result.poses.push_back(std::move(pose));
  }
  return result;
}

void TerrainMincoPlanner::publishPlanMeta(
  const std_msgs::msg::Header & header, const bool replanned,
  const float path_start_s, const std::string & reason)
{
  if (!plan_meta_pub_) {
    return;
  }
  navigation::msg::PlanMeta meta;
  meta.header = header;
  meta.path_id = path_id_;
  meta.publish_seq = ++publish_seq_;
  meta.replanned = replanned;
  meta.path_start_s = path_start_s;
  meta.reason = reason;
  plan_meta_pub_->publish(meta);
}

nav_msgs::msg::Path TerrainMincoPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  const auto planning_started = std::chrono::steady_clock::now();
  const auto node = node_.lock();
  if (!node) {
    throw nav2_core::PlannerException("planner node expired");
  }
  if (start.header.frame_id != costmap_ros_->getGlobalFrameID() ||
    goal.header.frame_id != costmap_ros_->getGlobalFrameID())
  {
    throw nav2_core::PlannerException("start and goal must be in the global costmap frame");
  }

  const Eigen::Vector2d start_xy(start.pose.position.x, start.pose.position.y);
  const Eigen::Vector2d goal_xy(goal.pose.position.x, goal.pose.position.y);
  const PlanningSnapshot snapshot = makeSnapshot();

  size_t nearest_index = 0;
  std::string replan_reason;
  const bool cache_safe = cachedPathSafe(
    start_xy, goal_xy, snapshot, nearest_index, replan_reason);
  const auto return_cached = [&](const std::string & reason) {
    auto result = trimmedCachedPath(start, nearest_index);
    const float path_start_s = nearest_index < cached_path_s_.size() ?
      static_cast<float>(cached_path_s_[nearest_index]) : 0.0F;
    publishPlanMeta(result.header, false, path_start_s, reason);
    return result;
  };

  const auto steady_now = std::chrono::steady_clock::now();
  if (have_lethal_source_count_ &&
    snapshot.lethal_source_count < last_lethal_source_count_ && cache_safe &&
    !optimization_pending_)
  {
    optimization_pending_ = true;
    optimization_pending_since_ = steady_now;
  }
  last_lethal_source_count_ = snapshot.lethal_source_count;
  have_lethal_source_count_ = true;

  bool optional_optimization = false;
  double cached_remaining_length = 0.0;
  if (cache_safe) {
    cached_remaining_length = nearest_index < cached_path_s_.size() && !cached_path_s_.empty() ?
      cached_path_s_.back() - cached_path_s_[nearest_index] : 0.0;
    const double check_period = 1.0 / optimization_check_frequency_;
    const bool stable_long_enough = optimization_pending_ &&
      steady_now - optimization_pending_since_ >=
      std::chrono::duration<double>(costmap_stable_time_);
    const bool check_due = last_optimization_attempt_.time_since_epoch().count() == 0 ||
      steady_now - last_optimization_attempt_ >= std::chrono::duration<double>(check_period);
    const bool cooldown_done = last_accepted_replan_.time_since_epoch().count() == 0 ||
      steady_now - last_accepted_replan_ >=
      std::chrono::duration<double>(optimization_cooldown_);
    optional_optimization = stable_long_enough && check_due && cooldown_done &&
      cached_remaining_length > output_spacing_;
    if (!optional_optimization) {
      RCLCPP_DEBUG(node->get_logger(), "%s reused its safe cached path", name_.c_str());
      return return_cached("cache_trim");
    }
    last_optimization_attempt_ = steady_now;
    replan_reason = "costmap_improved";
  } else {
    RCLCPP_INFO(
      node->get_logger(), "%s cache invalid: %s; starting full replan",
      name_.c_str(), replan_reason.c_str());
  }

  const auto start_cell = snapshot.cost->geometry.containing_cell(start_xy);
  const auto goal_cell = snapshot.cost->geometry.containing_cell(goal_xy);
  if (!start_cell || !goal_cell) {
    throw nav2_core::PlannerException("start or goal is outside the rolling global costmap");
  }
  const auto endpoint_error = [&](const char * name, const Eigen::Vector2d & point,
      const Eigen::Vector2i & cell) -> std::string {
      const int cost = static_cast<int>(snapshot.cost->raw_cost_at_cell(cell));
      const int terrain = static_cast<int>(snapshot.direction->terrain_label_at_cell(cell));
      return std::string(name) + " is blocked by the planner footprint clearance: xy=(" +
             std::to_string(point.x()) + ", " + std::to_string(point.y()) +
             "), cell=(" + std::to_string(cell.x()) + ", " +
             std::to_string(cell.y()) + "), cost=" + std::to_string(cost) +
             ", terrain=" + std::to_string(terrain);
    };
  if (!terrain_core::grid_cell_traversable(
      *snapshot.cost, *start_cell, occupied_cost_threshold_))
  {
    throw nav2_core::PlannerException(endpoint_error("start", start_xy, *start_cell));
  }
  if (!terrain_core::grid_cell_traversable(
      *snapshot.cost, *goal_cell, occupied_cost_threshold_))
  {
    throw nav2_core::PlannerException(endpoint_error("goal", goal_xy, *goal_cell));
  }

  // The upstream terrain A* deliberately rejects routes whose first/last cell
  // is terrain body (its full executor locks replanning during a committed
  // traversal). This planner has no execution state machine, so make only the
  // endpoint cells flat for passage extraction. Adjacent edges, MINCO and the
  // final validator still use the real direction field.
  auto astar_direction = snapshot.direction;
  if (snapshot.direction->is_terrain_body_cell(*start_cell) ||
    snapshot.direction->is_terrain_body_cell(*goal_cell))
  {
    auto directions = snapshot.direction->data;
    auto labels = snapshot.direction->terrain;
    const auto flatten = [&](const Eigen::Vector2i & cell) {
        const size_t index = static_cast<size_t>(cell.y()) *
          static_cast<size_t>(snapshot.direction->geometry.width()) +
          static_cast<size_t>(cell.x());
        directions[index] = Eigen::Vector2d::Zero();
        labels[index] = static_cast<uint8_t>(terrain_core::TerrainType::FLAT);
      };
    flatten(*start_cell);
    flatten(*goal_cell);
    astar_direction = std::make_shared<terrain_core::DirectionMap>(
      snapshot.direction->geometry, std::move(directions), std::move(labels));
  }

  const terrain_core::SpatialGridAstar astar({obstacle_weight_, max_astar_expansions_});
  const auto search = astar.search(
    *snapshot.cost, *astar_direction, snapshot.terrain,
    *start_cell, *goal_cell, occupied_cost_threshold_, min_terrain_alignment_cosine_);
  if (!search.success) {
    if (optional_optimization) {
      optimization_pending_ = false;
      return return_cached("optimization_no_path");
    }
    throw nav2_core::PlannerException("terrain A* failed: " + search.error);
  }
  const auto anchors = simplifyRoute(search.route, start_xy, goal_xy, snapshot);
  if (anchors.size() < 2) {
    if (optional_optimization) {
      optimization_pending_ = false;
      return return_cached("optimization_invalid_seed");
    }
    throw nav2_core::PlannerException("terrain A* produced an invalid seed");
  }
  double astar_length = 0.0;
  for (size_t i = 1; i < anchors.size(); ++i) {
    astar_length += (anchors[i] - anchors[i - 1]).norm();
  }
  if (optional_optimization &&
    astar_length >= cached_remaining_length * (1.0 - minimum_length_improvement_ratio_))
  {
    optimization_pending_ = false;
    RCLCPP_DEBUG(
      node->get_logger(),
      "%s kept path_id=%lu: candidate A* %.2f m vs cached %.2f m",
      name_.c_str(), static_cast<unsigned long>(path_id_), astar_length,
      cached_remaining_length);
    return return_cached("optimization_not_better");
  }
  const auto boundaries = makeMincoSeed(anchors, *snapshot.direction);
  std::vector<double> durations;
  durations.reserve(boundaries.size() - 1);
  for (size_t i = 0; i + 1 < boundaries.size(); ++i) {
    durations.push_back(std::max(
      (boundaries[i + 1].position - boundaries[i].position).norm(), 0.05));
  }

  terrain_core::MincoOptimizer::Params minco;
  minco.weights.energy = 0.8;
  minco.weights.time = 12.0;
  minco.weights.obstacle = minco_obstacle_weight_;
  minco.weights.curvature = minco_curvature_weight_;
  minco.weights.curvature_rate = minco_curvature_rate_weight_;
  minco.weights.directed_regularity = 1000.0;
  minco.weights.traversal_velocity_window = 0.0;
  minco.weights.traversal_alignment = minco_alignment_weight_;
  minco.weights.prohibited_traversal = 1000.0;
  minco.weights.runup_curvature = 200.0;
  minco.geometry = {curvature_max_, curvature_rate_max_, 1.0e-6};
  // This is the smooth forward/backward transition band inside MINCO, not the
  // hard terrain alignment limit. A* and final validation enforce that limit.
  minco.directed_cosine_min = 0.1;
  minco.terrain_gate = {0.1, 0.9};
  minco.samples_per_segment = 16;
  minco.max_iterations = minco_max_iterations_;
  minco.optimizer.max_function_evaluations = std::max(800, minco_max_iterations_ * 3);
  minco.runup_body_norm_lo = 0.9;
  minco.runup_body_norm_hi = 0.95;
  minco.runup_saturation_length = 0.05;
  minco.runup_transition_distance = 0.1;

  const terrain_core::MincoOptimizer optimizer(minco);
  const auto optimized = optimizer.optimize(
    boundaries, durations, *snapshot.cost, *snapshot.direction, snapshot.terrain);
  if (!optimized.success) {
    if (optional_optimization) {
      optimization_pending_ = false;
      return return_cached("optimization_minco_failed");
    }
    throw nav2_core::PlannerException("MINCO failed: " + optimized.error);
  }
  std::string validation_error;
  if (!validateTrajectory(optimized.trajectory, snapshot, validation_error)) {
    if (optional_optimization) {
      optimization_pending_ = false;
      return return_cached("optimization_validation_failed");
    }
    throw nav2_core::PlannerException("MINCO validation failed: " + validation_error);
  }
  if (optional_optimization &&
    optimized.trajectory.total_arc_length() >=
    cached_remaining_length * (1.0 - minimum_length_improvement_ratio_))
  {
    optimization_pending_ = false;
    return return_cached("optimization_not_better");
  }

  std_msgs::msg::Header header;
  header.stamp = node->now();
  header.frame_id = costmap_ros_->getGlobalFrameID();
  cached_path_ = trajectoryToPath(optimized.trajectory, header, goal.pose.orientation);
  cached_path_s_.assign(cached_path_.poses.size(), 0.0);
  for (size_t i = 1; i < cached_path_.poses.size(); ++i) {
    const auto & a = cached_path_.poses[i - 1].pose.position;
    const auto & b = cached_path_.poses[i].pose.position;
    cached_path_s_[i] = cached_path_s_[i - 1] + std::hypot(b.x - a.x, b.y - a.y);
  }
  cached_goal_ = goal_xy;
  have_cached_path_ = true;
  optimization_pending_ = false;
  last_accepted_replan_ = std::chrono::steady_clock::now();
  ++path_id_;
  publishPlanMeta(header, true, 0.0F, replan_reason);
  const double planning_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - planning_started).count();
  RCLCPP_INFO(
    node->get_logger(),
    "%s accepted path_id=%lu: reason=%s, A* expansions=%d, seed=%zu, "
    "MINCO segments=%d, path=%.2f m, total=%.1f ms",
    name_.c_str(), static_cast<unsigned long>(path_id_), replan_reason.c_str(),
    search.route.expansions, anchors.size(),
    optimized.trajectory.segment_count(), optimized.trajectory.total_arc_length(), planning_ms);
  return cached_path_;
}

}  // namespace navigation

PLUGINLIB_EXPORT_CLASS(navigation::TerrainMincoPlanner, nav2_core::GlobalPlanner)
