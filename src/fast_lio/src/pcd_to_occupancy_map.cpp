// 先验 PCD 地图 -> 2D 占据栅格。
//
// 取 z∈[z_min, z_max] 的点投影到 XY 平面，发布 /map（OccupancyGrid，
// transient_local，和 map_server 同款 QoS），可选存成 .pgm/.yaml。

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <utility>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace
{
constexpr int8_t kOccupied = 100;
constexpr int8_t kFree = 0;
constexpr int8_t kUnknown = -1;
constexpr int8_t kTerrainFlat = 0;
constexpr int8_t kTerrainObstacle = 1;
constexpr int8_t kTerrainSlope = 2;
constexpr int8_t kTerrainStep = 3;
constexpr double kPi = 3.14159265358979323846;

struct StepSeed
{
  int x;
  int y;
  double dx;
  double dy;
};
}  // namespace

class PcdToOccupancyMap : public rclcpp::Node
{
public:
  PcdToOccupancyMap() : Node("pcd_to_occupancy_map")
  {
    pcd_file_ = declare_parameter<std::string>("pcd_file", "");
    resolution_ = declare_parameter<double>("resolution", 0.05);
    z_min_ = declare_parameter<double>("z_min", 0.15);
    z_max_ = declare_parameter<double>("z_max", 1.5);
    min_points_ = declare_parameter<int>("min_points_per_cell", 1);
    // true: 没观测过的格子标 unknown(-1)；false: 一律当空闲(0)
    mark_unknown_ = declare_parameter<bool>("mark_unknown", true);
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    topic_ = declare_parameter<std::string>("occupancy_topic", "/map");
    save_prefix_ = declare_parameter<std::string>("save_prefix", "");
    auto_terrain_ = declare_parameter<bool>("auto_terrain", true);
    terrain_type_topic_ = declare_parameter<std::string>(
      "terrain_type_topic", "/terrain/type");
    terrain_direction_topic_ = declare_parameter<std::string>(
      "terrain_direction_topic", "/terrain/direction");
    // 给 RViz 用的彩色叠加层：/terrain/type 的取值只有 0..3，RViz 的 Map 显示
    // （map/costmap/raw 三种调色板）都无法把 1/2/3 区分开，所以额外把坡道/台阶
    // 逐格发成一个带 RGB 的点云，按类别着色。
    terrain_cloud_topic_ = declare_parameter<std::string>(
      "terrain_cloud_topic", "/terrain/colored");
    surface_z_min_ = declare_parameter<double>("surface_z_min", -0.30);
    surface_z_max_ = declare_parameter<double>("surface_z_max", 1.20);
    slope_min_deg_ = declare_parameter<double>("slope_min_deg", 5.0);
    slope_max_deg_ = declare_parameter<double>("slope_max_deg", 35.0);
    slope_fit_radius_ = declare_parameter<double>("slope_fit_radius", 0.20);
    slope_max_residual_ = declare_parameter<double>("slope_max_residual", 0.020);
    stair_rise_ = declare_parameter<double>("stair_rise", 0.15);
    stair_tread_ = declare_parameter<double>("stair_tread", 0.30);
    stair_rise_tolerance_ = declare_parameter<double>("stair_rise_tolerance", 0.025);
    stair_half_width_ = declare_parameter<double>("stair_half_width", 0.55);
    stair_min_width_ = declare_parameter<double>("stair_min_width", 0.65);
    terrain_min_component_area_ = declare_parameter<double>(
      "terrain_min_component_area", 0.50);

    if (pcd_file_.empty()) {
      throw std::runtime_error("参数 pcd_file 未设置，需要指向先验 PCD 地图");
    }
    if (z_max_ <= z_min_) {
      throw std::runtime_error("z_max 必须大于 z_min");
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(topic_, qos);
    terrain_type_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      terrain_type_topic_, qos);
    terrain_direction_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      terrain_direction_topic_, qos);
    terrain_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      terrain_cloud_topic_, qos);

    RCLCPP_INFO(get_logger(), "读取先验地图 %s ...", pcd_file_.c_str());
    build_grid();
    log_source();
    publisher_->publish(grid_);
    if (auto_terrain_) {
      terrain_type_publisher_->publish(terrain_type_grid_);
      terrain_direction_publisher_->publish(terrain_direction_grid_);
      if (!terrain_cloud_.data.empty()) {
        terrain_cloud_publisher_->publish(terrain_cloud_);
      }
    }
    RCLCPP_INFO(
      get_logger(), "栅格图 %ux%u @ %.3fm，原点 (%.2f, %.2f)，已发布到 %s",
      grid_.info.width, grid_.info.height, resolution_,
      grid_.info.origin.position.x, grid_.info.origin.position.y, topic_.c_str());
    if (auto_terrain_) {
      RCLCPP_INFO(
        get_logger(),
        "PCD 地形识别已发布: %s 和 %s（平地=%zu 障碍=%zu 坡道=%zu 台阶=%zu）",
        terrain_type_topic_.c_str(), terrain_direction_topic_.c_str(),
        terrain_counts_[0], terrain_counts_[1], terrain_counts_[2], terrain_counts_[3]);
      RCLCPP_INFO(
        get_logger(), "%s 已发布（坡道/台阶彩色叠加，%zu 个格子）",
        terrain_cloud_topic_.c_str(),
        static_cast<std::size_t>(terrain_cloud_.width) *
        static_cast<std::size_t>(terrain_cloud_.height));
    }

    if (!save_prefix_.empty()) {
      save_map();
    }
  }

private:
  void log_source() const
  {
    // 把真正读到的文件信息打出来，避免"看的到底是哪张图"说不清。
    std::error_code ec;
    const auto resolved = std::filesystem::canonical(pcd_file_, ec);
    const std::string path = ec ? pcd_file_ : resolved.string();

    struct stat file_info {};
    std::string stamp = "未知";
    if (stat(path.c_str(), &file_info) == 0) {
      char buffer[32] = {0};
      std::strftime(buffer, sizeof(buffer), "%F %T",
                    std::localtime(&file_info.st_mtime));
      stamp = buffer;
    }
    RCLCPP_INFO(get_logger(), "先验地图实际文件: %s", path.c_str());
    RCLCPP_INFO(
      get_logger(),
      "  大小 %ld 字节 | 修改时间 %s | 总点数 %zu | 表面 z∈[%.2f,%.2f] 内 %zu 点",
      static_cast<long>(file_info.st_size), stamp.c_str(), total_points_, surface_z_min_,
      surface_z_max_, band_points_);
  }

  [[nodiscard]] static std::size_t cell_index(
    const int x, const int y, const int width)
  {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
           static_cast<std::size_t>(x);
  }

  void detect_slopes(
    const std::vector<float> & surface, const int width, const int height,
    std::vector<int8_t> & terrain, std::vector<double> & direction_x,
    std::vector<double> & direction_y) const
  {
    const int radius = std::max(1, static_cast<int>(std::ceil(
      slope_fit_radius_ / resolution_)));
    const double min_slope = std::tan(slope_min_deg_ * kPi / 180.0);
    const double max_slope = std::tan(slope_max_deg_ * kPi / 180.0);

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const std::size_t center_index = cell_index(x, y, width);
        if (!std::isfinite(surface[center_index])) {
          continue;
        }
        double sx = 0.0, sy = 0.0, sz = 0.0;
        double sxx = 0.0, sxy = 0.0, syy = 0.0;
        double sxz = 0.0, syz = 0.0;
        double min_height = std::numeric_limits<double>::infinity();
        double max_height = -std::numeric_limits<double>::infinity();
        int samples = 0;
        for (int dy = -radius; dy <= radius; ++dy) {
          for (int dx = -radius; dx <= radius; ++dx) {
            const int nx = x + dx;
            const int ny = y + dy;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height ||
              dx * dx + dy * dy > radius * radius)
            {
              continue;
            }
            const float height_value = surface[cell_index(nx, ny, width)];
            if (!std::isfinite(height_value)) {
              continue;
            }
            const double px = static_cast<double>(dx) * resolution_;
            const double py = static_cast<double>(dy) * resolution_;
            const double pz = height_value;
            sx += px;
            sy += py;
            sz += pz;
            sxx += px * px;
            sxy += px * py;
            syy += py * py;
            sxz += px * pz;
            syz += py * pz;
            min_height = std::min(min_height, pz);
            max_height = std::max(max_height, pz);
            ++samples;
          }
        }
        if (samples < 8 || max_height - min_height < 0.04) {
          continue;
        }

        const double inv_n = 1.0 / static_cast<double>(samples);
        const double cxx = sxx - sx * sx * inv_n;
        const double cxy = sxy - sx * sy * inv_n;
        const double cyy = syy - sy * sy * inv_n;
        const double cxz = sxz - sx * sz * inv_n;
        const double cyz = syz - sy * sz * inv_n;
        const double determinant = cxx * cyy - cxy * cxy;
        if (std::abs(determinant) < 1.0e-9) {
          continue;
        }
        const double gx = (cxz * cyy - cyz * cxy) / determinant;
        const double gy = (cyz * cxx - cxz * cxy) / determinant;
        const double gradient = std::hypot(gx, gy);
        if (gradient < min_slope || gradient > max_slope) {
          continue;
        }
        const double mean_x = sx * inv_n;
        const double mean_y = sy * inv_n;
        const double mean_z = sz * inv_n;
        double squared_error = 0.0;
        int residual_samples = 0;
        for (int dy = -radius; dy <= radius; ++dy) {
          for (int dx = -radius; dx <= radius; ++dx) {
            const int nx = x + dx;
            const int ny = y + dy;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height ||
              dx * dx + dy * dy > radius * radius)
            {
              continue;
            }
            const float height_value = surface[cell_index(nx, ny, width)];
            if (!std::isfinite(height_value)) {
              continue;
            }
            const double px = static_cast<double>(dx) * resolution_;
            const double py = static_cast<double>(dy) * resolution_;
            const double predicted = mean_z + gx * (px - mean_x) + gy * (py - mean_y);
            const double error = static_cast<double>(height_value) - predicted;
            squared_error += error * error;
            ++residual_samples;
          }
        }
        const double residual = std::sqrt(
          squared_error / std::max(1, residual_samples));
        if (residual > slope_max_residual_) {
          continue;
        }
        terrain[center_index] = kTerrainSlope;
        direction_x[center_index] = gx / gradient;
        direction_y[center_index] = gy / gradient;
      }
    }
  }

  void detect_stairs(
    const std::vector<float> & surface, const int width, const int height,
    std::vector<int8_t> & terrain, std::vector<double> & direction_x,
    std::vector<double> & direction_y) const
  {
    constexpr std::array<std::pair<int, int>, 4> edge_directions {{
      {1, 0}, {0, 1}, {1, 1}, {1, -1}}};
    std::vector<StepSeed> seeds;

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const float first = surface[cell_index(x, y, width)];
        if (!std::isfinite(first)) {
          continue;
        }
        for (const auto & [ox, oy] : edge_directions) {
          const int nx = x + ox;
          const int ny = y + oy;
          if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
            continue;
          }
          const float second = surface[cell_index(nx, ny, width)];
          if (!std::isfinite(second)) {
            continue;
          }
          const double height_delta = static_cast<double>(second - first);
          if (std::abs(std::abs(height_delta) - stair_rise_) > stair_rise_tolerance_) {
            continue;
          }
          const double grid_norm = std::hypot(static_cast<double>(ox), static_cast<double>(oy));
          const double sign = height_delta > 0.0 ? 1.0 : -1.0;
          const int step_x = static_cast<int>(sign) * ox;
          const int step_y = static_cast<int>(sign) * oy;
          const double up_x = sign * static_cast<double>(ox) / grid_norm;
          const double up_y = sign * static_cast<double>(oy) / grid_norm;
          const int low_x = height_delta > 0.0 ? x : nx;
          const int low_y = height_delta > 0.0 ? y : ny;
          const double low_height = std::min(first, second);
          const int nominal_steps = std::max(2, static_cast<int>(std::lround(
            stair_tread_ / (resolution_ * grid_norm))));
          bool repeated_rise = false;
          for (int delta_steps = -2; delta_steps <= 2 && !repeated_rise; ++delta_steps) {
            const int travel = std::max(2, nominal_steps + delta_steps);
            const int bx = low_x + step_x * travel;
            const int by = low_y + step_y * travel;
            const int ax = bx + step_x;
            const int ay = by + step_y;
            if (bx < 0 || bx >= width || by < 0 || by >= height ||
              ax < 0 || ax >= width || ay < 0 || ay >= height)
            {
              continue;
            }
            const float before = surface[cell_index(bx, by, width)];
            const float after = surface[cell_index(ax, ay, width)];
            if (!std::isfinite(before) || !std::isfinite(after)) {
              continue;
            }
            const double next_delta = static_cast<double>(after - before);
            repeated_rise = std::abs(next_delta - stair_rise_) <= stair_rise_tolerance_ &&
              static_cast<double>(before) >= low_height + 0.5 * stair_rise_;
          }
          if (repeated_rise) {
            seeds.push_back({low_x, low_y, up_x, up_y});
          }
        }
      }
    }

    std::vector<StepSeed> supported_seeds;
    supported_seeds.reserve(seeds.size());
    for (const auto & seed : seeds) {
      double across_min = 0.0;
      double across_max = 0.0;
      int same_riser_support = 0;
      for (const auto & candidate : seeds) {
        if (seed.dx * candidate.dx + seed.dy * candidate.dy < 0.95) {
          continue;
        }
        const double mx = static_cast<double>(candidate.x - seed.x) * resolution_;
        const double my = static_cast<double>(candidate.y - seed.y) * resolution_;
        const double along = mx * seed.dx + my * seed.dy;
        const double across = -mx * seed.dy + my * seed.dx;
        if (std::abs(along) <= 0.10 && std::abs(across) <= 0.75) {
          across_min = std::min(across_min, across);
          across_max = std::max(across_max, across);
          ++same_riser_support;
        }
      }
      if (same_riser_support >= 4 && across_max - across_min >= stair_min_width_) {
        supported_seeds.push_back(seed);
      }
    }

    const double along_before = 0.15;
    const double along_after = stair_tread_ + 0.20;
    const int radius = static_cast<int>(std::ceil(
      std::max(stair_half_width_, along_after) / resolution_));
    for (const auto & seed : supported_seeds) {
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          const int x = seed.x + dx;
          const int y = seed.y + dy;
          if (x < 0 || x >= width || y < 0 || y >= height) {
            continue;
          }
          const double mx = static_cast<double>(dx) * resolution_;
          const double my = static_cast<double>(dy) * resolution_;
          const double along = mx * seed.dx + my * seed.dy;
          const double across = -mx * seed.dy + my * seed.dx;
          if (along < -along_before || along > along_after ||
            std::abs(across) > stair_half_width_)
          {
            continue;
          }
          const std::size_t index = cell_index(x, y, width);
          terrain[index] = kTerrainStep;
          direction_x[index] += seed.dx;
          direction_y[index] += seed.dy;
        }
      }
    }
    for (std::size_t i = 0; i < terrain.size(); ++i) {
      if (terrain[i] != kTerrainStep) {
        continue;
      }
      const double norm = std::hypot(direction_x[i], direction_y[i]);
      if (norm > 1.0e-9) {
        direction_x[i] /= norm;
        direction_y[i] /= norm;
      }
    }
  }

  void remove_small_terrain_components(
    const int width, const int height, const int8_t label,
    std::vector<int8_t> & terrain, std::vector<double> & direction_x,
    std::vector<double> & direction_y) const
  {
    const int minimum_cells = std::max(1, static_cast<int>(std::ceil(
      terrain_min_component_area_ / (resolution_ * resolution_))));
    std::vector<uint8_t> visited(terrain.size(), 0);
    constexpr std::array<std::pair<int, int>, 8> neighbors {{
      {-1, -1}, {0, -1}, {1, -1}, {-1, 0},
      {1, 0}, {-1, 1}, {0, 1}, {1, 1}}};
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const std::size_t start = cell_index(x, y, width);
        if (visited[start] || terrain[start] != label) {
          continue;
        }
        std::deque<std::pair<int, int>> queue;
        std::vector<std::size_t> component;
        queue.push_back({x, y});
        visited[start] = 1;
        while (!queue.empty()) {
          const auto [cx, cy] = queue.front();
          queue.pop_front();
          component.push_back(cell_index(cx, cy, width));
          for (const auto & [dx, dy] : neighbors) {
            const int nx = cx + dx;
            const int ny = cy + dy;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
              continue;
            }
            const std::size_t next = cell_index(nx, ny, width);
            if (!visited[next] && terrain[next] == label) {
              visited[next] = 1;
              queue.push_back({nx, ny});
            }
          }
        }
        if (static_cast<int>(component.size()) < minimum_cells) {
          for (const std::size_t index : component) {
            terrain[index] = kTerrainFlat;
            direction_x[index] = 0.0;
            direction_y[index] = 0.0;
          }
        }
      }
    }
  }

  void build_grid()
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, cloud) != 0) {
      throw std::runtime_error("读不了 PCD 文件: " + pcd_file_);
    }
    total_points_ = cloud.size();

    if (surface_z_max_ <= surface_z_min_ || z_max_ <= z_min_ ||
      slope_min_deg_ < 0.0 || slope_max_deg_ <= slope_min_deg_ ||
      stair_rise_ <= 0.0 || stair_tread_ <= 0.0 || stair_min_width_ <= 0.0)
    {
      throw std::runtime_error("PCD 栅格或自动地形参数无效");
    }

    const double inf = std::numeric_limits<double>::max();
    double min_x = inf, min_y = inf, max_x = -inf, max_y = -inf;
    std::size_t kept = 0;
    for (const auto & point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z))
      {
        continue;
      }
      if (point.z < surface_z_min_ || point.z > surface_z_max_) {
        continue;
      }
      min_x = std::min(min_x, static_cast<double>(point.x));
      min_y = std::min(min_y, static_cast<double>(point.y));
      max_x = std::max(max_x, static_cast<double>(point.x));
      max_y = std::max(max_y, static_cast<double>(point.y));
      ++kept;
    }
    if (kept == 0) {
      band_points_ = 0;
      throw std::runtime_error("可通行表面高度范围内没有点，检查 surface_z_min/max");
    }
    band_points_ = kept;

    const double origin_x = std::floor(min_x / resolution_) * resolution_;
    const double origin_y = std::floor(min_y / resolution_) * resolution_;
    const double span_x = std::ceil(max_x / resolution_) * resolution_ - origin_x;
    const double span_y = std::ceil(max_y / resolution_) * resolution_ - origin_y;
    const int width = std::max(1, static_cast<int>(std::lround(span_x / resolution_)));
    const int height = std::max(1, static_cast<int>(std::lround(span_y / resolution_)));

    const std::size_t cell_count = static_cast<std::size_t>(width) * height;
    std::vector<float> surface(cell_count, std::numeric_limits<float>::infinity());
    std::vector<int> seen(cell_count, 0);
    for (const auto & point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z))
      {
        continue;
      }
      const int ix = std::clamp(
        static_cast<int>(std::floor((point.x - origin_x) / resolution_)), 0, width - 1);
      const int iy = std::clamp(
        static_cast<int>(std::floor((point.y - origin_y) / resolution_)), 0, height - 1);
      const std::size_t index = static_cast<std::size_t>(iy) * width + ix;
      ++seen[index];
      if (point.z >= surface_z_min_ && point.z <= surface_z_max_) {
        surface[index] = std::min(surface[index], point.z);
      }
    }

    // A 3x3 median suppresses isolated low returns without blurring an entire
    // 0.30 m stair tread. Missing cells remain unknown.
    std::vector<float> filtered_surface = surface;
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        std::array<float, 9> values {};
        int count = 0;
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const int nx = x + dx;
            const int ny = y + dy;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
              continue;
            }
            const float value = surface[static_cast<std::size_t>(ny) * width + nx];
            if (std::isfinite(value)) {
              values[static_cast<std::size_t>(count++)] = value;
            }
          }
        }
        if (count >= 3) {
          std::sort(values.begin(), values.begin() + count);
          filtered_surface[static_cast<std::size_t>(y) * width + x] =
            values[static_cast<std::size_t>(count / 2)];
        }
      }
    }

    std::vector<int8_t> terrain(cell_count, kTerrainFlat);
    std::vector<int8_t> direction(cell_count, kUnknown);
    std::vector<double> direction_x(cell_count, 0.0);
    std::vector<double> direction_y(cell_count, 0.0);
    detect_slopes(
      filtered_surface, width, height, terrain, direction_x, direction_y);
    detect_stairs(
      filtered_surface, width, height, terrain, direction_x, direction_y);
    remove_small_terrain_components(
      width, height, kTerrainSlope, terrain, direction_x, direction_y);
    remove_small_terrain_components(
      width, height, kTerrainStep, terrain, direction_x, direction_y);

    // Occupancy is measured relative to the local walking surface. This keeps
    // platforms, ramps and treads free while walls/objects above them remain
    // obstacles. Directional terrain is explicitly traversable in the static
    // layer; live obstacles are still supplied by the STVL layer.
    std::vector<int> obstacle_counts(cell_count, 0);
    for (const auto & point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z))
      {
        continue;
      }
      const int ix = static_cast<int>(std::floor((point.x - origin_x) / resolution_));
      const int iy = static_cast<int>(std::floor((point.y - origin_y) / resolution_));
      if (ix < 0 || ix >= width || iy < 0 || iy >= height) {
        continue;
      }
      const std::size_t index = static_cast<std::size_t>(iy) * width + ix;
      if (!std::isfinite(filtered_surface[index])) {
        continue;
      }
      const double relative_z = point.z - filtered_surface[index];
      if (relative_z >= z_min_ && relative_z <= z_max_) {
        ++obstacle_counts[index];
      }
    }

    grid_.header.frame_id = frame_id_;
    grid_.header.stamp = now();
    grid_.info.resolution = resolution_;
    grid_.info.width = static_cast<uint32_t>(width);
    grid_.info.height = static_cast<uint32_t>(height);
    grid_.info.origin.position.x = origin_x;
    grid_.info.origin.position.y = origin_y;
    grid_.info.origin.position.z = 0.0;
    grid_.info.origin.orientation.w = 1.0;
    grid_.data.resize(cell_count);
    for (std::size_t i = 0; i < cell_count; ++i) {
      const bool directional = terrain[i] == kTerrainSlope || terrain[i] == kTerrainStep;
      if (!directional && obstacle_counts[i] >= min_points_) {
        grid_.data[i] = kOccupied;
        terrain[i] = kTerrainObstacle;
      } else if (!mark_unknown_ || std::isfinite(filtered_surface[i]) || seen[i] > 0) {
        grid_.data[i] = kFree;
      } else {
        grid_.data[i] = kUnknown;
      }
      if (terrain[i] >= kTerrainSlope) {
        double angle = std::atan2(direction_y[i], direction_x[i]);
        if (angle < 0.0) {
          angle += 2.0 * kPi;
        }
        direction[i] = static_cast<int8_t>(std::clamp(
          static_cast<int>(std::lround(angle * 100.0 / (2.0 * kPi))), 0, 100));
      }
      ++terrain_counts_[static_cast<std::size_t>(terrain[i])];
    }

    terrain_type_grid_ = grid_;
    terrain_type_grid_.data = terrain;
    terrain_direction_grid_ = grid_;
    terrain_direction_grid_.data = direction;

    build_terrain_cloud(terrain, filtered_surface, origin_x, origin_y, width, height);
  }

  // /terrain/type 的 0..3 在 RViz 的 Map 显示里无法区分（map 调色板下 1/2/3 都是
  // 接近白色，costmap 调色板下都是接近蓝色的透明格），因此把坡道/台阶逐格导出一个
  // 带 rgb 的点云：坡道=琥珀色，台阶=洋红。z 取局部行驶表面之上 5 cm，避免和 /map 平面
  // 重叠。障碍格不重复发，/map 已经画出来了。
  void build_terrain_cloud(
    const std::vector<int8_t> & terrain, const std::vector<float> & surface,
    const double origin_x, const double origin_y, const int width, const int height)
  {
    std::vector<std::size_t> cells;
    cells.reserve(terrain_counts_[2] + terrain_counts_[3]);
    for (std::size_t i = 0; i < terrain.size(); ++i) {
      if (terrain[i] == kTerrainSlope || terrain[i] == kTerrainStep) {
        cells.push_back(i);
      }
    }
    terrain_cloud_ = sensor_msgs::msg::PointCloud2 {};
    if (cells.empty()) {
      return;
    }

    terrain_cloud_.header = grid_.header;
    terrain_cloud_.height = 1;
    terrain_cloud_.width = static_cast<uint32_t>(cells.size());
    sensor_msgs::PointCloud2Modifier modifier(terrain_cloud_);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(cells.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(terrain_cloud_, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(terrain_cloud_, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(terrain_cloud_, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(terrain_cloud_, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(terrain_cloud_, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(terrain_cloud_, "b");
    for (const std::size_t index : cells) {
      const int x = static_cast<int>(index % static_cast<std::size_t>(width));
      const int y = static_cast<int>(index / static_cast<std::size_t>(width));
      const bool is_step = terrain[index] == kTerrainStep;
      const double height =
        std::isfinite(surface[index]) ? static_cast<double>(surface[index]) + 0.05 : 0.05;
      *iter_x = static_cast<float>(origin_x + (static_cast<double>(x) + 0.5) * resolution_);
      *iter_y = static_cast<float>(origin_y + (static_cast<double>(y) + 0.5) * resolution_);
      *iter_z = static_cast<float>(height);
      *iter_r = 255;
      *iter_g = is_step ? 0 : 191;
      *iter_b = is_step ? 255 : 0;
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_r;
      ++iter_g;
      ++iter_b;
    }
  }

  void save_map() const
  {
    const int width = static_cast<int>(grid_.info.width);
    const int height = static_cast<int>(grid_.info.height);

    // map_server 约定 0=占据(黑)、254=空闲(白)、205=未知(灰)；图像行序自上而下，
    // 栅格行序自下而上，所以要上下翻转。
    std::vector<uint8_t> image(static_cast<std::size_t>(width) * height, 254);
    for (int row = 0; row < height; ++row) {
      const int image_row = height - 1 - row;
      for (int col = 0; col < width; ++col) {
        const int8_t value = grid_.data[static_cast<std::size_t>(row) * width + col];
        uint8_t & pixel = image[static_cast<std::size_t>(image_row) * width + col];
        if (value == kOccupied) {
          pixel = 0;
        } else if (value < 0) {
          pixel = 205;
        }
      }
    }

    const std::string pgm_path = save_prefix_ + ".pgm";
    std::ofstream pgm(pgm_path, std::ios::binary);
    if (!pgm) {
      RCLCPP_WARN(get_logger(), "写不了 %s", pgm_path.c_str());
      return;
    }
    pgm << "P5\n" << width << " " << height << "\n255\n";
    pgm.write(reinterpret_cast<const char *>(image.data()),
              static_cast<std::streamsize>(image.size()));

    const std::string yaml_path = save_prefix_ + ".yaml";
    std::ofstream yaml(yaml_path);
    if (!yaml) {
      RCLCPP_WARN(get_logger(), "写不了 %s", yaml_path.c_str());
      return;
    }
    const std::string base = pgm_path.substr(pgm_path.find_last_of('/') + 1);
    yaml << "image: " << base << "\n"
         << "resolution: " << grid_.info.resolution << "\n"
         << "origin: [" << grid_.info.origin.position.x << ", "
         << grid_.info.origin.position.y << ", 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.196\n";

    RCLCPP_INFO(get_logger(), "已保存 %s 和 %s", pgm_path.c_str(), yaml_path.c_str());
  }

  std::string pcd_file_;
  double resolution_{0.05};
  double z_min_{0.15};
  double z_max_{1.5};
  int min_points_{1};
  bool mark_unknown_{true};
  std::string frame_id_;
  std::string topic_;
  std::string save_prefix_;
  bool auto_terrain_{true};
  std::string terrain_type_topic_;
  std::string terrain_direction_topic_;
  std::string terrain_cloud_topic_;
  double surface_z_min_{-0.30};
  double surface_z_max_{1.20};
  double slope_min_deg_{5.0};
  double slope_max_deg_{35.0};
  double slope_fit_radius_{0.20};
  double slope_max_residual_{0.020};
  double stair_rise_{0.15};
  double stair_tread_{0.30};
  double stair_rise_tolerance_{0.025};
  double stair_half_width_{0.55};
  double stair_min_width_{0.65};
  double terrain_min_component_area_{0.50};
  std::size_t total_points_{0};
  std::size_t band_points_{0};
  std::array<std::size_t, 4> terrain_counts_ {};

  nav_msgs::msg::OccupancyGrid grid_;
  nav_msgs::msg::OccupancyGrid terrain_type_grid_;
  nav_msgs::msg::OccupancyGrid terrain_direction_grid_;
  sensor_msgs::msg::PointCloud2 terrain_cloud_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr terrain_type_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr terrain_direction_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_cloud_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PcdToOccupancyMap>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("pcd_to_occupancy_map"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
