#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "ros_utils.h"

using PointType = pcl::PointXYZI;

bool enabled = true;
bool fill_free_space = true;
int min_points_per_cell = 1;
double resolution = 0.1;
double padding = 5.0;
double obstacle_min_height = -0.3;
double obstacle_max_height = 1.5;
std::string input_topic = "lio_sam/mapping/cloud_global_2d";
std::string output_topic = "/map";
std::string map_frame = "map";

OccupancyGridPublisher pubOccupancyGrid;

template<typename StampT>
void publishOccupancyFromHits(
    const StampT &stamp,
    const std::vector<std::pair<double, double>> &hits)
{
    if (hits.empty()) {
        ROS_PRINT_WARN("cloud_to_occupancy: no valid hits remain after filtering");
        return;
    }

    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();

    for (const auto &hit : hits) {
        min_x = std::min(min_x, hit.first);
        min_y = std::min(min_y, hit.second);
        max_x = std::max(max_x, hit.first);
        max_y = std::max(max_y, hit.second);
    }

    min_x -= padding;
    min_y -= padding;
    max_x += padding;
    max_y += padding;

    const auto width = static_cast<std::uint32_t>(
        std::max(1.0, std::ceil((max_x - min_x) / resolution)));
    const auto height = static_cast<std::uint32_t>(
        std::max(1.0, std::ceil((max_y - min_y) / resolution)));

    std::vector<int> hit_count(width * height, 0);
    for (const auto &hit : hits) {
        const int cell_x = static_cast<int>(std::floor((hit.first - min_x) / resolution));
        const int cell_y = static_cast<int>(std::floor((hit.second - min_y) / resolution));
        if (cell_x < 0 || cell_y < 0 ||
            cell_x >= static_cast<int>(width) || cell_y >= static_cast<int>(height)) {
            continue;
        }

        const std::size_t index =
            static_cast<std::size_t>(cell_y) * width + static_cast<std::size_t>(cell_x);
        hit_count[index] += 1;
    }

    OccupancyGridMsg grid;
    grid.header.stamp = stamp;
    grid.header.frame_id = map_frame;
    grid.info.map_load_time = stamp;
    grid.info.resolution = static_cast<float>(resolution);
    grid.info.width = width;
    grid.info.height = height;
    grid.info.origin.position.x = min_x;
    grid.info.origin.position.y = min_y;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(width * height, fill_free_space ? 0 : -1);

    for (std::size_t idx = 0; idx < hit_count.size(); ++idx) {
        if (hit_count[idx] >= min_points_per_cell) {
            grid.data[idx] = 100;
        }
    }

    ros_publish(pubOccupancyGrid, grid);
}

void cloudCallback(const Pcl2MsgConstPtr& msg)
{
    pcl::PointCloud<PointType> cloud;
    pcl::fromROSMsg(*msg, cloud);
    std::vector<std::pair<double, double>> hits;
    hits.reserve(cloud.points.size());

    for (const auto& point : cloud.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }
        if (point.z < obstacle_min_height || point.z > obstacle_max_height) {
            continue;
        }
        hits.emplace_back(static_cast<double>(point.x), static_cast<double>(point.y));
    }

    publishOccupancyFromHits(msg->header.stamp, hits);
}

void readParams()
{
    rosparam_get("occupancy_map/enabled", enabled, true);
    rosparam_get("occupancy_map/input_topic", input_topic, std::string("lio_sam/mapping/cloud_global_2d"));
    rosparam_get("occupancy_map/output_topic", output_topic, std::string("/map"));
    rosparam_get("occupancy_map/resolution", resolution, 0.10);
    rosparam_get("occupancy_map/padding", padding, 5.0);
    rosparam_get("occupancy_map/obstacle_min_height", obstacle_min_height, -0.30);
    rosparam_get("occupancy_map/obstacle_max_height", obstacle_max_height, 1.50);
    rosparam_get("occupancy_map/min_points_per_cell", min_points_per_cell, 1);
    rosparam_get("occupancy_map/fill_free_space", fill_free_space, true);
    rosparam_get("occupancy_map/map_frame", map_frame, std::string("map"));
}

int main(int argc, char** argv)
{
#ifdef USE_ROS1
    ros::init(argc, argv, "cloud_to_occupancy");
    init_ros_node();
#elif defined(USE_ROS2)
    rclcpp::init(argc, argv);
    init_ros_node(rclcpp::Node::make_shared("cloud_to_occupancy"));
#endif

    readParams();

#ifdef USE_ROS1
    pubOccupancyGrid = create_publisher<OccupancyGridMsg>(output_topic, 1);
#elif defined(USE_ROS2)
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    pubOccupancyGrid = create_publisher_qos<OccupancyGridMsg>(output_topic, map_qos);
#endif

    if (!enabled) {
        ROS_PRINT_INFO("cloud_to_occupancy disabled");
    } else {
        auto subCloud = create_subscriber<PointCloud2Msg>(input_topic, 1, cloudCallback);
        ROS_PRINT_INFO(
            "cloud_to_occupancy subscribed to pointcloud %s, publishing %s in frame %s",
            input_topic.c_str(), output_topic.c_str(), map_frame.c_str());

        RateType rate(100.0);
        while (ros_ok()) {
            spin_once();
            rate.sleep();
        }
    }

    ros_shutdown();
    return 0;
}
