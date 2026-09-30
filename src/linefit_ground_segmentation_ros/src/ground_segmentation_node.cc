#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "ground_segmentation/ground_segmentation.h"

class SegmentationNode : public rclcpp::Node {
public:
  SegmentationNode(const rclcpp::NodeOptions &node_options);
  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  GroundSegmentationParams params_;
  std::shared_ptr<GroundSegmentation> segmenter_;
  std::string gravity_aligned_frame_;
  bool self_filter_enabled_{true};
  std::string self_filter_frame_{"base_link"};
  Eigen::Vector3d self_filter_min_{-0.35, -0.35, 0.0};
  Eigen::Vector3d self_filter_max_{0.35, 0.35, 1.4};
  // 只把地面以上这个高度带内的非地面点当作障碍物（z=0 为地面，单位 m）。
  double obstacle_min_height_{0.0};
  double obstacle_max_height_{1.0};
  bool restamp_output_{true};
};

SegmentationNode::SegmentationNode(const rclcpp::NodeOptions &node_options)
    : Node("ground_segmentation", node_options) {
  gravity_aligned_frame_ =
      this->declare_parameter("gravity_aligned_frame", "gravity_aligned");

  params_.visualize = this->declare_parameter("visualize", params_.visualize);
  params_.n_bins = this->declare_parameter("n_bins", params_.n_bins);
  params_.n_segments =
      this->declare_parameter("n_segments", params_.n_segments);
  params_.max_dist_to_line =
      this->declare_parameter("max_dist_to_line", params_.max_dist_to_line);
  params_.max_slope = this->declare_parameter("max_slope", params_.max_slope);
  params_.min_slope = this->declare_parameter("min_slope", params_.min_slope);
  params_.long_threshold =
      this->declare_parameter("long_threshold", params_.long_threshold);
  params_.max_long_height =
      this->declare_parameter("max_long_height", params_.max_long_height);
  params_.max_start_height =
      this->declare_parameter("max_start_height", params_.max_start_height);
  params_.sensor_height =
      this->declare_parameter("sensor_height", params_.sensor_height);
  params_.line_search_angle =
      this->declare_parameter("line_search_angle", params_.line_search_angle);
  params_.n_threads = this->declare_parameter("n_threads", params_.n_threads);
  obstacle_min_height_ =
      this->declare_parameter("obstacle_min_height", obstacle_min_height_);
  obstacle_max_height_ =
      this->declare_parameter("obstacle_max_height", obstacle_max_height_);
  restamp_output_ = this->declare_parameter("restamp_output", restamp_output_);
  self_filter_enabled_ =
      this->declare_parameter("self_filter.enabled", self_filter_enabled_);
  self_filter_frame_ =
      this->declare_parameter("self_filter.frame", self_filter_frame_);
  const auto self_filter_min = this->declare_parameter<std::vector<double>>(
      "self_filter.box_min", {-0.35, -0.35, 0.0});
  const auto self_filter_max = this->declare_parameter<std::vector<double>>(
      "self_filter.box_max", {0.35, 0.35, 1.4});
  if (self_filter_min.size() != 3 || self_filter_max.size() != 3) {
    throw std::runtime_error(
        "self_filter.box_min and self_filter.box_max must contain 3 values");
  }
  self_filter_min_ = Eigen::Vector3d(
      self_filter_min[0], self_filter_min[1], self_filter_min[2]);
  self_filter_max_ = Eigen::Vector3d(
      self_filter_max[0], self_filter_max[1], self_filter_max[2]);
  if ((self_filter_min_.array() >= self_filter_max_.array()).any()) {
    throw std::runtime_error(
        "every self_filter.box_min value must be smaller than box_max");
  }
  // These YAML parameters are expressed as linear distances while the
  // segmentation library stores their squared values internally.
  const double r_max = this->declare_parameter(
      "r_max", std::sqrt(params_.r_max_square));
  const double max_fit_error = this->declare_parameter(
      "max_fit_error", std::sqrt(params_.max_error_square));
  params_.r_max_square = r_max * r_max;
  params_.max_error_square = max_fit_error * max_fit_error;
  segmenter_ = std::make_shared<GroundSegmentation>(params_);
  std::string ground_topic, obstacle_topic, input_topic;
  ground_topic = this->declare_parameter("ground_output_topic", "ground_cloud");
  obstacle_topic =
      this->declare_parameter("obstacle_output_topic", "obstacle_cloud");
  input_topic = this->declare_parameter("input_topic", "input_cloud");
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&SegmentationNode::scanCallback, this, std::placeholders::_1));
  // Nav2 costmap requests RELIABLE point-cloud delivery. A RELIABLE publisher
  // remains compatible with Best Effort visualization subscribers as well.
  const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
  ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      ground_topic, output_qos);
  obstacle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      obstacle_topic, output_qos);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  RCLCPP_INFO(
      this->get_logger(),
      "Segmentation initialized: input=%s, obstacle=%s, max_range=%.2fm, "
      "self_filter=%s frame=%s box=[%.2f %.2f %.2f]-[%.2f %.2f %.2f]m, restamp=%s",
      input_topic.c_str(), obstacle_topic.c_str(), r_max,
      self_filter_enabled_ ? "on" : "off", self_filter_frame_.c_str(),
      self_filter_min_.x(), self_filter_min_.y(), self_filter_min_.z(),
      self_filter_max_.x(), self_filter_max_.y(), self_filter_max_.z(),
      restamp_output_ ? "on" : "off");
}

void SegmentationNode::scanCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
  std::vector<double> point_heights;
  filtered_cloud.reserve(cloud.size());
  point_heights.reserve(cloud.size());

  // Transform points to base_link only for the vehicle-box test. Keep the
  // published coordinates in the original LiDAR frame so downstream TF and
  // message headers remain unchanged.
  Eigen::Affine3d base_from_cloud = Eigen::Affine3d::Identity();
  bool have_base_transform = msg->header.frame_id == self_filter_frame_;
  if (!have_base_transform) {
    try {
      const auto tf_stamped = tf_buffer_->lookupTransform(
          self_filter_frame_, msg->header.frame_id, msg->header.stamp);
      base_from_cloud.translation() = Eigen::Vector3d(
          tf_stamped.transform.translation.x,
          tf_stamped.transform.translation.y,
          tf_stamped.transform.translation.z);
      base_from_cloud.linear() = Eigen::Quaterniond(
          tf_stamped.transform.rotation.w,
          tf_stamped.transform.rotation.x,
          tf_stamped.transform.rotation.y,
          tf_stamped.transform.rotation.z).normalized().toRotationMatrix();
      have_base_transform = true;
    } catch (tf2::TransformException &ex) {
      // Fail safe: keep all points as possible obstacles instead of deleting
      // nearby environmental points with an invalid transform.
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Self-filter skipped: cannot transform %s to %s: %s",
          msg->header.frame_id.c_str(), self_filter_frame_.c_str(), ex.what());
    }
  }

  for (const auto &point : cloud) {
    const Eigen::Vector3d point_cloud(point.x, point.y, point.z);
    const Eigen::Vector3d point_base = have_base_transform
        ? base_from_cloud * point_cloud
        : Eigen::Vector3d(
            point.x, point.y, point.z + params_.sensor_height);
    if (self_filter_enabled_ && have_base_transform &&
        (point_base.array() >= self_filter_min_.array()).all() &&
        (point_base.array() <= self_filter_max_.array()).all()) {
      continue;
    }
    filtered_cloud.push_back(point);
    point_heights.push_back(point_base.z());
  }
  cloud.swap(filtered_cloud);
  pcl::PointCloud<pcl::PointXYZ> cloud_transformed;

  std::vector<int> labels;

  bool is_original_pc = true;
  if (!gravity_aligned_frame_.empty()) {
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_->lookupTransform(
          gravity_aligned_frame_, msg->header.frame_id, msg->header.stamp);
      // Remove translation part.
      tf_stamped.transform.translation.x = 0;
      tf_stamped.transform.translation.y = 0;
      tf_stamped.transform.translation.z = 0;
      Eigen::Affine3d tf;
      tf.translate(Eigen::Vector3d(0, 0, 0));
      tf.rotate(Eigen::Quaterniond(
          tf_stamped.transform.rotation.w, tf_stamped.transform.rotation.x,
          tf_stamped.transform.rotation.y, tf_stamped.transform.rotation.z));
      // tf::transformMsgToEigen(tf_stamped.transform, tf);
      pcl::transformPointCloud(cloud, cloud_transformed, tf);
      is_original_pc = false;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to transform point cloud into "
                  "gravity frame: %s",
                  ex.what());
    }
  }

  // Trick to avoid PC copy if we do not transform.
  const pcl::PointCloud<pcl::PointXYZ> &cloud_proc =
      is_original_pc ? cloud : cloud_transformed;

  segmenter_->segment(cloud_proc, &labels);
  pcl::PointCloud<pcl::PointXYZ> ground_cloud, obstacle_cloud;
  for (size_t i = 0; i < cloud.size(); ++i) {
    // Points beyond r_max have label 0 as well, but label 0 means "not ground",
    // not necessarily "valid obstacle". Vehicle returns are handled only by
    // the base_link box above; there is no minimum-radius filter.
    const double range_square =
        cloud_proc[i].x * cloud_proc[i].x +
        cloud_proc[i].y * cloud_proc[i].y;
    if (range_square >= params_.r_max_square) {
      continue;
    }
    if (labels[i] == 1) {
      ground_cloud.push_back(cloud[i]);
      continue;
    }
    // 车体系 z=0 为地面；仅输出指定高度带内的非地面点。
    const double z_above_ground = point_heights[i];
    if (z_above_ground >= obstacle_min_height_ &&
        z_above_ground <= obstacle_max_height_) {
      obstacle_cloud.push_back(cloud[i]);
    }
  }
  auto ground_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  auto obstacle_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(ground_cloud, *ground_msg);
  pcl::toROSMsg(obstacle_cloud, *obstacle_msg);
  ground_msg->header = msg->header;
  obstacle_msg->header = msg->header;
  if (restamp_output_) {
    // The Livox PointCloud2 stamp is not guaranteed to share the dynamic TF
    // cache's time base. Segmentation runs on the newest sensor frame, so use
    // reception/processing time for downstream message filters while keeping
    // the original livox_frame coordinates and the static body self-filter.
    const auto output_stamp = this->now();
    ground_msg->header.stamp = output_stamp;
    obstacle_msg->header.stamp = output_stamp;
  }
  ground_pub_->publish(*ground_msg);
  obstacle_pub_->publish(*obstacle_msg);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<SegmentationNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
