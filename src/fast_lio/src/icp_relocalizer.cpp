// 基于 PCL GICP 的重定位：把实时点云对齐到先验 PCD 地图，输出 fast_lio 的 reloc 位姿。
//
// 流程：
// 1. 读先验 PCD，按高度带过滤 + 体素降采样，作为 ICP 目标点云
// 2. 订阅实时点云，用 TF 变换到 base_frame，同样过滤 + 降采样
// 3. 以当前 odom->base_link 为初值，先多方向粗配准，再进行精配准
// 4. 收敛且通过残差/姿态/修正距离检查后，把 base_frame 在地图里的位姿发到 /reloc/cloud_align
//
// 坐标系约定：fast_lio 的 reloc 接口要的是 T_robot^odom，即机器人在 odom 里的位姿。
// 本节点算出来的是 T_base^map，重定位成功后 odom 会与 map 重合，所以直接按 odom
// 作为 frame_id 发出。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class IcpRelocalizer : public rclcpp::Node
{
public:
  IcpRelocalizer() : Node("icp_relocalizer")
  {
    pcd_file_ = declare_parameter<std::string>("pcd_file", "");
    map_voxel_ = declare_parameter<double>("map_voxel", 0.2);
    source_voxel_ = declare_parameter<double>("source_voxel", 0.2);
    z_min_ = declare_parameter<double>("z_min", -0.5);
    z_max_ = declare_parameter<double>("z_max", 1.5);
    input_topic_ = declare_parameter<std::string>("input_topic", "/cloud_registered_body");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    output_topic_ = declare_parameter<std::string>("output_topic", "/reloc/cloud_align");
    coarse_max_correspondence_distance_ =
      declare_parameter<double>("coarse_max_correspondence_distance", 2.0);
    coarse_iterations_ = declare_parameter<int>("coarse_iterations", 40);
    yaw_search_steps_ = declare_parameter<int>("yaw_search_steps", 4);
    max_correspondence_distance_ =
      declare_parameter<double>("max_correspondence_distance", 1.0);
    max_iterations_ = declare_parameter<int>("max_iterations", 50);
    fitness_threshold_ = declare_parameter<double>("fitness_threshold", 0.5);
    min_source_points_ = declare_parameter<int>("min_source_points", 200);
    // 冷启动保护：启动后这段时间内不匹配，等 fast_lio 出稳定的点云
    startup_delay_ = declare_parameter<double>("startup_delay", 5.0);
    // 地面机器人 roll/pitch 应接近 0，超过这个角度视为误匹配
    max_roll_pitch_deg_ = declare_parameter<double>("max_roll_pitch_deg", 10.0);
    // 相对 ICP 初值的最大净平移（米），超过说明匹配滑走了
    max_correction_dist_ = declare_parameter<double>("max_correction_dist", 2.0);
    // 先攒这么多帧融合成本地子图再匹配一次（单帧点云几何约束不够，容易滑）
    accumulate_frames_ = declare_parameter<int>("accumulate_frames", 30);
    use_odom_as_initial_guess_ =
      declare_parameter<bool>("use_odom_as_initial_guess", true);
    initial_x_ = declare_parameter<double>("initial_x", 0.0);
    initial_y_ = declare_parameter<double>("initial_y", 0.0);
    initial_z_ = declare_parameter<double>("initial_z", 0.0);
    initial_yaw_ = declare_parameter<double>("initial_yaw", 0.0);
    one_shot_ = declare_parameter<bool>("one_shot", true);
    cooldown_ = declare_parameter<double>("cooldown", 3.0);
    publish_static_map_to_odom_ =
      declare_parameter<bool>("publish_static_map_to_odom", true);

    if (pcd_file_.empty()) {
      throw std::runtime_error("参数 pcd_file 未设置，需要指向先验 PCD 地图");
    }

    RCLCPP_INFO(get_logger(), "读取先验地图 %s ...", pcd_file_.c_str());
    PointCloud::Ptr raw(new PointCloud());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, *raw) != 0) {
      throw std::runtime_error("读不了 PCD 文件: " + pcd_file_);
    }
    target_ = filter_and_downsample(raw, map_voxel_);
    if (target_->size() < 50) {
      throw std::runtime_error(
              "先验地图在 z∈[" + std::to_string(z_min_) + ", " +
              std::to_string(z_max_) + "] 内只有 " +
              std::to_string(target_->size()) + " 个点，检查 pcd_file 和高度带");
    }
    RCLCPP_INFO(
      get_logger(), "先验地图 %zu 个点（体素 %.2fm）", target_->size(), map_voxel_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    if (publish_static_map_to_odom_) {
      // 重定位就是把 odom 原点对齐到先验图，所以 map->odom 恒为单位阵。
      // 启动就发，RViz 的 Map 显示和 Nav2 的 static_layer 才不用干等重定位。
      publish_map_to_odom();
      RCLCPP_INFO(
        get_logger(), "已发布静态 map -> odom = 单位阵（重定位后 odom 仍与 map 重合）");
    }

    publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(output_topic_, 10);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&IcpRelocalizer::cloud_callback, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "等待 %s，以当前里程计为初值进行粗到细 GICP，成功后发到 %s",
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  PointCloud::Ptr filter_and_downsample(const PointCloud::Ptr & input, double voxel) const
  {
    PointCloud::Ptr z_filtered(new PointCloud());
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min_, z_max_);
    pass.filter(*z_filtered);

    PointCloud::Ptr downsampled(new PointCloud());
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setInputCloud(z_filtered);
    grid.setLeafSize(
      static_cast<float>(voxel), static_cast<float>(voxel), static_cast<float>(voxel));
    grid.filter(*downsampled);
    return downsampled;
  }

  void publish_map_to_odom()
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = "map";
    tf.child_frame_id = odom_frame_;
    tf.transform.rotation.w = 1.0;
    static_broadcaster_->sendTransform(tf);
  }

  void reset_accumulator()
  {
    accumulated_->clear();
    frame_count_ = 0;
  }

  bool lookup(const std::string & target_frame, const std::string & source_frame,
              Eigen::Matrix4f & out)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        target_frame, source_frame, tf2::TimePointZero);
      Eigen::Quaternionf q(
        static_cast<float>(tf.transform.rotation.w),
        static_cast<float>(tf.transform.rotation.x),
        static_cast<float>(tf.transform.rotation.y),
        static_cast<float>(tf.transform.rotation.z));
      out = Eigen::Matrix4f::Identity();
      out.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
      out.block<3, 1>(0, 3) = Eigen::Vector3f(
        static_cast<float>(tf.transform.translation.x),
        static_cast<float>(tf.transform.translation.y),
        static_cast<float>(tf.transform.translation.z));
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "拿不到 %s <- %s 的 TF: %s",
        target_frame.c_str(), source_frame.c_str(), e.what());
      return false;
    }
  }

  Eigen::Matrix4f initial_guess()
  {
    Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
    guess.block<3, 3>(0, 0) =
      Eigen::AngleAxisf(static_cast<float>(initial_yaw_), Eigen::Vector3f::UnitZ())
      .toRotationMatrix();
    guess.block<3, 1>(0, 3) = Eigen::Vector3f(
      static_cast<float>(initial_x_), static_cast<float>(initial_y_),
      static_cast<float>(initial_z_));

    if (!use_odom_as_initial_guess_) {
      return guess;
    }
    Eigen::Matrix4f odom;
    if (!lookup(odom_frame_, base_frame_, odom)) {
      return guess;
    }
    return odom * guess;
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (done_) {
      return;
    }
    const auto now_time = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now_time - start_time_).count() < startup_delay_) {
      return;
    }
    if (std::chrono::duration<double>(now_time - last_attempt_).count() < cooldown_) {
      return;
    }

    PointCloud::Ptr cloud(new PointCloud());
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) {
      return;
    }

    if (!msg->header.frame_id.empty() && msg->header.frame_id != base_frame_) {
      Eigen::Matrix4f transform;
      if (!lookup(base_frame_, msg->header.frame_id, transform)) {
        return;
      }
      pcl::transformPointCloud(*cloud, *cloud, transform);
    }

    cloud = filter_and_downsample(cloud, source_voxel_);
    if (static_cast<int>(cloud->size()) < min_source_points_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "实时点云只剩 %zu 个点，跳过（阈值 %d）",
        cloud->size(), min_source_points_);
      return;
    }

    // 先攒若干帧融合成本地子图，再匹配一次。单帧点云几何约束不足，容易滑到局部极小。
    if (accumulate_frames_ > 1) {
      *accumulated_ += *cloud;
      ++frame_count_;
      if (frame_count_ < accumulate_frames_) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000, "累积点云 %d/%d 帧（%zu 点，稳住别动）",
          frame_count_, accumulate_frames_, accumulated_->size());
        return;
      }
      cloud = filter_and_downsample(accumulated_, source_voxel_);
      RCLCPP_INFO(
        get_logger(), "已融合 %d 帧，合并后 %zu 点，开始匹配", frame_count_,
        cloud->size());
    }
    if (static_cast<int>(cloud->size()) < min_source_points_) {
      RCLCPP_WARN(get_logger(), "融合后点数仍不足，重置累积");
      reset_accumulator();
      return;
    }

    last_attempt_ = now_time;
    const auto started = std::chrono::steady_clock::now();
    const Eigen::Matrix4f guess = initial_guess();
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> coarse;
    coarse.setInputSource(cloud);
    coarse.setInputTarget(target_);
    coarse.setMaxCorrespondenceDistance(coarse_max_correspondence_distance_);
    coarse.setMaximumIterations(coarse_iterations_);
    coarse.setMaximumOptimizerIterations(20);
    coarse.setCorrespondenceRandomness(20);
    coarse.setTransformationEpsilon(1e-6);
    coarse.setEuclideanFitnessEpsilon(1e-3);

    // 不依赖人工初值：在 odom 初值周围均匀尝试多个航向，选粗配准残差最小者。
    Eigen::Matrix4f coarse_transform = guess;
    double coarse_score = std::numeric_limits<double>::infinity();
    bool coarse_converged = false;
    const int yaw_steps = std::max(1, yaw_search_steps_);
    for (int i = 0; i < yaw_steps; ++i) {
      const float yaw = static_cast<float>(2.0 * M_PI * i / yaw_steps);
      Eigen::Matrix4f candidate = guess;
      candidate.block<3, 3>(0, 0) =
        Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix() *
        guess.block<3, 3>(0, 0);
      PointCloud coarse_aligned;
      coarse.align(coarse_aligned, candidate);
      if (!coarse.hasConverged()) {
        continue;
      }
      const double score = coarse.getFitnessScore(coarse_max_correspondence_distance_);
      if (std::isfinite(score) && score < coarse_score) {
        coarse_score = score;
        coarse_transform = coarse.getFinalTransformation();
        coarse_converged = true;
      }
    }
    if (!coarse_converged) {
      RCLCPP_WARN(get_logger(), "GICP 粗配准未收敛，重置累积后重试");
      reset_accumulator();
      return;
    }

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> fine;
    fine.setInputSource(cloud);
    fine.setInputTarget(target_);
    fine.setMaxCorrespondenceDistance(max_correspondence_distance_);
    fine.setMaximumIterations(max_iterations_);
    fine.setMaximumOptimizerIterations(20);
    fine.setCorrespondenceRandomness(20);
    fine.setTransformationEpsilon(1e-8);
    fine.setEuclideanFitnessEpsilon(1e-4);

    PointCloud aligned;
    fine.align(aligned, coarse_transform);
    if (!fine.hasConverged()) {
      RCLCPP_WARN(get_logger(), "GICP 精配准未收敛，重置累积后重试");
      reset_accumulator();
      return;
    }

    // getFitnessScore 返回均方误差，开方换成米。
    const double rmse = std::sqrt(fine.getFitnessScore(max_correspondence_distance_));
    if (rmse > fitness_threshold_) {
      RCLCPP_WARN(
        get_logger(), "ICP 残差 %.3fm 超过阈值 %.3fm，重置累积后重试", rmse,
        fitness_threshold_);
      reset_accumulator();
      return;
    }

    const Eigen::Matrix4f transform = fine.getFinalTransformation();

    // 地面机器人的 roll/pitch 应该接近 0；歪着的匹配直接拒掉。
    const Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
    const double pitch = std::asin(
      std::clamp(-static_cast<double>(rotation(2, 0)), -1.0, 1.0));
    const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
    const double tilt_deg = std::max(std::abs(roll), std::abs(pitch)) * 180.0 / M_PI;
    if (tilt_deg > max_roll_pitch_deg_) {
      RCLCPP_WARN(
        get_logger(), "ICP 结果 roll/pitch=%.1f° 超过 %.1f°，疑似误匹配，重置累积后重试",
        tilt_deg, max_roll_pitch_deg_);
      reset_accumulator();
      return;
    }

    // 相对初值的净平移；跑太远说明匹配滑走了。
    const Eigen::Matrix4f correction = transform * guess.inverse();
    const double correction_dist = correction.block<3, 1>(0, 3).norm();
    if (correction_dist > max_correction_dist_) {
      RCLCPP_WARN(
        get_logger(), "ICP 相对初值移动了 %.2fm（上限 %.2fm），疑似误匹配，重置累积后重试",
        correction_dist, max_correction_dist_);
      reset_accumulator();
      return;
    }

    const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    publish_pose(msg, transform, rmse, elapsed);
  }

  void publish_pose(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                    const Eigen::Matrix4f & transform, double rmse, double elapsed)
  {
    const Eigen::Vector3f t = transform.block<3, 1>(0, 3);
    Eigen::Quaternionf q(transform.block<3, 3>(0, 0));
    q.normalize();

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = odom_frame_;
    pose.header.stamp = msg->header.stamp;
    pose.pose.position.x = t.x();
    pose.pose.position.y = t.y();
    pose.pose.position.z = t.z();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    publisher_->publish(pose);

    const double yaw = std::atan2(transform(1, 0), transform(0, 0)) * 180.0 / M_PI;
    RCLCPP_INFO(
      get_logger(),
      "重定位成功：xyz=(%.3f, %.3f, %.3f) yaw=%.1f° | 残差 %.3fm | 用时 %.2fs | 已发到 %s",
      t.x(), t.y(), t.z(), yaw, rmse, elapsed, output_topic_.c_str());

    if (publish_static_map_to_odom_) {
      publish_map_to_odom();
    }

    if (one_shot_) {
      done_ = true;
      RCLCPP_INFO(get_logger(), "one_shot=true，停止后续匹配");
    }
  }

  std::string pcd_file_;
  double map_voxel_{0.2};
  double source_voxel_{0.2};
  double z_min_{-0.5};
  double z_max_{1.5};
  std::string input_topic_;
  std::string base_frame_;
  std::string odom_frame_;
  std::string output_topic_;
  double coarse_max_correspondence_distance_{2.0};
  int coarse_iterations_{40};
  int yaw_search_steps_{4};
  double max_correspondence_distance_{1.0};
  int max_iterations_{50};
  double fitness_threshold_{0.5};
  int min_source_points_{200};
  int accumulate_frames_{30};
  PointCloud::Ptr accumulated_{new PointCloud()};
  int frame_count_{0};
  double startup_delay_{5.0};
  double max_roll_pitch_deg_{10.0};
  double max_correction_dist_{2.0};
  bool use_odom_as_initial_guess_{true};
  double initial_x_{0.0};
  double initial_y_{0.0};
  double initial_z_{0.0};
  double initial_yaw_{0.0};
  bool one_shot_{true};
  double cooldown_{3.0};
  bool publish_static_map_to_odom_{true};

  PointCloud::Ptr target_;
  std::chrono::steady_clock::time_point last_attempt_{};
  std::chrono::steady_clock::time_point start_time_{std::chrono::steady_clock::now()};
  bool done_{false};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<IcpRelocalizer>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("icp_relocalizer"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
