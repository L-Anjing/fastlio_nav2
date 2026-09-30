#include <omp.h>
#include <algorithm>
#include <condition_variable>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <so3_math.h>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include "preprocess.h"
#include "ikd-Tree/ikdtree_public.h"
#include <reloc.h>
#include <atomic>
#include <array>
#include <chrono>
#include <set>
#include "posebuffer.h"
#include "ros2_utils.h"
#include "common_utils.h"

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = false, path_en = true;
/**************************/

bool feature_pub_en = false, effect_pub_en = false;

std::vector<float> res_last;
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
mutex mtx_odom_output;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string lid_topic, imu_topic;
string reloc_topic;
string odom_frame = "odom";
string robot_frame = "base_link";
string robot_hf_frame = "base_link_hf";
string imu_frame = "imu_link";
// 默认与 Livox 驱动消息的 frame_id 一致；实际取值由 YAML 的 frames/lidar 覆盖。
string lidar_frame = "livox_frame";
double odom_log_interval_sec = 1.0;

double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
std::vector<char> point_selected_surf;
bool   lidar_pushed, flg_first_scan = true, flg_EKF_inited;
std::atomic<bool> flg_exit(false);
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool reloc_en = false;
bool zero_odom_at_start = true;
std::atomic<bool> odom_origin_ready(false);
int lidar_type;
std::string pcd_output_dir() {
    return root_dir + "/PCD/";
}

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       param_t_imu_in_lidar = {0.011, 0.02329, -0.04412};
vector<double>       param_R_imu_in_lidar = {1.0, 0.0, 0.0,
                                             0.0, 1.0, 0.0,
                                             0.0, 0.0, 1.0};
vector<double>       param_t_lidar_in_robot(3, 0.0);
vector<double>       param_R_lidar_in_robot = {1.0, 0.0, 0.0,
                                                0.0, 1.0, 0.0,
                                                0.0, 0.0, 1.0};
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<ImuMsgConstPtr> imu_buffer;

mutex mtx_reloc;
RelocState reloc_state;
std::atomic<bool> relocalize_flag(false);

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE_PUBLIC<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D t_imu_in_lidar(Zero3d);
M3D R_imu_in_lidar(Eye3d);
// ! Fixed T_lidar^imu derived from the configured T_imu^lidar.
// ! Kept outside the EKF state for high-frequency IMU pose conversion.
V3D t_lidar_in_imu_fixed(Zero3d);
M3D R_lidar_in_imu_fixed(Eye3d);
V3D t_lidar_in_robot(Zero3d);
M3D R_lidar_in_robot(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

PathMsg path;
OdomMsg odomAftMapped;
QuaternionMsg geoQuat;
PoseStampedMsg msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

// ------------------------------------------------------------------
// High-frequency odometry: continuous IMU-rate prediction stream.
//
// The main thread refreshes hf_base_ with the corrected EKF state at each
// scan end (and at rebase / relocalization); the publish thread then
// re-predicts forward from that base using every IMU sample that arrives
// afterwards, with the same kinematics as the EKF process model. This makes
// /OdometryHighFreq advance at the true IMU rate (uniform 200 Hz output)
// instead of only updating once per LiDAR scan.
// ------------------------------------------------------------------
struct ImuRawSample
{
    double timestamp = 0.0;   // synced IMU time [s]
    V3D    gyro = Zero3d;     // raw angular velocity [rad/s]
    V3D    acc  = Zero3d;     // raw linear acceleration [m/s^2]
};

struct HighFreqBase
{
    std::mutex mtx;
    bool      valid = false;         // set once the first corrected base exists
    uint64_t  epoch = 0;             // bumped on every base refresh
    double    timestamp = 0.0;       // time of the stored state (scan end)
    V3D       pos = Zero3d;          // T_imu^odom translation
    M3D       rot = Eye3d;           // T_imu^odom rotation
    V3D       vel = Zero3d;
    V3D       grav = Zero3d;         // world-frame gravity (S2)
    V3D       bg = Zero3d;           // gyro bias
    V3D       ba = Zero3d;           // accel bias
    double    acc_scale = 1.0;       // G_m_s2 / |mean_acc|

    // Retained raw IMU samples (oldest dropped at capacity). The publish
    // thread re-integrates every sample newer than the base timestamp, so the
    // ring must survive from one scan end to the next.
    std::deque<ImuRawSample> imu_samples;
};
HighFreqBase hf_base;

void hf_update_base(const state_ikfom &s, double timestamp, double acc_scale)
{
    std::lock_guard<std::mutex> lock(hf_base.mtx);
    hf_base.valid = true;
    hf_base.epoch++;
    hf_base.timestamp = timestamp;
    hf_base.pos  = V3D(s.pos(0), s.pos(1), s.pos(2));
    hf_base.rot  = s.rot.toRotationMatrix();
    hf_base.vel  = V3D(s.vel(0), s.vel(1), s.vel(2));
    hf_base.grav = V3D(s.grav[0], s.grav[1], s.grav[2]);
    hf_base.bg   = V3D(s.bg(0), s.bg(1), s.bg(2));
    hf_base.ba   = V3D(s.ba(0), s.ba(1), s.ba(2));
    hf_base.acc_scale = acc_scale;
    // IMU samples already covered by the corrected state are dropped; the
    // publish thread re-predicts forward from this base.
    while (!hf_base.imu_samples.empty() &&
           hf_base.imu_samples.front().timestamp <= timestamp) {
        hf_base.imu_samples.pop_front();
    }
}

void SigHandle(int sig)
{
    flg_exit = true;
    ROS_PRINT_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

struct RobotPose
{
    V3D position;
    Eigen::Quaterniond rotation;
};

RobotPose robot_pose_from_imu(
    const V3D &imu_position,
    const Eigen::Quaterniond &imu_rotation,
    const V3D &t_lidar_in_imu_used,
    const M3D &R_lidar_in_imu_used)
{
    // ! Convention: T_B^A maps p^B to p^A. The EKF pose is T_imu^odom.
    // ! T_robot^odom = T_imu^odom * T_lidar^imu * T_robot^lidar,
    // ! where T_robot^lidar = inverse(T_lidar^robot).
    const M3D R_robot_in_imu =
        R_lidar_in_imu_used * R_lidar_in_robot.transpose();
    const V3D t_robot_in_imu =
        t_lidar_in_imu_used - R_robot_in_imu * t_lidar_in_robot;

    RobotPose pose;
    pose.position = imu_position + imu_rotation * t_robot_in_imu;
    pose.rotation = imu_rotation * Eigen::Quaterniond(R_robot_in_imu);
    pose.rotation.normalize();
    return pose;
}

RobotPose imu_pose_from_robot(const RobotPose &robot_pose)
{
    // T_imu^odom = T_robot^odom * T_lidar^robot * T_imu^lidar.
    const M3D R_imu_in_robot = R_lidar_in_robot * R_imu_in_lidar;
    const V3D t_imu_in_robot =
        t_lidar_in_robot + R_lidar_in_robot * t_imu_in_lidar;

    RobotPose pose;
    pose.position = robot_pose.position + robot_pose.rotation * t_imu_in_robot;
    pose.rotation = robot_pose.rotation * Eigen::Quaterniond(R_imu_in_robot);
    pose.rotation.normalize();
    return pose;
}

bool initialize_robot_centered_odom_origin(double timestamp)
{
    if (!zero_odom_at_start || odom_origin_ready.load(std::memory_order_acquire)) {
        return false;
    }

    state_ikfom rebased_state = kf.get_x();
    const RobotPose initial_robot_pose = robot_pose_from_imu(
        V3D(rebased_state.pos(0), rebased_state.pos(1), rebased_state.pos(2)),
        Eigen::Quaterniond(rebased_state.rot.toRotationMatrix()),
        V3D(rebased_state.t_lidar_in_imu(0), rebased_state.t_lidar_in_imu(1),
            rebased_state.t_lidar_in_imu(2)),
        rebased_state.R_lidar_in_imu.toRotationMatrix());

    /*
    ! T_old_odom^new_odom = inverse(T_robot_start^old_odom). Applying it to
    ! the EKF world frame makes T_robot_start^new_odom exactly identity.
    */ 
    const M3D rotation_new_from_old =
        initial_robot_pose.rotation.toRotationMatrix().transpose();
    rebased_state.pos = rotation_new_from_old *
        (V3D(rebased_state.pos(0), rebased_state.pos(1), rebased_state.pos(2)) -
         initial_robot_pose.position);
    rebased_state.rot = Eigen::Quaterniond(rotation_new_from_old) *
        Eigen::Quaterniond(rebased_state.rot.toRotationMatrix());
    rebased_state.vel = rotation_new_from_old *
        V3D(rebased_state.vel(0), rebased_state.vel(1), rebased_state.vel(2));
    rebased_state.grav = S2(rotation_new_from_old *
        V3D(rebased_state.grav[0], rebased_state.grav[1], rebased_state.grav[2]));

    auto covariance = kf.get_P();
    Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF> rebase_jacobian =
        Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF>::Identity();
    rebase_jacobian.block<3, 3>(0, 0) = rotation_new_from_old;
    rebase_jacobian.block<3, 3>(12, 12) = rotation_new_from_old;
    covariance = rebase_jacobian * covariance * rebase_jacobian.transpose();

    kf.change_x(rebased_state);
    kf.change_P(covariance);
    state_point = rebased_state;
    p_imu->RebaseWorldFrame(rotation_new_from_old);

    // Re-seed the high-frequency stream in the new world frame. The publish
    // thread re-predicts forward from this exact zero pose.
    hf_update_base(rebased_state, timestamp, p_imu->get_acc_scale());
    odom_origin_ready.store(true, std::memory_order_release);

    ROS_PRINT_INFO(
        "odom origin initialized at robot center; removed initial xyz=(%.3f, %.3f, %.3f) m",
        initial_robot_pose.position.x(), initial_robot_pose.position.y(),
        initial_robot_pose.position.z());
    return true;
}

bool is_proper_rotation_matrix(const vector<double> &values)
{
    if (values.size() != 9) {
        return false;
    }

    M3D rotation;
    rotation << values[0], values[1], values[2],
                values[3], values[4], values[5],
                values[6], values[7], values[8];
    return rotation.allFinite() &&
           (rotation.transpose() * rotation - M3D(Eye3d)).norm() < 1e-3 &&
           fabs(rotation.determinant() - 1.0) < 1e-3;
}

bool is_finite_vector(const vector<double> &values)
{
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
    });
}

void publish_static_sensor_transforms()
{
    auto make_transform = [](const std::string &parent, const std::string &child,
                             const V3D &translation, const M3D &rotation) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = get_ros_now();
        transform.header.frame_id = parent;
        transform.child_frame_id = child;
        transform.transform.translation.x = translation.x();
        transform.transform.translation.y = translation.y();
        transform.transform.translation.z = translation.z();
        const Eigen::Quaterniond q(rotation);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        return transform;
    };

    static auto static_broadcaster =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(get_ros_node());
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.push_back(
        make_transform(robot_frame, lidar_frame, t_lidar_in_robot, R_lidar_in_robot));
    transforms.push_back(
        make_transform(lidar_frame, imu_frame, t_imu_in_lidar, R_imu_in_lidar));
    static_broadcaster->sendTransform(transforms);
}

inline void dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.R_lidar_in_imu * p_body + s.t_lidar_in_imu) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.R_lidar_in_imu * p_body + state_point.t_lidar_in_imu) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.R_lidar_in_imu * p_body + state_point.t_lidar_in_imu) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.R_lidar_in_imu * p_body + state_point.t_lidar_in_imu) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.R_lidar_in_imu * p_body_lidar + state_point.t_lidar_in_imu);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const Pcl2MsgConstPtr &msg) 
{
    mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    const double stamp_sec = get_ros_time_sec(msg->header.stamp);
    if (stamp_sec < last_timestamp_lidar)
    {
        ROS_PRINT_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(stamp_sec);
    last_timestamp_lidar = stamp_sec;
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void livox_pcl_cbk(const LivoxCustomMsgConstPtr &msg) 
{
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    const double stamp_sec = get_ros_time_sec(msg->header.stamp);
    if (stamp_sec < last_timestamp_lidar)
    {
        ROS_PRINT_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = stamp_sec;
    
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const ImuMsgConstPtr &msg_in) 
{   
    publish_count ++;
    // cout<<"IMU got at: "<<get_ros_time_sec(msg_in->header.stamp)<<endl;
    ImuMsgPtr msg(new ImuMsg(*msg_in));


    const double msg_in_stamp_sec = get_ros_time_sec(msg_in->header.stamp);
    msg->header.stamp = get_ros_time(msg_in_stamp_sec - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = get_ros_time(timediff_lidar_wrt_imu + msg_in_stamp_sec);
    }
    double timestamp = get_ros_time_sec(msg->header.stamp);

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        ROS_PRINT_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();

    // Feed the high-frequency integrator with raw samples. Only the
    // timestamped ring is written here; the propagation itself runs in the
    // publish thread, so this stays cheap even when callbacks are batched.
    {
        std::lock_guard<std::mutex> lock(hf_base.mtx);
        if (!hf_base.imu_samples.empty() &&
            timestamp < hf_base.imu_samples.back().timestamp) {
            hf_base.imu_samples.clear();   // time loopback
        }
        ImuRawSample sample;
        sample.timestamp = timestamp;
        sample.gyro << msg->angular_velocity.x,
                       msg->angular_velocity.y,
                       msg->angular_velocity.z;
        sample.acc  << msg->linear_acceleration.x,
                       msg->linear_acceleration.y,
                       msg->linear_acceleration.z;
        hf_base.imu_samples.push_back(sample);
        while (hf_base.imu_samples.size() > 256) {
            hf_base.imu_samples.pop_front();
        }
    }
    sig_buffer.notify_all();
}

/*** relocation callback ***/
void reloc_cbk(const PoseStampedMsgConstPtr &msg_in) 
{
    if (!msg_in->header.frame_id.empty() && msg_in->header.frame_id != odom_frame) {
        ROS_PRINT_WARN(
            "Ignoring reloc pose in frame '%s'; expected '%s'.",
            msg_in->header.frame_id.c_str(), odom_frame.c_str());
        return;
    }

    double timestamp = get_ros_time_sec(msg_in->header.stamp);
    if (timestamp <= 0.0) {
        timestamp = get_ros_now().seconds();
    }
    double x = msg_in->pose.position.x;
    double y = msg_in->pose.position.y;
    double z = msg_in->pose.position.z;

    double qx = msg_in->pose.orientation.x;
    double qy = msg_in->pose.orientation.y;
    double qz = msg_in->pose.orientation.z;
    double qw = msg_in->pose.orientation.w;

    Eigen::Quaterniond orientation(qw, qx, qy, qz);
    if (!orientation.coeffs().allFinite() || orientation.squaredNorm() < 1e-12 ||
        !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        ROS_PRINT_WARN("Ignoring reloc pose with non-finite position or invalid quaternion.");
        return;
    }
    orientation.normalize();
    qx = orientation.x();
    qy = orientation.y();
    qz = orientation.z();
    qw = orientation.w();
    
    std::lock_guard<std::mutex> lock(mtx_reloc);
    reloc_state = RelocState(x, y, z,
                    qx, qy, qz, qw, timestamp);
    relocalize_flag.store(true); 
    ROS_PRINT_INFO("Reloc received: (%.3f, %.3f, %.3f), quat=(%.3f, %.3f, %.3f, %.3f)",
        x, y, z, qx, qy, qz, qw);
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();


        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            ROS_PRINT_WARN("Too few input point cloud!");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }
        if(lidar_type == MARSIM)
            lidar_end_time = meas.lidar_beg_time;

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = get_ros_time_sec(imu_buffer.front()->header.stamp);
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = get_ros_time_sec(imu_buffer.front()->header.stamp);
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());

void publish_frame_world(const Pcl2Publisher & pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }
        Pcl2Msg laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = odom_frame;
        ros_publish(pubLaserCloudFull, laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            const std::string all_points_dir =
                pcd_output_dir() + "scans_" + std::to_string(pcd_index) + ".pcd";
            pcl::PCDWriter pcd_writer;
            const int save_result = pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            if (save_result != 0) {
                ROS_PRINT_ERROR("failed to save PCD chunk: %s", all_points_dir.c_str());
            } else {
                ROS_PRINT_INFO("saved PCD chunk: %s (%zu points)",
                               all_points_dir.c_str(), pcl_wait_save->size());
                pcl_wait_save->clear();
                scan_wait_num = 0;
            }
        }
    }
}

void publish_frame_body(const Pcl2Publisher & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    Pcl2Msg laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = imu_frame;
    ros_publish(pubLaserCloudFull_body, laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(const Pcl2Publisher & pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    Pcl2Msg laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = get_ros_time(lidar_end_time);
    laserCloudFullRes3.header.frame_id = odom_frame;
    ros_publish(pubLaserCloudEffect, laserCloudFullRes3);
}

void publish_map(const Pcl2Publisher & pubLaserCloudMap)
{
    Pcl2Msg laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = get_ros_time(lidar_end_time);
    laserCloudMap.header.frame_id = odom_frame;
    ros_publish(pubLaserCloudMap, laserCloudMap);
}

void publish_odometryhighfreq(const OdomPublisher& pubOdomHighFreq)
{
    static auto br_hf = std::make_shared<tf2_ros::TransformBroadcaster>(get_ros_node());

    // Fixed 200 Hz cadence: one pose per tick, re-predicted through every
    // IMU sample that arrived since the last scan-end correction.
    const std::chrono::milliseconds period(5);
    auto next_tick = std::chrono::steady_clock::now();

    // Local integrator state, re-seeded from hf_base whenever the corrected
    // base is refreshed (scan end / rebase / relocalization).
    bool   have_state = false;
    uint64_t my_epoch = 0;
    double state_ts = 0.0;
    V3D pos = Zero3d, vel = Zero3d, grav = Zero3d, bg = Zero3d, ba = Zero3d;
    M3D rot = Eye3d;
    double acc_scale = 1.0;

    // Newest integrated IMU input, kept for the per-tick "fill" step so every
    // tick emits a strictly newer timestamp instead of repeating the last
    // pose, plus a self-adapting estimate of the IMU sample period.
    bool   have_last_imu = false;
    V3D    last_gyro = Zero3d, last_acc = Zero3d;
    double period_ema = 0.005;        // typical IMU sample period [s]
    double prev_sample_ts = -1.0;
    int    ticks_since_integration = 0;
    bool   integrated_this_tick = false;
    double last_published_ts = -1.0;

    while (ros_ok() && !flg_exit)
    {
        next_tick += period;
        std::this_thread::sleep_until(next_tick);
        integrated_this_tick = false;
        ticks_since_integration++;

        std::vector<ImuRawSample> samples;
        {
            std::lock_guard<std::mutex> lock(hf_base.mtx);
            if (!hf_base.valid) {
                continue;
            }
            if (!have_state || hf_base.epoch != my_epoch) {
                pos       = hf_base.pos;
                rot       = hf_base.rot;
                vel       = hf_base.vel;
                grav      = hf_base.grav;
                bg        = hf_base.bg;
                ba        = hf_base.ba;
                acc_scale = hf_base.acc_scale;
                state_ts  = hf_base.timestamp;
                my_epoch  = hf_base.epoch;
                have_state = true;
            }
            for (const auto &s : hf_base.imu_samples) {
                if (s.timestamp > state_ts) {
                    samples.push_back(s);
                }
            }
        }
        if (!have_state) {
            continue;
        }

        // Forward propagation with the same kinematics as the EKF process
        // model (get_f in use-ikfom.hpp) plus the accelerometer G-scaling
        // applied in IMU_Processing.hpp:413. Bias states are constant
        // between corrections, exactly as in the EKF.
        for (const auto &s : samples)
        {
            const double dt = s.timestamp - state_ts;
            if (dt <= 0.0) continue;
            const V3D gyro_c  = s.gyro - bg;
            const V3D acc_norm = s.acc * acc_scale;
            const V3D acc_w   = rot * (acc_norm - ba);
            pos += vel * dt + 0.5 * (acc_w + grav) * dt * dt;
            vel += (acc_w + grav) * dt;
            rot  = rot * Exp(gyro_c, dt);
            state_ts = s.timestamp;
            last_gyro = s.gyro;
            last_acc  = s.acc;
            have_last_imu = true;
            integrated_this_tick = true;
            ticks_since_integration = 0;
            if (prev_sample_ts > 0.0) {
                const double pdt = s.timestamp - prev_sample_ts;
                if (pdt > 0.002 && pdt < 0.02) {
                    period_ema = 0.9 * period_ema + 0.1 * pdt;
                }
            }
            prev_sample_ts = s.timestamp;
        }

        // If this tick integrated no fresh IMU sample (tick/IMU phase
        // aliasing), advance the pose by a short constant-input step so
        // every tick still emits a strictly newer timestamp. The step stays
        // below the expected sample period, so the next real sample is never
        // overtaken and integrates normally. If no fresh sample has been
        // integrated for a while (IMU outage), the fill stops and the pose
        // holds instead of drifting.
        if (!integrated_this_tick && have_last_imu &&
            ticks_since_integration <= 40)
        {
            double dt_fill = 0.8 * period_ema;
            if (dt_fill > 0.0045) dt_fill = 0.0045;
            const V3D acc_norm = last_acc * acc_scale;
            const V3D acc_w   = rot * (acc_norm - ba);
            const V3D gyro_c  = last_gyro - bg;
            pos += vel * dt_fill + 0.5 * (acc_w + grav) * dt_fill * dt_fill;
            vel += (acc_w + grav) * dt_fill;
            rot  = rot * Exp(gyro_c, dt_fill);
            state_ts += dt_fill;
        }

        // Keep the stream strictly monotonic. At a scan correction the base
        // is re-seeded from the (slightly older) scan-end state, so the
        // re-predicted pose can land behind the last published timestamp.
        // Clamp the timestamp forward by one normal sample period (advancing
        // the pose with the newest IMU input) so downstream consumers never
        // see repeated or backward timestamps, and the mm-level correction
        // jump is spread over a normal step instead of producing a
        // velocity spike.
        if (last_published_ts > 0.0 && state_ts <= last_published_ts) {
            const double dt_bump = last_published_ts + period_ema - state_ts;
            if (have_last_imu && dt_bump > 0.0) {
                const V3D acc_norm = last_acc * acc_scale;
                const V3D acc_w   = rot * (acc_norm - ba);
                const V3D gyro_c  = last_gyro - bg;
                pos += vel * dt_bump + 0.5 * (acc_w + grav) * dt_bump * dt_bump;
                vel += (acc_w + grav) * dt_bump;
                rot  = rot * Exp(gyro_c, dt_bump);
            }
            state_ts = last_published_ts + period_ema;
        }
        last_published_ts = state_ts;

        OdomMsg msg;
        msg.header.stamp = get_ros_time(state_ts);
        msg.header.frame_id = odom_frame;
        msg.child_frame_id = robot_hf_frame;

        const Eigen::Quaterniond imu_rotation(rot);
        const RobotPose robot_pose = robot_pose_from_imu(
            pos, imu_rotation,
            t_lidar_in_imu_fixed, R_lidar_in_imu_fixed);
        msg.pose.pose.position.x = robot_pose.position.x();
        msg.pose.pose.position.y = robot_pose.position.y();
        msg.pose.pose.position.z = robot_pose.position.z();
        msg.pose.pose.orientation.x = robot_pose.rotation.x();
        msg.pose.pose.orientation.y = robot_pose.rotation.y();
        msg.pose.pose.orientation.z = robot_pose.rotation.z();
        msg.pose.pose.orientation.w = robot_pose.rotation.w();

        // nav_msgs/Odometry requires twist to be expressed in child_frame_id.
        // `vel` is the EKF IMU-origin velocity in odom/world coordinates;
        // translate it to the robot origin first, then rotate it into the
        // robot (base_link_hf) frame. This makes /OdometryHighFreq suitable
        // as CLOSED_LOOP feedback for a holonomic chassis.
        V3D angular_vel_body = Zero3d;
        V3D linear_vel_body =
            robot_pose.rotation.conjugate() * vel;
        if (have_last_imu) {
            const V3D angular_vel_imu = last_gyro - bg;
            const V3D angular_vel_world = rot * angular_vel_imu;
            const V3D robot_offset_world = robot_pose.position - pos;
            const V3D linear_vel_robot_world =
                vel + angular_vel_world.cross(robot_offset_world);
            linear_vel_body =
                robot_pose.rotation.conjugate() * linear_vel_robot_world;
            angular_vel_body =
                robot_pose.rotation.conjugate() * angular_vel_world;
        }
        msg.twist.twist.linear.x = linear_vel_body.x();
        msg.twist.twist.linear.y = linear_vel_body.y();
        msg.twist.twist.linear.z = linear_vel_body.z();
        msg.twist.twist.angular.x = angular_vel_body.x();
        msg.twist.twist.angular.y = angular_vel_body.y();
        msg.twist.twist.angular.z = angular_vel_body.z();
        ros_publish(pubOdomHighFreq, msg);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = msg.header.stamp;
        tf_msg.header.frame_id = odom_frame;
        tf_msg.child_frame_id = robot_hf_frame;
        tf_msg.transform.translation.x = msg.pose.pose.position.x;
        tf_msg.transform.translation.y = msg.pose.pose.position.y;
        tf_msg.transform.translation.z = msg.pose.pose.position.z;
        tf_msg.transform.rotation = msg.pose.pose.orientation;
        br_hf->sendTransform(tf_msg);
    }
}

template<typename T>
void set_posestamp(T & out)
{
    const RobotPose robot_pose = robot_pose_from_imu(
        V3D(state_point.pos(0), state_point.pos(1), state_point.pos(2)),
        Eigen::Quaterniond(state_point.rot.toRotationMatrix()),
        V3D(state_point.t_lidar_in_imu(0), state_point.t_lidar_in_imu(1), state_point.t_lidar_in_imu(2)),
        state_point.R_lidar_in_imu.toRotationMatrix());
    out.pose.position.x = robot_pose.position.x();
    out.pose.position.y = robot_pose.position.y();
    out.pose.position.z = robot_pose.position.z();
    out.pose.orientation.x = robot_pose.rotation.x();
    out.pose.orientation.y = robot_pose.rotation.y();
    out.pose.orientation.z = robot_pose.rotation.z();
    out.pose.orientation.w = robot_pose.rotation.w();
}

void publish_odometry(const OdomPublisher & pubOdomAftMapped, double timestamp = -1.0)
{
    odomAftMapped.header.frame_id = odom_frame;
    odomAftMapped.child_frame_id = robot_frame;
    set_posestamp(odomAftMapped.pose);
    odomAftMapped.header.stamp = get_ros_time(timestamp >= 0.0 ? timestamp : lidar_end_time);
    if (odom_log_interval_sec > 0.0) {
        static auto last_log_time = std::chrono::steady_clock::time_point{};
        const auto now = std::chrono::steady_clock::now();
        if (last_log_time.time_since_epoch().count() == 0 ||
            now - last_log_time >= std::chrono::duration<double>(odom_log_interval_sec)) {
            const RobotPose robot_pose = robot_pose_from_imu(
                V3D(state_point.pos(0), state_point.pos(1), state_point.pos(2)),
                Eigen::Quaterniond(state_point.rot.toRotationMatrix()),
                V3D(state_point.t_lidar_in_imu(0), state_point.t_lidar_in_imu(1), state_point.t_lidar_in_imu(2)),
                state_point.R_lidar_in_imu.toRotationMatrix());
            /*
             ! Eigen::eulerAngles(2,1,0) constrains its first angle to [0, pi]:
             ! when the true yaw drifts across 0 deg, the log spuriously jumps
             ! to ~180 deg on all three angles although the quaternion is
             ! continuous. Use plain atan2 extraction, which matches
             ! tf2::Matrix3x3::getRPY used downstream.
            */
            const Eigen::Matrix3d rot_mat = robot_pose.rotation.toRotationMatrix();
            const double yaw_rad = std::atan2(rot_mat(1, 0), rot_mat(0, 0));
            const double pitch_rad =
                std::atan2(-rot_mat(2, 0), std::hypot(rot_mat(0, 0), rot_mat(1, 0)));
            const double roll_rad = std::atan2(rot_mat(2, 1), rot_mat(2, 2));
            const V3D rpy_deg(
                roll_rad * 180.0 / M_PI,
                pitch_rad * 180.0 / M_PI,
                yaw_rad * 180.0 / M_PI);
            ROS_PRINT_INFO(
                "odom[%s -> %s] xyz=(%.3f, %.3f, %.3f) m rpy=(%.2f, %.2f, %.2f) deg",
                odom_frame.c_str(), robot_frame.c_str(),
                robot_pose.position.x(), robot_pose.position.y(), robot_pose.position.z(),
                rpy_deg.x(), rpy_deg.y(), rpy_deg.z());
            last_log_time = now;
        }
    }

    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    ros_publish(pubOdomAftMapped, odomAftMapped);

    static auto br = std::make_shared<tf2_ros::TransformBroadcaster>(get_ros_node());

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = odomAftMapped.header.stamp;
    tf_msg.header.frame_id = odom_frame;
    tf_msg.child_frame_id  = robot_frame;

    tf_msg.transform.translation.x = odomAftMapped.pose.pose.position.x;
    tf_msg.transform.translation.y = odomAftMapped.pose.pose.position.y;
    tf_msg.transform.translation.z = odomAftMapped.pose.pose.position.z;
    tf_msg.transform.rotation = odomAftMapped.pose.pose.orientation;

    br->sendTransform(tf_msg);
}

void publish_path(const PathPublisher pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time);

        msg_body_pose.header.frame_id = odom_frame;

        /*** if path is too large, rviz will crash ***/
        static int jjj = 0;
        jjj++;

        if (jjj % 10 == 0)
        {
            path.poses.push_back(msg_body_pose);
            path.header.stamp = msg_body_pose.header.stamp;
            ros_publish(pubPath, path);
        }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    if (point_selected_surf.size() < static_cast<size_t>(feats_down_size))
    {
        point_selected_surf.resize(feats_down_size, 1);
    }
    if (res_last.size() < static_cast<size_t>(feats_down_size))
    {
        res_last.resize(feats_down_size, -1000.0f);
    }
    laserCloudOri->resize(feats_down_size);
    corr_normvect->resize(feats_down_size);

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.R_lidar_in_imu * p_body + s.t_lidar_in_imu) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        return;
    }

    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        V3D point_this = s.R_lidar_in_imu * point_this_be + s.t_lidar_in_imu;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            M3D point_be_crossmat;
            point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
            V3D B(point_be_crossmat * s.R_lidar_in_imu.conjugate() * C);
            ekfom_data.h_x.block<1, 12>(i,0) <<
                norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A),
                VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) <<
                norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A),
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    init_ros_node(rclcpp::Node::make_shared("fast_lio"));

    // Load ROS 2 parameters.
    rosparam_get("publish/path_en", path_en, true);
    rosparam_get("publish/scan_publish_en", scan_pub_en, true);
    rosparam_get("publish/dense_publish_en", dense_pub_en, true);
    rosparam_get("publish/scan_bodyframe_pub_en", scan_body_pub_en, true);
    rosparam_get("publish/feature_pub_en", feature_pub_en, false);
    rosparam_get("publish/effect_pub_en", effect_pub_en, false);
    rosparam_get("reloc/reloc_en", reloc_en, false);
    rosparam_get("max_iteration", NUM_MAX_ITERATIONS, 4);
    rosparam_get("common/lid_topic", lid_topic, std::string("/livox/lidar"));
    rosparam_get("common/imu_topic", imu_topic, std::string("/livox/imu"));
    rosparam_get("frames/odom", odom_frame, std::string("odom"));
    rosparam_get("frames/robot", robot_frame, std::string("base_link"));
    rosparam_get("frames/robot_hf", robot_hf_frame, std::string("base_link_hf"));
    rosparam_get("frames/imu", imu_frame, std::string("imu_link"));
    rosparam_get("frames/lidar", lidar_frame, std::string("livox_frame"));
    rosparam_get("odometry/zero_at_start", zero_odom_at_start, true);
    rosparam_get("diagnostics/odom_log_interval_sec", odom_log_interval_sec, 1.0);
    rosparam_get("reloc/reloc_topic", reloc_topic, std::string("/reloc/cloud_align"));
    rosparam_get("common/time_sync_en", time_sync_en, false);
    rosparam_get("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    rosparam_get("filter_size_corner", filter_size_corner_min, 0.5);
    rosparam_get("filter_size_surf", filter_size_surf_min, 0.5);
    rosparam_get("filter_size_map", filter_size_map_min, 0.5);
    rosparam_get("cube_side_length", cube_len, 200.0);
    rosparam_get("mapping/det_range", DET_RANGE, 300.f);
    rosparam_get("mapping/fov_degree", fov_deg, 180.0);
    rosparam_get("mapping/gyr_cov", gyr_cov, 0.1);
    rosparam_get("mapping/acc_cov", acc_cov, 0.1);
    rosparam_get("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
    rosparam_get("mapping/b_acc_cov", b_acc_cov, 0.0001);
    rosparam_get("preprocess/blind", p_pre->blind, 0.01);
    rosparam_get("preprocess/lidar_type", lidar_type, (int)AVIA);
    rosparam_get("preprocess/scan_line", p_pre->N_SCANS, 16);
    rosparam_get("preprocess/timestamp_unit", p_pre->time_unit, (int)US);
    rosparam_get("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    rosparam_get("point_filter_num", p_pre->point_filter_num, 2);
    rosparam_get("feature_extract_enable", p_pre->feature_enabled, false);
    rosparam_get("runtime_pos_log_enable", runtime_pos_log, false);
    rosparam_get("pcd_save/pcd_save_en", pcd_save_en, false);
    rosparam_get("pcd_save/interval", pcd_save_interval, -1);
    rosparam_get("mapping/extrinsic_est_en", extrinsic_est_en, false);
    rosparam_get("sensor_extrinsic/imu_in_lidar_T", param_t_imu_in_lidar,
                 std::vector<double>{0.011, 0.02329, -0.04412});
    rosparam_get("sensor_extrinsic/imu_in_lidar_R", param_R_imu_in_lidar,
                 std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    rosparam_get("robot_extrinsic/lidar_to_robot_T", param_t_lidar_in_robot, std::vector<double>{0.0, 0.0, 0.0});
    rosparam_get("robot_extrinsic/lidar_to_robot_R", param_R_lidar_in_robot,
                 std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});

    if (odom_log_interval_sec < 0.0) {
        ROS_PRINT_WARN("diagnostics/odom_log_interval_sec must be >= 0; disabling odometry logs.");
        odom_log_interval_sec = 0.0;
    }
    if (pcd_save_interval == 0 || pcd_save_interval < -1) {
        ROS_PRINT_WARN("pcd_save/interval must be -1 or a positive integer; forcing -1.");
        pcd_save_interval = -1;
    }
    const std::array<std::string, 5> frame_names = {
        odom_frame, robot_frame, robot_hf_frame, lidar_frame, imu_frame};
    std::set<std::string> unique_frames;
    for (const auto &frame : frame_names) {
        if (frame.empty() || frame.front() == '/' || !unique_frames.insert(frame).second) {
            ROS_PRINT_ERROR(
                "frames must be non-empty, unique TF names without a leading '/': "
                "odom=%s robot=%s robot_hf=%s lidar=%s imu=%s",
                odom_frame.c_str(), robot_frame.c_str(), robot_hf_frame.c_str(),
                lidar_frame.c_str(), imu_frame.c_str());
            return 1;
        }
    }
    if (param_t_imu_in_lidar.size() != 3 || param_R_imu_in_lidar.size() != 9 ||
        param_t_lidar_in_robot.size() != 3 || param_R_lidar_in_robot.size() != 9 ||
        !is_finite_vector(param_t_imu_in_lidar) ||
        !is_finite_vector(param_t_lidar_in_robot) ||
        !is_proper_rotation_matrix(param_R_imu_in_lidar) ||
        !is_proper_rotation_matrix(param_R_lidar_in_robot)) {
        ROS_PRINT_ERROR(
            "sensor_extrinsic and robot_extrinsic must use T=[x,y,z] and "
            "proper row-major 3x3 rotation matrices.");
        return 1;
    }
    if (extrinsic_est_en) {
        ROS_PRINT_WARN(
            "mapping/extrinsic_est_en=true: EKF T_lidar^imu may change while the public "
            "%s -> %s static TF remains the configured YAML value.",
            lidar_frame.c_str(), imu_frame.c_str());
    }
    path.header.stamp = get_ros_now();
    path.header.frame_id = odom_frame;

    p_pre->lidar_type = lidar_type;
    cout<<"p_pre->lidar_type "<<p_pre->lidar_type<<endl;

    /*** variables definition ***/
    int frame_num = 0;
    double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0,
           aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    
    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZI());

    point_selected_surf.assign(100000, 1);
    res_last.assign(100000, -1000.0f);
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

    // Convention: T_B^A is the pose of B relative to A and maps p^B to p^A.
    // The official dimensions directly provide T_imu^lidar.
    t_imu_in_lidar<<VEC_FROM_ARRAY(param_t_imu_in_lidar);
    R_imu_in_lidar<<MAT_FROM_ARRAY(param_R_imu_in_lidar);
    R_imu_in_lidar = Eigen::Quaterniond(R_imu_in_lidar).normalized().toRotationMatrix();

    // FAST-LIO point processing uses T_lidar^imu = inverse(T_imu^lidar).
    R_lidar_in_imu_fixed = R_imu_in_lidar.transpose();
    t_lidar_in_imu_fixed = -R_lidar_in_imu_fixed * t_imu_in_lidar;

    // robot_extrinsic directly provides T_lidar^robot.
    t_lidar_in_robot<<VEC_FROM_ARRAY(param_t_lidar_in_robot);
    R_lidar_in_robot<<MAT_FROM_ARRAY(param_R_lidar_in_robot);
    // YAML matrices are rounded decimal values. Project them back onto SO(3)
    // before inversion/composition so transpose is a true inverse.
    R_lidar_in_robot = Eigen::Quaterniond(R_lidar_in_robot).normalized().toRotationMatrix();
    // 自车几何滤波在车体系下判，需要完整的 T_lidar^base。
    p_imu->set_initial_lidar_in_imu(t_lidar_in_imu_fixed, R_lidar_in_imu_fixed);
    if (!zero_odom_at_start) {
        odom_origin_ready.store(true, std::memory_order_release);
    }
    publish_static_sensor_transforms();
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    p_imu->lidar_type = lidar_type;
    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(),"w");
    
    create_directory(pcd_output_dir());

    ofstream fout_pre, fout_out, fout_dbg;
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
    fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
    if (fout_pre && fout_out)
        cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    else
        cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

    /*** ROS subscribe initialization ***/
    rclcpp::SensorDataQoS lidar_qos;
    lidar_qos.keep_last(20);
    rclcpp::SensorDataQoS imu_qos;
    imu_qos.keep_last(2000);
    if (p_pre->lidar_type == AVIA) {
        static auto sub_pcl = create_subscriber_qos<LivoxMsg>(lid_topic, lidar_qos, livox_pcl_cbk);
    } else {
        static auto sub_pcl = create_subscriber_qos<PointCloud2Msg>(lid_topic, lidar_qos, standard_pcl_cbk);
    }
    
    rclcpp::Subscription<PoseStampedMsg>::SharedPtr sub_reloc;
    if (reloc_en) {
        sub_reloc = create_subscriber<PoseStampedMsg>(reloc_topic, 10, reloc_cbk);
        ROS_PRINT_INFO("reloc input enabled: %s [%s]", reloc_topic.c_str(), odom_frame.c_str());
    }
    auto sub_imu = create_subscriber_qos<ImuMsg>(imu_topic, imu_qos, imu_cbk);

    auto pubLaserCloudFull = create_publisher<PointCloud2Msg>("/cloud_registered", 10);
    auto pubLaserCloudFull_body = create_publisher<PointCloud2Msg>("/cloud_registered_body", 10);
    auto pubLaserCloudEffect = create_publisher<PointCloud2Msg>("/cloud_effected", 10);
    auto pubLaserCloudMap = create_publisher<PointCloud2Msg>("/Laser_map", 10);
    auto mapping_odom_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto high_freq_odom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    auto path_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto pubOdomAftMapped = create_publisher_qos<OdometryMsg>("/Odometry", mapping_odom_qos);
    auto pubPath = create_publisher_qos<PathMsg>("/path", path_qos);
    auto pubOdomHighFreq = create_publisher_qos<OdometryMsg>("/OdometryHighFreq", high_freq_odom_qos);
    p_pre->pub_corn = create_publisher<PointCloud2Msg>("/corn_feature", 10);
    p_pre->pub_surf = create_publisher<PointCloud2Msg>("/surf_feature", 10);

    std::thread odomhighthread([&](){
        publish_odometryhighfreq(pubOdomHighFreq);
    });

//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    RateType rate(5000);
    while (ros_ok() && !flg_exit)
    {
        spin_once();

        if(reloc_en)
        {
            // relocalization trigger
            if(relocalize_flag.exchange(false, std::memory_order_acq_rel))
            {
                std::lock_guard<std::mutex> output_lock(mtx_odom_output);
                odom_origin_ready.store(false, std::memory_order_release);
                feats_down_world->clear();
                p_imu->Reset();
                // Reloc input is T_robot^odom. Convert it to the EKF state T_imu^odom.
                state_ikfom state_point_reloc = kf.get_x();
                RelocState requested_robot_state;
                {
                    std::lock_guard<std::mutex> lock(mtx_reloc);
                    requested_robot_state = reloc_state;
                }
                const RobotPose requested_robot_pose{
                    V3D(requested_robot_state.x_, requested_robot_state.y_, requested_robot_state.z_),
                    Eigen::Quaterniond(requested_robot_state.qw_, requested_robot_state.qx_,
                                       requested_robot_state.qy_, requested_robot_state.qz_)};
                const RobotPose requested_imu_pose = imu_pose_from_robot(requested_robot_pose);
                state_point_reloc.pos = requested_imu_pose.position;
                state_point_reloc.rot = requested_imu_pose.rotation;
                kf.reset(state_point_reloc);
                state_point = state_point_reloc;
                // Re-seed the high-frequency stream at the relocated pose; old
                // buffered samples were removed by ImuProcess::Reset().
                hf_update_base(state_point_reloc, requested_robot_state.timestamp_,
                               p_imu->get_acc_scale());
                ikdtree.delete_tree_nodes(&ikdtree.Root_Node);

                // Keep the low-frequency robot TF consistent with the relocated
                // high-frequency stream immediately, before mapping reinitializes.
                publish_odometry(pubOdomAftMapped, requested_robot_state.timestamp_);

                odom_origin_ready.store(true, std::memory_order_release);

                ROS_PRINT_INFO(
                    "Reloc robot pose: xyz=(%.2f %.2f %.2f), quat=(%.2f %.2f %.2f %.2f)",
                    requested_robot_state.x_, requested_robot_state.y_, requested_robot_state.z_,
                    requested_robot_state.qx_, requested_robot_state.qy_,
                    requested_robot_state.qz_, requested_robot_state.qw_);
                flg_first_scan = true;
                continue;
            }
        }   

        if(sync_packages(Measures)) 
        {
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }

            double t0, t1, t2, t3, t5;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            t0 = omp_get_wtime();

            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.t_lidar_in_imu; // T_lidar^odom translation

            if (!feats_undistort || feats_undistort->empty())
            {
                ROS_PRINT_WARN("No point, skip this scan!");
                continue;
            }

            if (initialize_robot_centered_odom_origin(lidar_end_time)) {
                // Publish one exact identity sample before normal mapping starts.
                publish_odometry(pubOdomAftMapped);
            }
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.t_lidar_in_imu;

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                continue;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                ROS_PRINT_WARN("No point, skip this scan!");
                continue;
            }

            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.R_lidar_in_imu);
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose()<<" "<<state_point.t_lidar_in_imu.transpose()<<" "<<state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            if(feature_pub_en) // If you need to see map point, change to "if(1)"
            {
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();
            
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();
            // Refresh the high-frequency prediction base with the corrected
            // state; the publish thread re-predicts forward at IMU rate.
            hf_update_base(state_point, lidar_end_time, p_imu->get_acc_scale());
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.t_lidar_in_imu;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();
            
            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped);

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();
            
            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath);
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);
            if (effect_pub_en) publish_effect_world(pubLaserCloudEffect);
            if (feature_pub_en) publish_map(pubLaserCloudMap);

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter ++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.R_lidar_in_imu);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.t_lidar_in_imu.transpose()<<" "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
                dump_lio_state_to_log(fp);
            }
        }

        rate.sleep();
    }            

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        const string file_name = pcd_save_interval > 0
            ? "scans_" + std::to_string(++pcd_index) + ".pcd"
            : "scans.pcd";
        string all_points_dir = pcd_output_dir() + file_name;
        pcl::PCDWriter pcd_writer;
        cout << "saving accumulated global map to " << all_points_dir << endl;
        const int save_result = pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
        if (save_result != 0) {
            ROS_PRINT_ERROR("failed to save accumulated global map: %s", all_points_dir.c_str());
        } else {
            ROS_PRINT_INFO("saved accumulated global map: %s (%zu points)",
                           all_points_dir.c_str(), pcl_wait_save->size());
        }
    }

    fout_out.close();
    fout_pre.close();

    if (!flg_exit && runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }
    
    flg_exit = true;
    if (odomhighthread.joinable()) {
        odomhighthread.join();
    }
    return 0;
}
