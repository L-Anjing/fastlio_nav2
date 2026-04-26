#include "utility.h"
#include "ros_utils.h"

using namespace std;

namespace
{
Eigen::Quaterniond yawOnlyQuaternion(const Eigen::Quaterniond &orientation)
{
    const Eigen::Vector3d euler = orientation.toRotationMatrix().eulerAngles(2, 1, 0);
    return Eigen::Quaterniond(Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitZ())).normalized();
}
} // namespace

// Topics
string lidarFrame;
string baselinkFrame;
string odometryFrame;
string mapFrame;
string highFrequencyBaselinkFrame;

// CPU Params
int numberOfCores;

// Surrounding map
float surroundingkeyframeAddingDistThreshold; 
float surroundingkeyframeAddingAngleThreshold; 
float surroundingKeyframeSearchRadius;

// Loop closure
bool  loopClosureEnableFlag;
float loopClosureFrequency;
int   surroundingKeyframeSize;
float historyKeyframeSearchRadius;
float historyKeyframeSearchTimeDiff;
int   historyKeyframeSearchNum;
float historyKeyframeFitnessScore;

// global map visualization radius
float globalMapVisualizationSearchRadius;
float globalMapVisualizationPoseDensity;
float globalMapVisualizationLeafSize;

float mappingICPSize;

bool scanSliceEnable;
float scanSliceMinZ;
float scanSliceMaxZ;
float scanAngleMin;
float scanAngleMax;
float scanAngleIncrement;
float scanTime;
float scanRangeMin;
float scanRangeMax;

int ikdtreeSearchNeighborNum;
bool occupancyMapEnabled = true;
bool mapFrameOriginInitialized = false;
Eigen::Quaterniond mapFrameRotationFromOdom = Eigen::Quaterniond::Identity();
Eigen::Vector3d mapFrameTranslationFromOdom = Eigen::Vector3d::Zero();

void read_liosam_params() {
    // CPU parameters
    rosparam_get("lio_sam/numberOfCores", numberOfCores, 2);

    // Keyframe Strategy
    rosparam_get("lio_sam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0f);
    rosparam_get("lio_sam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2f);
    rosparam_get("lio_sam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0f);

    // Loop closure parameters
    rosparam_get("lio_sam/loopClosureEnableFlag", loopClosureEnableFlag, false);
    rosparam_get("lio_sam/loopClosureFrequency", loopClosureFrequency, 1.0f);
    rosparam_get("lio_sam/surroundingKeyframeSize", surroundingKeyframeSize, 50);
    rosparam_get("lio_sam/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0f);
    rosparam_get("lio_sam/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0f);
    rosparam_get("lio_sam/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
    rosparam_get("lio_sam/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3f);

    // Global pointcloud visualization
    rosparam_get("lio_sam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3f);
    rosparam_get("lio_sam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0f);
    rosparam_get("lio_sam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0f);

    rosparam_get("lio_sam/mappingICPSize", mappingICPSize, 0.2f);
    rosparam_get("occupancy_map/enabled", occupancyMapEnabled, true);
    rosparam_get("lio_sam/ikdtreeSearchNeighborNum", ikdtreeSearchNeighborNum, 8);
}

void read_frame_params() {
    rosparam_get("common/lidarFrame", lidarFrame, std::string("lidar"));
    rosparam_get("common/baselinkFrame", baselinkFrame, std::string("base_link"));
    rosparam_get("common/odometryFrame", odometryFrame, std::string("odom"));
    rosparam_get("common/mapFrame", mapFrame, std::string("map"));
    rosparam_get("common/highFrequencyBaselinkFrame", highFrequencyBaselinkFrame, std::string("base_link_hf"));
}

void read_pcl2scan_params() {
    rosparam_get("pointcloud_to_laserscan/slice_enable", scanSliceEnable, true);
    rosparam_get("pointcloud_to_laserscan/min_height", scanSliceMinZ, -1.5f);
    rosparam_get("pointcloud_to_laserscan/max_height", scanSliceMaxZ, 0.8f);
    rosparam_get("pointcloud_to_laserscan/angle_min", scanAngleMin, -3.14159f);
    rosparam_get("pointcloud_to_laserscan/angle_max", scanAngleMax, 3.14159f);
    rosparam_get("pointcloud_to_laserscan/angle_increment", scanAngleIncrement, 0.00436f);
    rosparam_get("pointcloud_to_laserscan/scan_time", scanTime, 0.1f);
    rosparam_get("pointcloud_to_laserscan/range_min", scanRangeMin, 0.1f);
    rosparam_get("pointcloud_to_laserscan/range_max", scanRangeMax, 100.0f);
}

void setMapFrameOriginFromPose(const Eigen::Vector3d &origin_position,
                               const Eigen::Quaterniond &origin_orientation)
{
    const Eigen::Quaterniond origin_yaw = yawOnlyQuaternion(origin_orientation.normalized());
    mapFrameRotationFromOdom = origin_yaw.conjugate().normalized();
    mapFrameTranslationFromOdom = -(mapFrameRotationFromOdom * origin_position);
    mapFrameOriginInitialized = true;
}
