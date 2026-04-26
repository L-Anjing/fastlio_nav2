#ifndef UTILITY_H
#define UTILITY_H

#include <string>
#include <deque>
#include <vector>
#include <atomic>

#include <Eigen/Geometry>
#include <pcl_conversions/pcl_conversions.h>
#include "ros_utils.h"

//Topics
extern std::string lidarFrame;
extern std::string baselinkFrame;
extern std::string odometryFrame;
extern std::string mapFrame;
extern std::string highFrequencyBaselinkFrame;

// CPU Params
extern int numberOfCores;

// Surrounding map
extern float surroundingkeyframeAddingDistThreshold; 
extern float surroundingkeyframeAddingAngleThreshold; 
extern float surroundingKeyframeSearchRadius;

// Loop closure
extern bool  loopClosureEnableFlag;
extern float loopClosureFrequency;
extern int   surroundingKeyframeSize;
extern float historyKeyframeSearchRadius;
extern float historyKeyframeSearchTimeDiff;
extern int   historyKeyframeSearchNum;
extern float historyKeyframeFitnessScore;

// global map visualization radius
extern float globalMapVisualizationSearchRadius;
extern float globalMapVisualizationPoseDensity;
extern float globalMapVisualizationLeafSize;

extern float mappingICPSize;

extern bool scanSliceEnable;
extern float scanSliceMinZ;
extern float scanSliceMaxZ;
extern float scanAngleMin;
extern float scanAngleMax;
extern float scanAngleIncrement;
extern float scanTime;
extern float scanRangeMin;
extern float scanRangeMax;

extern int ikdtreeSearchNeighborNum;
extern bool occupancyMapEnabled;
extern bool mapFrameOriginInitialized;
extern Eigen::Quaterniond mapFrameRotationFromOdom;
extern Eigen::Vector3d mapFrameTranslationFromOdom;

extern std::atomic<bool> flg_exit;

void read_frame_params();
void read_pcl2scan_params();
void read_liosam_params();
void setMapFrameOriginFromPose(const Eigen::Vector3d &origin_position,
                               const Eigen::Quaterniond &origin_orientation);
inline Eigen::Vector3d transformPositionOdomToMap(const Eigen::Vector3d &position)
{
    return mapFrameRotationFromOdom * position + mapFrameTranslationFromOdom;
}

inline Eigen::Quaterniond transformOrientationOdomToMap(const Eigen::Quaterniond &orientation)
{
    return (mapFrameRotationFromOdom * orientation).normalized();
}

template<typename PointT>
inline PointT transformPointOdomToMap(const PointT &point)
{
    PointT mapped = point;
    const Eigen::Vector3d position =
        transformPositionOdomToMap(Eigen::Vector3d(point.x, point.y, point.z));
    mapped.x = position.x();
    mapped.y = position.y();
    mapped.z = position.z();
    return mapped;
}

template<typename PointT>
inline void transformPointOdomToMap(
    const PointT *point_in,
    PointT *point_out)
{
    *point_out = transformPointOdomToMap(*point_in);
}

template<typename PointT>
inline typename pcl::PointCloud<PointT>::Ptr transformCloudOdomToMap(
    const typename pcl::PointCloud<PointT>::Ptr &cloudIn)
{
    typename pcl::PointCloud<PointT>::Ptr cloudOut(new pcl::PointCloud<PointT>());
    cloudOut->resize(cloudIn->size());
    for (std::size_t i = 0; i < cloudIn->points.size(); ++i)
        cloudOut->points[i] = transformPointOdomToMap(cloudIn->points[i]);
    return cloudOut;
}

template<typename T>
void publishCloud(Pcl2Publisher &thisPub, const T &thisCloud, TimeType thisStamp, const std::string &thisFrame)
{
    PointCloud2Msg tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (ros_subscription_count(thisPub) != 0)
        ros_publish(thisPub, tempCloud);
}

#endif
