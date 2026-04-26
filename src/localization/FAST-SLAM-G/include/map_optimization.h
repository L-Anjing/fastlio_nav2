#ifndef MAP_OPTIMIZATION_H
#define MAP_OPTIMIZATION_H

extern float transformTobeMapped[6];
extern Eigen::Vector3d translationLidarToIMU;
extern Eigen::Matrix3d rotationLidarToIMU;

void MapOptimizationInit();

void saveKeyFramesAndFactor(
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr feats_down_body,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr feats_undistort);

void correctPoses();

void publishSamMsg();

void loopClosureThread();

void setLaserCurTime(double lidar_end_time);

void visualizeGlobalMapThread();

#endif
