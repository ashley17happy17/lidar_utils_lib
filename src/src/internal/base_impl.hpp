#pragma once
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_utils {
namespace internal {
void executeCrop(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                 const Eigen::Vector3f &minBound,
                 const Eigen::Vector3f &maxBound);

void executeDenoise(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                    float denoiseThres);

void executeDownsample(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                       float voxelSize);

void mergeCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &base_cloud,
                pcl::PointCloud<pcl::PointXYZI>::Ptr &other_cloud,
                float voxelSize);

} // namespace internal
} // namespace lidar_utils