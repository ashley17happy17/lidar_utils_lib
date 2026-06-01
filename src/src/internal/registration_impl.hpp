#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_utils {
namespace internal {

void executeScanToMapMatching(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                              pcl::PointCloud<pcl::PointXYZI>::Ptr &map,
                              Eigen::Matrix4f &in_transform,
                              Eigen::Matrix4f &out_transform,
                              float max_correspondence_distance,
                              float voxel_size, float score_threshold);

void executeScanToScanMatching(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr &localmap,
                               Eigen::Matrix4f &in_transform,
                               Eigen::Matrix4f &out_transform,
                               float max_correspondence_distance,
                               float voxel_size, float score_threshold);

} // namespace internal
} // namespace lidar_utils