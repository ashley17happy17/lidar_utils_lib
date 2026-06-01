#pragma once
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_utils {
namespace internal {

void executeMergeCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &base_cloud,
                       pcl::PointCloud<pcl::PointXYZI>::ConstPtr other_cloud,
                       float voxelSize);

} // namespace internal
} // namespace lidar_utils