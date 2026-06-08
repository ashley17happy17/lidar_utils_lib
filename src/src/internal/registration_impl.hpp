#pragma once
#include "lidar_utils/types.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_utils {
namespace internal {

void executeScanToMapMatching(CloudType::Ptr &cloud, CloudType::Ptr &map,
                              Eigen::Matrix4d &in_transform,
                              Eigen::Matrix4d &out_transform,
                              float max_correspondence_distance,
                              float voxel_size, float score_threshold);

void executeScanToScanMatching(CloudType::Ptr &cloud, CloudType::Ptr &localmap,
                               Eigen::Matrix4d &in_transform,
                               Eigen::Matrix4d &out_transform,
                               float max_correspondence_distance,
                               float voxel_size, float score_threshold);

} // namespace internal
} // namespace lidar_utils