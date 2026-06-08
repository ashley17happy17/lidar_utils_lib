#pragma once
#include "lidar_utils/types.hpp"
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_utils {
namespace internal {

void executeMerge(CloudType::Ptr &base_cloud, ConstCloudType other_cloud,
                  float voxelSize);

} // namespace internal
} // namespace lidar_utils