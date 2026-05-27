#pragma once
#include "lidar_utils/types.hpp"
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_utils {
namespace internal {

void executeReorder(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                    SensorType sensorType);

} // namespace internal
} // namespace lidar_utils