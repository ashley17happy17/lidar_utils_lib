#pragma once
#include "pc_utils/types.hpp"
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pc_utils {
namespace internal {

void executeReorder(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                    SensorType sensorType);

} // namespace internal
} // namespace pc_utils