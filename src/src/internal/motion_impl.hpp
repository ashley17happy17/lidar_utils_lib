#pragma once
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pc_utils/types.hpp"

namespace pc_utils {
namespace internal {

void executeMotionAndDG(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                        const Eigen::Matrix4d &posPrev,
                        const Eigen::Matrix4d &posCurr, SensorType sensorType,
                        bool motionEnable);

} // namespace internal
} // namespace pc_utils