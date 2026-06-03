#pragma once
#include "lidar_utils/types.hpp"
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace lidar_utils {
namespace internal {

void executeMotionAndDG(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                        const std::vector<double> &timestamps,
                        const Eigen::VectorXd &posPrev,
                        const Eigen::VectorXd &posCurr, bool motionEnable);

} // namespace internal
} // namespace lidar_utils