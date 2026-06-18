#pragma once
#include "lidar_utils/types.hpp"
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace lidar_utils {
namespace internal {

void executeDirectGeoreference(CloudType::Ptr &inCloud,
                               CloudType::Ptr &outCloud,
                               Eigen::Matrix4d &trans);

void executeMotionAndDG(CloudType::Ptr &cloud,
                        const std::vector<double> &timestamps,
                        const Eigen::VectorXd &posCurr,
                        const Eigen::VectorXd &posNext, bool motionEnable);

} // namespace internal
} // namespace lidar_utils