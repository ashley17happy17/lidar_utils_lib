#pragma once
#include <Eigen/Dense>

namespace lidar_utils {
namespace internal {

void executeEOPCalib(const Eigen::MatrixXd &la, const Eigen::MatrixXd &bs,
                     const Eigen::Vector3d &laCalib,
                     const Eigen::Vector3d &bsCalib);

} // namespace internal
} // namespace lidar_utils