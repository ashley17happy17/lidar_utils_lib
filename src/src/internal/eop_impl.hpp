#pragma once
#include <Eigen/Dense>

namespace lidar_utils {
namespace internal {

void executeEOPCalib(const Eigen::MatrixXd &la, const Eigen::MatrixXd &bs,
                     const Eigen::Vector3d &laCalib,
                     const Eigen::Vector3d &bsCalib);

Eigen::Matrix4d executeGetExtrinsics(const std::string &type,
                                     const Eigen::Vector3d &trans,
                                     const Eigen::Vector3d &rot);

} // namespace internal
} // namespace lidar_utils