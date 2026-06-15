#pragma once
#include <Eigen/Dense>

namespace lidar_utils {
namespace internal {

Eigen::Matrix4d
executeEOPCalib(const std::vector<Eigen::Matrix4d> &gps_relative_motions,
                const std::vector<Eigen::Matrix4d> &lidar_relative_motions);

Eigen::Matrix4d executeTransformEOP(const Eigen::Matrix4d &eop,
                                    const Eigen::Vector3d &out_la,
                                    const Eigen::Vector3d &out_bs);

void executePrintEOP(const Eigen::Matrix4d &eop,
                     const std::string &sensor_name);

Eigen::Matrix4d executeGetExtrinsics(const std::string &type,
                                     const Eigen::Vector3d &trans,
                                     const Eigen::Vector3d &rot);

} // namespace internal
} // namespace lidar_utils