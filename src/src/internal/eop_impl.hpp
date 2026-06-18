#pragma once
#include "lidar_utils/types.hpp"
#include <Eigen/Dense>

namespace lidar_utils {
namespace internal {

Eigen::Matrix4d executeEOPCalibDynamic(
    const std::vector<Eigen::Matrix4d> &gps_relative_motions,
    const std::vector<Eigen::Matrix4d> &lidar_relative_motions);

Eigen::Matrix4d
executeEOPCalibStatic(const std::vector<Eigen::Matrix4d> &absolute_extrinsics);

Eigen::Matrix4d executeTransformEOP(const Eigen::Matrix4d &eop,
                                    const Eigen::Vector3d &out_la,
                                    const Eigen::Vector3d &out_bs);

void executePrintEOP(const Eigen::Matrix4d &eop, const std::string &sensor_name,
                     DCMOrder order);

Eigen::Matrix4d executeGetExtrinsics(SensorType type,
                                     const Eigen::Vector3d &trans,
                                     const Eigen::Vector3d &rot);

} // namespace internal
} // namespace lidar_utils