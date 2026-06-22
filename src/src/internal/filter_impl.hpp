#pragma once
#include "lidar_utils/types.hpp"
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_utils {
namespace internal {
void executeCrop(CloudType::Ptr &cloud, const Eigen::Vector3f &minBound,
                 const Eigen::Vector3f &maxBound,
                 std::vector<double> *timestamps = nullptr);

void executeDenoise(CloudType::Ptr &cloud, float radius, float epsilon);

void executeRemoveArtifact(CloudType::Ptr &cloud,
                           std::vector<double> *timestamps = nullptr);

void executeDownsample(CloudType::Ptr &cloud, float voxelSize,
                       std::vector<double> *timestamps = nullptr);

void executeRemoveNaN(CloudType::Ptr &cloud,
                      std::vector<double> *timestamps = nullptr);

void executeExtractGround(CloudType::Ptr &cloudIn, CloudType::Ptr &groundCloud,
                          CloudType::Ptr &nonGroundCloud,
                          double distanceThreshold, int maxIterations);

} // namespace internal
} // namespace lidar_utils