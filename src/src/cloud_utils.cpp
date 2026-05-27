#include "lidar_utils/cloud_utils.hpp"
#include "internal/base_impl.hpp"
#include "internal/motion_impl.hpp"
#include "internal/reorder_impl.hpp"

namespace lidar_utils {

void CloudUtils::reorderCloud(CloudType::Ptr &cloud, SensorType sensorType) {
  // The internal implementation will modify 'cloud' in-place now
  internal::executeReorder(cloud, sensorType);
}

void CloudUtils::motionCompensateAndDG(CloudType::Ptr &cloud,
                                       const Eigen::Matrix4d &posPrev,
                                       const Eigen::Matrix4d &posCurr,
                                       SensorType sensorType,
                                       bool motionEnable) {
  // The internal implementation will overwrite 'cloud' with the compensated
  // world cloud
  internal::executeMotionAndDG(cloud, posPrev, posCurr, sensorType,
                               motionEnable);
}

void CloudUtils::cropCloud(CloudType::Ptr &cloud,
                           const Eigen::Vector3f &minBound,
                           const Eigen::Vector3f &maxBound) {
  internal::executeCrop(cloud, minBound, maxBound);
}

void CloudUtils::denoiseCloud(CloudType::Ptr &cloud, float denoiseThres) {
  internal::executeDenoise(cloud, denoiseThres);
}

void CloudUtils::downsampleCloud(CloudType::Ptr &cloud, float voxelSize) {
  internal::executeDownsample(cloud, voxelSize);
}

void CloudUtils::mergeCloud(CloudType::Ptr &base_cloud,
                            const CloudType::ConstPtr &other_cloud,
                            float voxelSize) {
  if (base_cloud && other_cloud) {
    *base_cloud += *other_cloud; // PCL native efficient concatenation
  }
}

} // namespace lidar_utils