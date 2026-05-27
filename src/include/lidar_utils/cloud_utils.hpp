#pragma once
#include "types.hpp"
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_utils {

class CloudUtils {
public:
  using PointType = pcl::PointXYZI;
  using CloudType = pcl::PointCloud<PointType>;

  /**
   * @brief 1. reorderCloud: Reorders point cloud in time sequence (In-place)
   * @note Now changed to void! Modifies the input cloud directly.
   */
  static void reorderCloud(CloudType::Ptr &cloud, SensorType sensorType);

  /**
   * @brief 2. motionCompensateAndDG: Compensates distortion and transforms to
   * world frame (In-place)
   * @note Changed to modify 'cloud' directly to match the toolbox style.
   */
  static void motionCompensateAndDG(CloudType::Ptr &cloud,
                                    const Eigen::Matrix4d &posPrev,
                                    const Eigen::Matrix4d &posCurr,
                                    SensorType sensorType, bool motionEnable);

  /**
   * @brief 3. cropCloud: Cuboid geometric cropping (In-place)
   */
  static void cropCloud(CloudType::Ptr &cloud, const Eigen::Vector3f &minBound,
                        const Eigen::Vector3f &maxBound);

  /**
   * @brief 4. denoiseCloud: Intensity threshold filtering (In-place)
   */
  static void denoiseCloud(CloudType::Ptr &cloud, float denoiseThres);

  /**
   * @brief 5. downsampleCloud: Voxel grid downsampling (In-place)
   */
  static void downsampleCloud(CloudType::Ptr &cloud, float voxelSize);

  /**
   * @brief 6. mergeCloud: Combines other_cloud into base_cloud (In-place)
   */
  static void mergeCloud(CloudType::Ptr &base_cloud,
                         const CloudType::ConstPtr &other_cloud,
                         float voxelSize);
};

} // namespace lidar_utils