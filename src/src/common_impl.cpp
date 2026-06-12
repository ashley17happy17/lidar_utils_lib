#include "internal/common_impl.hpp"
#include "internal/filter_impl.hpp"
#include <pcl/filters/voxel_grid.h>

namespace lidar_utils {
namespace internal {

void executeMerge(CloudType::Ptr &base_cloud,
                  ConstCloudType other_cloud,
                  float voxelSize) {
  if (!base_cloud) {
    base_cloud.reset(new CloudType());
  }
  if (!other_cloud || other_cloud->empty()) {
    return;
  }

  *base_cloud += *other_cloud;

  if (voxelSize > 0.0f) {
    executeDownsample(base_cloud, voxelSize);
  }
}

} // namespace internal
} // namespace lidar_utils