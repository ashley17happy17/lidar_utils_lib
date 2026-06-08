#include "internal/registration_impl.hpp"
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/search/kdtree.h>

namespace lidar_utils {
namespace internal {

void executeScanToMapMatching(CloudType::Ptr &cloud, CloudType::Ptr &map,
                              Eigen::Matrix4d &in_transform,
                              Eigen::Matrix4d &out_transform,
                              float max_correspondence_distance,
                              float voxel_size, float score_threshold) {}

void executeScanToScanMatching(CloudType::Ptr &cloud, CloudType::Ptr &localmap,
                               Eigen::Matrix4d &in_transform,
                               Eigen::Matrix4d &out_transform,
                               float max_correspondence_distance,
                               float voxel_size, float score_threshold) {}

} // namespace internal
} // namespace lidar_utils