#pragma once
#include "lidar_utils/types.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <small_gicp/points/point_cloud.hpp>

namespace lidar_utils {
namespace internal {

std::shared_ptr<small_gicp::PointCloud>
toGicpCloud(const CloudType::Ptr &pcl_cloud) {
  auto gicp_cloud = std::make_shared<small_gicp::PointCloud>();
  gicp_cloud->points.resize(pcl_cloud->size());
  for (size_t i = 0; i < pcl_cloud->size(); ++i) {
    gicp_cloud->points[i] =
        Eigen::Vector4f(pcl_cloud->points[i].x, pcl_cloud->points[i].y,
                        pcl_cloud->points[i].z, 1.0f)
            .cast<double>();
  }
  return gicp_cloud;
}

int executeGICP(CloudType::Ptr &cloud, CloudType::Ptr &map,
                Eigen::Matrix4d &in_transform, Eigen::Matrix4d &out_transform,
                const RegistrationConfig &config);

int executeNDT(CloudType::Ptr &cloud, CloudType::Ptr &map,
               Eigen::Matrix4d &in_transform, Eigen::Matrix4d &out_transform,
               const RegistrationConfig &config);

} // namespace internal
} // namespace lidar_utils