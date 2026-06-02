#pragma once
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_utils {
namespace internal {
void executeCrop(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                 const Eigen::Vector3f &minBound,
                 const Eigen::Vector3f &maxBound,
                 std::vector<double> *timestamps = nullptr);

void executeDenoise(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float radius,
                    float epsilon);

void executeRemoveArtifact(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                           std::vector<double> *timestamps = nullptr);

void executeDownsample(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                       float voxelSize,
                       std::vector<double> *timestamps = nullptr);

void executeRemoveNaN(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                      std::vector<double> *timestamps = nullptr);

} // namespace internal
} // namespace lidar_utils