#include "internal/filter_impl.hpp"
#include "lidar_utils/types.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <set>
#include <string>
#include <vector>

namespace lidar_utils {
namespace internal {

void executeCrop(CloudType::Ptr &cloud, const Eigen::Vector3f &minBound,
                 const Eigen::Vector3f &maxBound,
                 std::vector<double> *timestamps) {
  if (!cloud || cloud->empty())
    return;

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  filtered_cloud->reserve(cloud->size());

  std::vector<double> filtered_timestamps;
  bool has_time =
      (timestamps != nullptr && timestamps->size() == cloud->size());
  if (has_time) {
    filtered_timestamps.reserve(timestamps->size());
  }

  for (size_t i = 0; i < cloud->size(); ++i) {
    const auto &pt = cloud->points[i];

    // Check outer bound (must be inside [-maxBound, maxBound])
    bool inside_outer = (pt.x >= -maxBound.x() && pt.x <= maxBound.x() &&
                         pt.y >= -maxBound.y() && pt.y <= maxBound.y() &&
                         pt.z >= -maxBound.z() && pt.z <= maxBound.z());

    // Check inner bound (must be inside [-minBound, minBound])
    bool inside_inner = (pt.x >= -minBound.x() && pt.x <= minBound.x() &&
                         pt.y >= -minBound.y() && pt.y <= minBound.y() &&
                         pt.z >= -minBound.z() && pt.z <= minBound.z());

    // Keep point if it is inside the outer box but NOT inside the inner box
    if (inside_outer && !inside_inner) {
      filtered_cloud->push_back(pt);
      if (has_time) {
        filtered_timestamps.push_back((*timestamps)[i]);
      }
    }
  }

  *cloud = *filtered_cloud;
  if (has_time) {
    *timestamps = std::move(filtered_timestamps);
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;
}

void executeDenoise(CloudType::Ptr &cloud, float radius, float epsilon) {
  if (!cloud || cloud->empty())
    return;

  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(cloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_copy(
      new pcl::PointCloud<pcl::PointXYZI>(*cloud));

#pragma omp parallel for
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    if (kdtree.radiusSearch(cloud->points[i], radius, pointIdxRadiusSearch,
                            pointRadiusSquaredDistance) > 0) {
      if (pointIdxRadiusSearch.size() < 3)
        continue;

      // Calculate mean
      Eigen::Vector3f mean = Eigen::Vector3f::Zero();
      for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
        mean += cloud->points[pointIdxRadiusSearch[j]].getVector3fMap();
      }
      mean /= static_cast<float>(pointIdxRadiusSearch.size());

      // Calculate covariance
      Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
      for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
        Eigen::Vector3f diff =
            cloud->points[pointIdxRadiusSearch[j]].getVector3fMap() - mean;
        cov += diff * diff.transpose();
      }
      cov /= static_cast<float>(pointIdxRadiusSearch.size() -
                                1); // Sample covariance

      // Calculate A and b
      Eigen::Matrix3f e =
          (cov + epsilon * Eigen::Matrix3f::Identity()).inverse();
      Eigen::Matrix3f A = cov * e;
      Eigen::Vector3f b = mean - A * mean;

      // Update point
      Eigen::Vector3f new_pt = A * cloud->points[i].getVector3fMap() + b;
      cloud_copy->points[i].x = new_pt.x();
      cloud_copy->points[i].y = new_pt.y();
      cloud_copy->points[i].z = new_pt.z();
    }
  }

  *cloud = *cloud_copy;
}

void executeRemoveArtifact(CloudType::Ptr &cloud,
                           std::vector<double> *timestamps) {
  if (!cloud || cloud->empty())
    return;

  const float CORE_INTENSITY_THRESHOLD = 150.0f;
  const size_t MIN_SIGN_POINTS = 50;
  const float NORMAL_RADIUS = 1.0f;
  const float CLUSTER_TOLERANCE = 0.5f;
  const int MIN_CLUSTER_SIZE = 10;
  const float PLANE_DISTANCE_THRESH = 2.2f;
  const float GHOST_DETECT_RADIUS = 10.0f;
  const float NORMAL_SIMILARITY_THRESH = 0.8f;
  const float GHOST_CUTOFF_MAX = 5.0f;
  const float PROTECT_INTENSITY_MIN = 50.0f;

  size_t num_points = cloud->size();
  std::vector<size_t> high_idx;
  pcl::PointCloud<pcl::PointXYZI>::Ptr high_pcd(
      new pcl::PointCloud<pcl::PointXYZI>());

  for (size_t i = 0; i < num_points; ++i) {
    if (cloud->points[i].intensity > CORE_INTENSITY_THRESHOLD) {
      high_idx.push_back(i);
      high_pcd->push_back(cloud->points[i]);
    }
  }

  if (high_idx.size() < MIN_SIGN_POINTS) {
    // std::cout << "[Debug] Skip: Not enough high-intensity points (" <<
    // high_idx.size() << ")" << std::endl;
    return;
  }

  // Fit plane
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.1);
  seg.setInputCloud(high_pcd);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty())
    return;

  Eigen::Vector4f plane_model =
      Eigen::Vector4f(coefficients->values[0], coefficients->values[1],
                      coefficients->values[2], coefficients->values[3]);

  Eigen::Vector3f sign_center(0, 0, 0);
  for (int idx : inliers->indices) {
    sign_center += high_pcd->points[idx].getVector3fMap();
  }
  sign_center /= static_cast<float>(inliers->indices.size());

  // === OPTIMIZATION: Filter Candidates BEFORE Normal Estimation ===
  std::set<size_t> high_idx_set(high_idx.begin(), high_idx.end());
  pcl::IndicesPtr candidate_indices(new std::vector<int>());
  Eigen::Vector3f sign_normal(plane_model[0], plane_model[1], plane_model[2]);

  for (size_t i = 0; i < num_points; ++i) {
    Eigen::Vector3f pt = cloud->points[i].getVector3fMap();
    float dist_p = std::abs(sign_normal.dot(pt) + plane_model[3]);
    float dist_c = (pt - sign_center).norm();

    if (dist_p < PLANE_DISTANCE_THRESH && dist_c < GHOST_DETECT_RADIUS) {
      candidate_indices->push_back(i);
    }
  }

  // If no candidates, exit early!
  if (candidate_indices->empty())
    return;

  // Calculate normals ONLY for the candidates (takes < 5ms instead of 1800ms)
  pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> ne;
  ne.setNumberOfThreads(omp_get_max_threads());
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZI>());
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);
  ne.setIndices(candidate_indices);
  ne.setKSearch(20); // ne.setRadiusSearch(NORMAL_RADIUS);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
      new pcl::PointCloud<pcl::Normal>());
  ne.compute(*cloud_normals);

  // Filter and statistics
  std::vector<size_t> ghost_indices;

  for (size_t idx = 0; idx < candidate_indices->size(); ++idx) {
    size_t i = (*candidate_indices)[idx];
    float intensity = cloud->points[i].intensity;

    bool m_intensity = intensity <= GHOST_CUTOFF_MAX;
    bool m_protected = intensity >= PROTECT_INTENSITY_MIN;
    bool m_not_main = (high_idx_set.find(i) == high_idx_set.end());

    if (!m_intensity || m_protected || !m_not_main)
      continue;

    Eigen::Vector3f pt = cloud->points[i].getVector3fMap();
    Eigen::Vector3f view = -pt;
    if (view.norm() > 0)
      view.normalize();

    // cloud_normals has the exact same size as candidate_indices
    float cos_sim =
        std::abs(cloud_normals->points[idx].getNormalVector3fMap().dot(view));
    bool m_normal = cos_sim > NORMAL_SIMILARITY_THRESH;

    if (m_normal) {
      ghost_indices.push_back(i);
    }
  }

  // std::cout << "[Debug] 平面過濾統計 (動態視線):" << std::endl;
  // std::cout << "  > 1. 符合共面距離 點數: " << cnt_plane << std::endl;
  // std::cout << "  > 2. 同時符合搜尋半徑 點數: " << cnt_radius << std::endl;
  // std::cout << "  > 3. 符合動態視線相似度 (" << NORMAL_SIMILARITY_THRESH
  //           << ") 點數: " << cnt_normal << std::endl;
  // std::cout << "  > 最終判定為鬼影並刪除的點數: " << ghost_indices.size()
  //           << " (已保護強度 >= " << PROTECT_INTENSITY_MIN << " 的點)"
  //           << std::endl;

  // Delete ghost points
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  std::vector<double> filtered_timestamps;
  std::set<size_t> ghost_set(ghost_indices.begin(), ghost_indices.end());

  for (size_t i = 0; i < num_points; ++i) {
    if (ghost_set.find(i) == ghost_set.end()) {
      filtered_cloud->push_back(cloud->points[i]);
      if (timestamps && timestamps->size() > i) {
        filtered_timestamps.push_back((*timestamps)[i]);
      }
    }
  }

  *cloud = *filtered_cloud;
  if (timestamps) {
    *timestamps = std::move(filtered_timestamps);
  }
}

void executeDownsample(CloudType::Ptr &cloud, float voxelSize,
                       std::vector<double> *timestamps) {
  if (!cloud || cloud->empty())
    return;

  if (!timestamps) {
    pcl::VoxelGrid<pcl::PointXYZI> ds;
    ds.setLeafSize(voxelSize, voxelSize, voxelSize);
    ds.setInputCloud(cloud);
    ds.filter(*cloud);
    return;
  }

  // Pack into PointXYZIT so VoxelGrid can average the timestamps!
  pcl::PointCloud<lidar_utils::PointXYZIT>::Ptr temp_cloud(
      new pcl::PointCloud<lidar_utils::PointXYZIT>());
  temp_cloud->reserve(cloud->size());
  for (size_t i = 0; i < cloud->size(); ++i) {
    lidar_utils::PointXYZIT pt;
    pt.x = cloud->points[i].x;
    pt.y = cloud->points[i].y;
    pt.z = cloud->points[i].z;
    pt.intensity = cloud->points[i].intensity;
    if (timestamps->size() > i) {
      pt.time = (*timestamps)[i];
    } else {
      pt.time = 0.0;
    }
    temp_cloud->push_back(pt);
  }

  pcl::VoxelGrid<lidar_utils::PointXYZIT> ds;
  ds.setLeafSize(voxelSize, voxelSize, voxelSize);
  ds.setInputCloud(temp_cloud);
  ds.filter(*temp_cloud);

  // Unpack back into XYZI and timestamps array
  cloud->clear();
  timestamps->clear();
  cloud->reserve(temp_cloud->size());
  timestamps->reserve(temp_cloud->size());

  for (const auto &pt : temp_cloud->points) {
    pcl::PointXYZI p_xyzi;
    p_xyzi.x = pt.x;
    p_xyzi.y = pt.y;
    p_xyzi.z = pt.z;
    p_xyzi.intensity = pt.intensity;
    cloud->push_back(p_xyzi);
    timestamps->push_back(pt.time);
  }
}

void executeRemoveNaN(CloudType::Ptr &cloud, std::vector<double> *timestamps) {
  if (!cloud || cloud->empty())
    return;

  if (!timestamps) {
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    return;
  }

  std::vector<int> indices;
  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::removeNaNFromPointCloud(*cloud, *temp_cloud, indices);

  std::vector<double> filtered_timestamps;
  filtered_timestamps.reserve(indices.size());

  for (int idx : indices) {
    if (timestamps->size() > static_cast<size_t>(idx)) {
      filtered_timestamps.push_back((*timestamps)[idx]);
    }
  }

  *cloud = *temp_cloud;
  *timestamps = std::move(filtered_timestamps);
}

} // namespace internal
} // namespace lidar_utils