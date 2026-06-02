#include "internal/filter_impl.hpp"
#include "lidar_utils/types.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <pcl/features/normal_3d.h>
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

void executeCrop(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                 const Eigen::Vector3f &minBound,
                 const Eigen::Vector3f &maxBound,
                 std::vector<double> *timestamps) {
  if (!cloud || cloud->empty())
    return;

  pcl::CropBox<pcl::PointXYZI> crop_box;

  Eigen::Vector4f min_pt(minBound.x(), minBound.y(), minBound.z(), 1.0f);
  Eigen::Vector4f max_pt(maxBound.x(), maxBound.y(), maxBound.z(), 1.0f);
  crop_box.setMin(min_pt);
  crop_box.setMax(max_pt);

  crop_box.setInputCloud(cloud);
  crop_box.setNegative(true);

  std::vector<int> indices;
  crop_box.filter(indices);

  // Synchronize both cloud and timestamps
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  std::vector<double> filtered_timestamps;
  filtered_cloud->reserve(indices.size());

  if (timestamps) {
    filtered_timestamps.reserve(indices.size());
  }

  for (int idx : indices) {
    filtered_cloud->push_back(cloud->points[idx]);
    if (timestamps && timestamps->size() > static_cast<size_t>(idx)) {
      filtered_timestamps.push_back((*timestamps)[idx]);
    }
  }

  *cloud = *filtered_cloud;
  if (timestamps) {
    *timestamps = std::move(filtered_timestamps);
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;
}

void executeDenoise(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float radius,
                    float epsilon) {
  if (!cloud || cloud->empty())
    return;

  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(cloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_copy(
      new pcl::PointCloud<pcl::PointXYZI>(*cloud));

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

void executeRemoveArtifact(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
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
    std::cout << "[Debug] 跳過：高強度點數不足 (" << high_idx.size() << ")"
              << std::endl;
    return;
  }

  // Calculate normals for the whole cloud
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZI>());
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);
  ne.setRadiusSearch(NORMAL_RADIUS);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
      new pcl::PointCloud<pcl::Normal>());
  ne.compute(*cloud_normals);

  // Cluster high intensity points
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree_high(
      new pcl::search::KdTree<pcl::PointXYZI>());
  tree_high->setInputCloud(high_pcd);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(CLUSTER_TOLERANCE);
  ec.setMinClusterSize(MIN_CLUSTER_SIZE);
  ec.setSearchMethod(tree_high);
  ec.setInputCloud(high_pcd);
  ec.extract(cluster_indices);

  if (cluster_indices.empty())
    return;

  // Find largest cluster
  int max_cluster_idx = 0;
  size_t max_size = 0;
  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    if (cluster_indices[i].indices.size() > max_size) {
      max_size = cluster_indices[i].indices.size();
      max_cluster_idx = i;
    }
  }

  std::set<size_t> main_sign_set;
  pcl::PointCloud<pcl::PointXYZI>::Ptr main_sign_pcd(
      new pcl::PointCloud<pcl::PointXYZI>());
  for (int idx : cluster_indices[max_cluster_idx].indices) {
    main_sign_set.insert(high_idx[idx]);
    main_sign_pcd->push_back(high_pcd->points[idx]);
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
  seg.setInputCloud(main_sign_pcd);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty())
    return;

  Eigen::Vector4f plane_model =
      Eigen::Vector4f(coefficients->values[0], coefficients->values[1],
                      coefficients->values[2], coefficients->values[3]);

  Eigen::Vector3f sign_center(0, 0, 0);
  for (int idx : inliers->indices) {
    sign_center += main_sign_pcd->points[idx].getVector3fMap();
  }
  sign_center /= static_cast<float>(inliers->indices.size());

  // std::cout << "[Debug] 號誌中心: " << sign_center.x() << " " <<
  // sign_center.y()
  //           << " " << sign_center.z() << std::endl;
  // std::cout << "[Debug] 擬合平面: " << plane_model[0] << "x + "
  //           << plane_model[1] << "y + " << plane_model[2] << "z + "
  //           << plane_model[3] << " = 0" << std::endl;

  // Filter and statistics
  std::vector<size_t> ghost_indices;
  Eigen::Vector3f sign_normal(plane_model[0], plane_model[1], plane_model[2]);

  int cnt_plane = 0, cnt_radius = 0, cnt_normal = 0;

  for (size_t i = 0; i < num_points; ++i) {
    Eigen::Vector3f pt = cloud->points[i].getVector3fMap();
    float dist_p = std::abs(sign_normal.dot(pt) + plane_model[3]);
    float dist_c = (pt - sign_center).norm();

    bool m_plane = dist_p < PLANE_DISTANCE_THRESH;
    bool m_radius = dist_c < GHOST_DETECT_RADIUS;

    if (m_plane)
      cnt_plane++;
    if (m_plane && m_radius)
      cnt_radius++;

    Eigen::Vector3f view = -pt;
    if (view.norm() > 0)
      view.normalize();
    float cos_sim =
        std::abs(cloud_normals->points[i].getNormalVector3fMap().dot(view));
    bool m_normal = cos_sim > NORMAL_SIMILARITY_THRESH;

    if (m_plane && m_radius && m_normal)
      cnt_normal++;

    float intensity = cloud->points[i].intensity;
    bool m_intensity = intensity <= GHOST_CUTOFF_MAX;
    bool m_protected = intensity >= PROTECT_INTENSITY_MIN;
    bool m_not_main = (main_sign_set.find(i) == main_sign_set.end());

    if (m_plane && m_radius && m_normal && m_intensity && (!m_protected) &&
        m_not_main) {
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

void executeDownsample(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                       float voxelSize, std::vector<double> *timestamps) {
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

void executeRemoveNaN(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                      std::vector<double> *timestamps) {
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