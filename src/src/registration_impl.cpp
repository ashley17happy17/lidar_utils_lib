#include "internal/registration_impl.hpp"
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/search/kdtree.h>
#include <small_gicp/registration/registration_helper.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

namespace lidar_utils {
namespace internal {

double executeGICP(CloudType::Ptr &cloud, CloudType::Ptr &map,
                   Eigen::Matrix4d &in_transform,
                   Eigen::Matrix4d &out_transform,
                   const RegistrationConfig &config) {
  if (!cloud || cloud->empty() || !map || map->empty()) {
    std::cerr << "[ERROR] Registration Error: cloud is empty." << std::endl;
    return -1.0;
  }

  // 0. GICP Settings
  small_gicp::RegistrationSetting gicp_settings_;
  gicp_settings_.type = small_gicp::RegistrationSetting::GICP;
  gicp_settings_.max_correspondence_distance =
      config.max_correspondence_distance;
  gicp_settings_.max_iterations = config.max_iterations;
  gicp_settings_.rotation_eps = config.rotation_eps;
  gicp_settings_.translation_eps = config.translation_eps;

  if (config.num_threads > 0) {
    gicp_settings_.num_threads = config.num_threads;
  }

  auto t0 = std::chrono::steady_clock::now();

  // 1. Get HD Map Pointers (Mutex check)
  std::shared_ptr<small_gicp::PointCloud> target_gicp = toGicpCloud(map);
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> target_tree =
      std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(target_gicp);

  auto current_settings = gicp_settings_;
  small_gicp::estimate_covariances_omp(*target_gicp, *target_tree, 20,
                                       current_settings.num_threads);

  // 2. Source Cloud Conversion
  auto source_gicp = toGicpCloud(cloud);
  auto source_tree =
      std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(source_gicp);
  small_gicp::estimate_covariances_omp(*source_gicp, *source_tree, 20,
                                       current_settings.num_threads);

  /*
  // --- DEBUG: Save clouds to check overlap ---
  {
    CloudType::Ptr source_pcl_debug(new CloudType());
    // Transform current cloud by initial guess to see if it's close to map
    pcl::transformPointCloud(*current_cloud, *source_pcl_debug, out_transform);

    pcl::io::savePCDFileASCII(
        "/root/catkin_ws/src/lidar_scan_match_c/scan_to_map_source.pcd",
        *source_pcl_debug);

    // Convert small_gicp target back to PCL for saving
    CloudType::Ptr target_pcl_debug(new CloudType());

    // 1. 改用 resize 預先劃分好確定的記憶體空間，徹底避開 push_back的長度檢查
    target_pcl_debug->points.resize(target_gicp->points.size());

// 2. 啟動 OpenMP 讓所有 CPU 核心平行轉換這數十萬個點
#pragma omp parallel for num_threads(10) schedule(dynamic)
    for (size_t i = 0; i < target_gicp->points.size(); ++i) {
      const auto &pt = target_gicp->points[i];
      auto &p = target_pcl_debug->points[i];
      p.x = pt.x();
      p.y = pt.y();
      p.z = pt.z();
      p.intensity = 1.0f;
    }

    // 手動補上 PCL 必要的 metadata
    target_pcl_debug->width = target_pcl_debug->points.size();
    target_pcl_debug->height = 1;
    target_pcl_debug->is_dense = true;
    pcl::io::savePCDFileBinary(
        "/root/catkin_ws/src/lidar_scan_match_c/scan_to_map_target.pcd",
        *target_pcl_debug);
  }
  // -------------------------------------------
  */

  // Pass the prebuilt target KdTree to GICP alignment
  Eigen::Isometry3d init_guess(in_transform.cast<double>());
  auto result = small_gicp::align(*target_gicp, *source_gicp, *target_tree,
                                  init_guess, current_settings);

  /*
  std::cout << "--- T_target_source ---" << std::endl
            << result.T_target_source.matrix() << std::endl;
  std::cout << "converged:" << result.converged << std::endl;
  std::cout << "error:" << result.error << std::endl;
  std::cout << "iterations:" << result.iterations << std::endl;
  std::cout << "num_inliers:" << result.num_inliers << std::endl;
  std::cout << "--- H ---" << std::endl << result.H << std::endl;
  std::cout << "--- b ---" << std::endl << result.b.transpose() << std::endl;
  */

  // == 無論成功與否，都強制提取最新的轉換矩陣 ==
  Eigen::Matrix4d T_res = result.T_target_source.matrix();
  out_transform = T_res;
  double out_fitness_score =
      (result.num_inliers > 0) ? (result.error / result.num_inliers) : 1e6;

  auto t1 = std::chrono::steady_clock::now();
  /*
// === DEBUG: 永遠存下 光達對地圖(HD Map) 的匹配結果 ===
{
  CloudType::Ptr matched_pcl_debug(new CloudType());
  pcl::transformPointCloud(*current_cloud, *matched_pcl_debug, out_transform);
  pcl::io::savePCDFileBinary(
      "/root/catkin_ws/src/lidar_scan_match_c/scan_to_map_source_matched.pcd",
      *matched_pcl_debug);
}
// -----------------------------------
*/

  double d_icp = std::chrono::duration<double, std::milli>(t1 - t0).count();
  std::cout << "[INFO] GICP Registration time: " << std::setprecision(13)
            << d_icp << "ms" << std::endl;

  if (result.num_inliers > 100 && out_fitness_score < config.score_threshold) {
    return out_fitness_score;
  } else {
    // std::cerr
    //     << "[WARN] Registration WARNING: failed to align, out_fitness_score:
    //     "
    //     << out_fitness_score << ", num_inliers: " << result.num_inliers
    //     << std::endl;
    return -1.0;
  }
}

double executeNDT(CloudType::Ptr &cloud, CloudType::Ptr &map,
                  Eigen::Matrix4d &in_transform, Eigen::Matrix4d &out_transform,
                  const RegistrationConfig &config) {
  pcl::NormalDistributionsTransform<PointType, PointType> ndt;

  ndt.setTransformationEpsilon(config.ndt_transformation_epsilon);
  ndt.setStepSize(config.ndt_step_size);
  ndt.setResolution(config.ndt_resolution);
  ndt.setMaximumIterations(config.max_iterations);
  ndt.setOulierRatio(config.ndt_oulier_ratio);

  ndt.setInputTarget(map);
  ndt.setInputSource(cloud);

  CloudType::Ptr aligned_cloud(new CloudType());
  ndt.align(*aligned_cloud, in_transform.cast<float>());

  out_transform = ndt.getFinalTransformation().cast<double>();

  if (ndt.hasConverged() && ndt.getFitnessScore() < config.score_threshold) {
    return ndt.getFitnessScore();
  } else {
    // std::cerr
    //     << "[WARN] Registration WARNING: failed to align, out_fitness_score:
    //     "
    //     << ndt.getFitnessScore() << ", hasConverged: " << ndt.hasConverged()
    //     << std::endl;
    return -1.0;
  }
}

} // namespace internal
} // namespace lidar_utils