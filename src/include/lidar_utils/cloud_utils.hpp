#pragma once
#include "types.hpp"
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

namespace lidar_utils {

class CloudUtils {
public:

  /**
   * @brief 6. mergeCloud: Combines other_cloud into base_cloud (In-place)
   */
  static void mergeCloud(CloudType::Ptr &base_cloud,
                         const CloudType::ConstPtr &other_cloud,
                         float voxelSize);

  static void eopCalib(const Eigen::MatrixXd &la, const Eigen::MatrixXd &bs,
                       const Eigen::Vector3d &laCalib,
                       const Eigen::Vector3d &bsCalib);

  static Eigen::Matrix4d getExtrinsics(const std::string &type,
                                       const Eigen::Vector3d &trans,
                                       const Eigen::Vector3d &rot);

  /**
   * @brief 3. cropCloud: Cuboid geometric cropping (In-place)
   */
  static void cropCloud(CloudType::Ptr &cloud, const Eigen::Vector3f &minBound,
                        const Eigen::Vector3f &maxBound);

  static void cropCloud(CloudType::Ptr &cloud, std::vector<double> &timestamps,
                        const Eigen::Vector3f &minBound,
                        const Eigen::Vector3f &maxBound);

  /**
   * @brief 4. denoiseCloud: Guided Filter for denoising (In-place)
   */
  static void denoiseCloud(CloudType::Ptr &cloud, float radius, float epsilon);

  /**
   * @brief 7. removeArtifact: Removes ghost artifacts (In-place)
   */
  static void removeArtifactCloud(CloudType::Ptr &cloud);

  static void removeArtifactCloud(CloudType::Ptr &cloud,
                                  std::vector<double> &timestamps);

  /**
   * @brief 5. downsampleCloud: Voxel grid downsampling (In-place)
   */
  static void downsampleCloud(CloudType::Ptr &cloud, float voxelSize);

  static void downsampleCloud(CloudType::Ptr &cloud,
                              std::vector<double> &timestamps, float voxelSize);

  static void removeNaNCloud(CloudType::Ptr &cloud);

  static void removeNaNCloud(CloudType::Ptr &cloud,
                             std::vector<double> &timestamps);

  static void readContent(const std::string &path,
                          std::vector<LidarContent> &file_list);

  static void readFile(CloudType::Ptr &cloud, std::string &filepath,
                       FileFormat format);

  static void readFile(CloudType::Ptr &cloud, std::vector<double> &timestamps,
                       std::string &filepath, FileFormat format);

  static void saveFile(CloudType::Ptr &cloud, std::string &filepath,
                       FileFormat format);

  /**
   * @brief 2. motionCompensateAndDG: Compensates distortion and transforms to
   * world frame (In-place)
   * @note Changed to modify 'cloud' directly to match the toolbox style.
   */

  static void directGeoreference(CloudType::Ptr &cloud, Eigen::Matrix4f &trans);

  static void motionCompensateAndDG(CloudType::Ptr &cloud,
                                    const std::vector<double> &timestamps,
                                    const Eigen::VectorXd &posCurr,
                                    const Eigen::VectorXd &posNext,
                                    bool motionEnable);

  static void scanToMapMatching(CloudType::Ptr &cloud, CloudType::Ptr &map,
                                Eigen::Matrix4f &in_transform,
                                Eigen::Matrix4f &out_transform,
                                float max_correspondence_distance,
                                float voxel_size, float score_threshold);

  static void scanToScanMatching(CloudType::Ptr &cloud,
                                 CloudType::Ptr &localmap,
                                 Eigen::Matrix4f &in_transform,
                                 Eigen::Matrix4f &out_transform,
                                 float max_correspondence_distance,
                                 float voxel_size, float score_threshold);
};
} // namespace lidar_utils