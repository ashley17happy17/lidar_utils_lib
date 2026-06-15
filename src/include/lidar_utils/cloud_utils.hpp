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
   * @brief mergeCloud: Combines other_cloud into base_cloud
   */
  static void mergeCloud(CloudType::Ptr &base_cloud,
                         const CloudType::ConstPtr &other_cloud,
                         float voxelSize);

  /**
   * @brief eopCalib: Calculates EOP
   */
  static Eigen::Matrix4d
  eopCalib(const std::vector<Eigen::Matrix4d> &gps_relative_motions,
           const std::vector<Eigen::Matrix4d> &lidar_relative_motions);

  /**
   * @brief transformEOP: Transforms EOP
   */
  static Eigen::Matrix4d transformEOP(const Eigen::Matrix4d &eop,
                                      const Eigen::Vector3d &out_la,
                                      const Eigen::Vector3d &out_bs);

  /**
   * @brief printEOP: Prints EOP
   */
  static void printEOP(const Eigen::Matrix4d &eop,
                       const std::string &sensor_name = "LiDAR");

  /**
   * @brief getExtrinsics: Gets extrinsic
   */
  static Eigen::Matrix4d getExtrinsics(const std::string &type,
                                       const Eigen::Vector3d &trans,
                                       const Eigen::Vector3d &rot);

  /**
   * @brief cropCloud: Cuboid geometric cropping
   */
  static void cropCloud(CloudType::Ptr &cloud, const Eigen::Vector3f &minBound,
                        const Eigen::Vector3f &maxBound);

  static void cropCloud(CloudType::Ptr &cloud, std::vector<double> &timestamps,
                        const Eigen::Vector3f &minBound,
                        const Eigen::Vector3f &maxBound);

  /**
   * @brief denoiseCloud: Guided Filter for denoising
   */
  static void denoiseCloud(CloudType::Ptr &cloud, float radius, float epsilon);

  /**
   * @brief removeArtifact: Removes ghost artifacts
   */
  static void removeArtifactCloud(CloudType::Ptr &cloud);

  static void removeArtifactCloud(CloudType::Ptr &cloud,
                                  std::vector<double> &timestamps);

  /**
   * @brief downsampleCloud: Voxel grid downsampling
   */
  static void downsampleCloud(CloudType::Ptr &cloud, float voxelSize);

  static void downsampleCloud(CloudType::Ptr &cloud,
                              std::vector<double> &timestamps, float voxelSize);

  /**
   * @brief removeNaNCloud: Removes NaN
   */
  static void removeNaNCloud(CloudType::Ptr &cloud);

  static void removeNaNCloud(CloudType::Ptr &cloud,
                             std::vector<double> &timestamps);

  /**
   * @brief readContent: Reads content
   */
  static void readContent(const std::string &path,
                          std::vector<LidarContent> &file_list);

  /**
   * @brief readFile: Reads file
   */
  static void readFile(CloudType::Ptr &cloud, std::string &filepath,
                       FileFormat format);

  static void readFile(CloudType::Ptr &cloud, std::vector<double> &timestamps,
                       std::string &filepath, FileFormat format);

  /**
   * @brief saveFile: Saves file
   */
  static void saveFile(CloudType::Ptr &cloud, std::string &filepath,
                       FileFormat format);

  /**
   * @brief directGeoreference: Direct georeference
   */
  static void directGeoreference(CloudType::Ptr &cloud, Eigen::Matrix4d &trans);

  /**
   * @brief motionCompensateAndDG: Compensates distortion and transforms to
   * world frame
   */
  static void motionCompensateAndDG(CloudType::Ptr &cloud,
                                    const std::vector<double> &timestamps,
                                    const Eigen::VectorXd &posCurr,
                                    const Eigen::VectorXd &posNext,
                                    bool motionEnable);

  /**
   * @brief GICP: GICP registration
   */
  static int GICP(CloudType::Ptr &cloud, CloudType::Ptr &map,
                  Eigen::Matrix4d &in_transform, Eigen::Matrix4d &out_transform,
                  const RegistrationConfig &config = RegistrationConfig());

  /**
   * @brief NDT: NDT registration
   */
  static int NDT(CloudType::Ptr &cloud, CloudType::Ptr &map,
                 Eigen::Matrix4d &in_transform, Eigen::Matrix4d &out_transform,
                 const RegistrationConfig &config = RegistrationConfig());
};
} // namespace lidar_utils