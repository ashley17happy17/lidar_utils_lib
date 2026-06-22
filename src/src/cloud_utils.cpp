#include "lidar_utils/cloud_utils.hpp"
#include "internal/common_impl.hpp"
#include "internal/eop_impl.hpp"
#include "internal/filter_impl.hpp"
#include "internal/io_impl.hpp"
#include "internal/registration_impl.hpp"
#include "internal/transform_impl.hpp"

namespace lidar_utils {

/***************************
 *  common_impl            *
 ***************************/
void CloudUtils::mergeCloud(CloudType::Ptr &base_cloud,
                            const CloudType::ConstPtr &other_cloud,
                            float voxelSize) {
  internal::executeMerge(base_cloud, other_cloud, voxelSize);
}

/***************************
 *  eop_impl               *
 ***************************/
Eigen::Matrix4d CloudUtils::eopCalibDynamic(
    const std::vector<Eigen::Matrix4d> &gps_relative_motions,
    const std::vector<Eigen::Matrix4d> &lidar_relative_motions) {
  return internal::executeEOPCalibDynamic(gps_relative_motions,
                                          lidar_relative_motions);
}

Eigen::Matrix4d CloudUtils::eopCalibStatic(
    const std::vector<Eigen::Matrix4d> &absolute_extrinsics) {
  return internal::executeEOPCalibStatic(absolute_extrinsics);
}

Eigen::Matrix4d CloudUtils::transformEOP(const Eigen::Matrix4d &eop,
                                         const Eigen::Vector3d &out_la,
                                         const Eigen::Vector3d &out_bs) {
  return internal::executeTransformEOP(eop, out_la, out_bs);
}

void CloudUtils::printEOP(const Eigen::Matrix4d &eop,
                          const std::string &sensor_name, DCMOrder order) {
  internal::executePrintEOP(eop, sensor_name, order);
}

Eigen::Matrix4d CloudUtils::getExtrinsics(SensorType type,
                                          const Eigen::Vector3d &trans,
                                          const Eigen::Vector3d &rot) {
  return internal::executeGetExtrinsics(type, trans, rot);
}

/***************************
 *  filter_impl            *
 ***************************/
void CloudUtils::cropCloud(CloudType::Ptr &cloud,
                           const Eigen::Vector3f &minBound,
                           const Eigen::Vector3f &maxBound) {
  internal::executeCrop(cloud, minBound, maxBound, nullptr);
}

void CloudUtils::cropCloud(CloudType::Ptr &cloud,
                           std::vector<double> &timestamps,
                           const Eigen::Vector3f &minBound,
                           const Eigen::Vector3f &maxBound) {
  internal::executeCrop(cloud, minBound, maxBound, &timestamps);
}

void CloudUtils::denoiseCloud(CloudType::Ptr &cloud, float radius,
                              float epsilon) {
  internal::executeDenoise(cloud, radius, epsilon);
}

void CloudUtils::removeArtifactCloud(CloudType::Ptr &cloud) {
  internal::executeRemoveArtifact(cloud, nullptr);
}

void CloudUtils::removeArtifactCloud(CloudType::Ptr &cloud,
                                     std::vector<double> &timestamps) {
  internal::executeRemoveArtifact(cloud, &timestamps);
}

void CloudUtils::downsampleCloud(CloudType::Ptr &cloud, float voxelSize) {
  internal::executeDownsample(cloud, voxelSize, nullptr);
}

void CloudUtils::downsampleCloud(CloudType::Ptr &cloud,
                                 std::vector<double> &timestamps,
                                 float voxelSize) {
  internal::executeDownsample(cloud, voxelSize, &timestamps);
}

void CloudUtils::removeNaNCloud(CloudType::Ptr &cloud) {
  internal::executeRemoveNaN(cloud, nullptr);
}

void CloudUtils::removeNaNCloud(CloudType::Ptr &cloud,
                                std::vector<double> &timestamps) {
  internal::executeRemoveNaN(cloud, &timestamps);
}

/***************************
 *  io_impl                *
 ***************************/
void CloudUtils::readContent(const std::string &path,
                             std::vector<LidarContent> &file_list) {
  internal::executeReadContent(path, file_list);
}

void CloudUtils::readFile(CloudType::Ptr &cloud, std::string &filepath,
                          FileFormat format) {
  internal::executeReadFile(cloud, filepath, format, nullptr);
}

void CloudUtils::readFile(CloudType::Ptr &cloud,
                          std::vector<double> &timestamps,
                          std::string &filepath, FileFormat format) {
  internal::executeReadFile(cloud, filepath, format, &timestamps);
}

void CloudUtils::saveFile(CloudType::Ptr &cloud, std::string &filepath,
                          FileFormat format) {
  internal::executeSaveFile(cloud, filepath, format);
}

/***************************
 *  transform_impl         *
 ***************************/

void CloudUtils::directGeoreference(CloudType::Ptr &inCloud,
                                    CloudType::Ptr &outCloud,
                                    Eigen::Matrix4d &trans) {
  internal::executeDirectGeoreference(inCloud, outCloud, trans);
}

void CloudUtils::motionCompensateAndDG(CloudType::Ptr &cloud,
                                       const std::vector<double> &timestamps,
                                       const Eigen::VectorXd &posCurr,
                                       const Eigen::VectorXd &posNext,
                                       bool motionEnable) {
  internal::executeMotionAndDG(cloud, timestamps, posCurr, posNext,
                               motionEnable);
}

/***************************
 *  registration_impl      *
 ***************************/
double CloudUtils::GICP(CloudType::Ptr &cloud, CloudType::Ptr &map,
                     Eigen::Matrix4d &in_transform,
                     Eigen::Matrix4d &out_transform,
                     const RegistrationConfig &config) {
  return internal::executeGICP(cloud, map, in_transform, out_transform, config);
}

double CloudUtils::NDT(CloudType::Ptr &cloud, CloudType::Ptr &map,
                    Eigen::Matrix4d &in_transform,
                    Eigen::Matrix4d &out_transform,
                    const RegistrationConfig &config) {
  return internal::executeNDT(cloud, map, in_transform, out_transform, config);
}

} // namespace lidar_utils