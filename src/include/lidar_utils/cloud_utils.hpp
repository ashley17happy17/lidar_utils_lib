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
   * @brief eopCalibDynamic: Dynamically calculates EOP without HDmap.
   */
  static Eigen::Matrix4d
  eopCalibDynamic(const std::vector<Eigen::Matrix4d> &gps_relative_motions,
                  const std::vector<Eigen::Matrix4d> &lidar_relative_motions);

  /**
   * @brief eopCalibStatic: Calculates EOP using Static Absolute Map-Based Least
   * Squares
   */
  static Eigen::Matrix4d
  eopCalibStatic(const std::vector<Eigen::Matrix4d> &absolute_extrinsics);

  /**
   * @brief transformEOP: Transforms EOP
   */
  static Eigen::Matrix4d transformEOP(const Eigen::Matrix4d &eop,
                                      const Eigen::Vector3d &out_la,
                                      const Eigen::Vector3d &out_bs);

  /**
   * @brief getRawExtrinsicsFromFLU: Converts FLU extrinsics back to RAW params.yaml format
   */
  static Eigen::Matrix4d getRawExtrinsicsFromFLU(SensorType type, 
                                                 const Eigen::Matrix4d &flu_ext);

  /**
   * @brief printEOP: Prints EOP
   */
  static void printEOP(const Eigen::Matrix4d &eop,
                       const std::string &sensor_name, DCMOrder order);

  /**
   * @brief getExtrinsics: Gets extrinsic
   */
  static Eigen::Matrix4d getExtrinsics(SensorType type,
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
   * @brief extractGround: Extracts ground and non-ground points using RANSAC
   */
  static void extractGround(const CloudType::Ptr &cloudIn,
                            CloudType::Ptr &groundCloud,
                            CloudType::Ptr &nonGroundCloud,
                            double distanceThreshold = 0.2,
                            int maxIterations = 100);

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
  static void directGeoreference(CloudType::Ptr &inCloud,
                                 CloudType::Ptr &outCloud,
                                 Eigen::Matrix4d &trans);

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
   * @brief motionCompensateAndDG (pluggable): deskews a scan (gyro rotation +
   * chosen translation source) AND georeferences it into the world frame in a
   * single call — the pluggable drop-in for the pose-interpolation overload
   * above. Pass the vehicle->world pose at scan start (e.g. gnss.getTransform());
   * the world->vehicle rotation that GNSS_TRANS needs is derived from it
   * internally, so no R is exposed and no separate directGeoreference is needed.
   * For IMU_ACC_TRANS, if @p v0_vehicle is left zero and @p gnss is provided, the
   * scan-start velocity seed is estimated from the GNSS trajectory. Set
   * @p motionEnable = false to skip deskew (georeference only).
   */
  static void motionCompensateAndDG(
      CloudType::Ptr &cloud, const std::vector<double> &timestamps,
      const std::vector<ImuSample> &imu, MotionMethod method,
      const Eigen::Matrix4d &T_vehicle_to_world,
      const std::vector<GnssSample> &gnss = std::vector<GnssSample>(),
      const std::vector<OdomSample> &odom = std::vector<OdomSample>(),
      const Eigen::Vector3d &v0_vehicle = Eigen::Vector3d::Zero(),
      bool motionEnable = true);

  /**
   * @brief wgs84ToEnu: Converts a WGS84 geodetic coordinate (lat/lon in
   * degrees, height in metres) to local ENU metres (East, North, Up) relative
   * to a reference origin (lat0/lon0/h0). Take the first GNSS fix as the origin,
   * then feed the returned vector into GnssSample{x=E, y=N, z=U} for GNSS_TRANS
   * motion compensation.
   */
  static Eigen::Vector3d wgs84ToEnu(double lat_deg, double lon_deg, double h,
                                    double lat0_deg, double lon0_deg, double h0);

  /**
   * @brief motionCompensate: Deskews a scan into the scan-start frame.
   * Rotation always comes from the IMU gyro; translation comes from the chosen
   * MotionMethod:
   *   - GNSS_TRANS   : GNSS position delta (needs @p gnss and, for world->vehicle
   *                    rotation, @p R_vehicle_from_world = vehicle-start orientation)
   *   - ODOM_TRANS   : integrated odometer velocity (needs @p odom, vehicle frame)
   *   - IMU_ACC_TRANS: double-integrated IMU accel (needs ImuSample accel +
   *                    @p v0_vehicle, the vehicle-frame velocity at scan start)
   * Gyro/accel must already be axis-aligned to the vehicle/LiDAR frame. Unused
   * source vectors may be left empty (defaulted).
   */
  static void
  motionCompensate(CloudType::Ptr &cloud, const std::vector<double> &timestamps,
                   const std::vector<ImuSample> &imu, MotionMethod method,
                   const std::vector<GnssSample> &gnss =
                       std::vector<GnssSample>(),
                   const std::vector<OdomSample> &odom =
                       std::vector<OdomSample>(),
                   const Eigen::Matrix3d &R_vehicle_from_world =
                       Eigen::Matrix3d::Identity(),
                   const Eigen::Vector3d &v0_vehicle =
                       Eigen::Vector3d::Zero());

  /**
   * @brief GICP: GICP registration
   */
  static double GICP(CloudType::Ptr &cloud, CloudType::Ptr &map,
                     Eigen::Matrix4d &in_transform,
                     Eigen::Matrix4d &out_transform,
                     const RegistrationConfig &config = RegistrationConfig());

  /**
   * @brief NDT: NDT registration
   */
  static double NDT(CloudType::Ptr &cloud, CloudType::Ptr &map,
                    Eigen::Matrix4d &in_transform,
                    Eigen::Matrix4d &out_transform,
                    const RegistrationConfig &config = RegistrationConfig());
};
} // namespace lidar_utils