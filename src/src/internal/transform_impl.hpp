#pragma once
#include "lidar_utils/types.hpp"
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace lidar_utils {
namespace internal {

void executeDirectGeoreference(CloudType::Ptr &inCloud,
                               CloudType::Ptr &outCloud,
                               Eigen::Matrix4d &trans);

void executeMotionAndDG(CloudType::Ptr &cloud,
                        const std::vector<double> &timestamps,
                        const Eigen::VectorXd &posCurr,
                        const Eigen::VectorXd &posNext, bool motionEnable);

// ---------------------------------------------------------------------------
// Motion compensation (deskew): gyro rotation + pluggable translation source.
//
// Reference: LIO-SAM imageProjection.cpp
//   integrateImuGyro         <-> imuDeskewInfo()  (integrate ang. vel. -> rot)
//   buildTranslation*        <-> odomDeskewInfo() (per-source relative motion)
//   executeMotionComensation <-> deskewPoint()    (per-point transform)
//
// Rotation is always the integrated IMU gyro. Translation comes from one of
// three sources (MotionMethod): GNSS position, odometer velocity, or IMU accel.
//
// ImuSample / GnssSample / OdomSample / MotionMethod live in
// lidar_utils/types.hpp.
// ---------------------------------------------------------------------------

// Cumulative rotation of the LiDAR across one scan, obtained by integrating
// the gyro. Angles are relative to the first in-window sample (defined as 0).
// Mirrors imuTime / imuRotX/Y/Z in imuDeskewInfo().
struct ImuRotationInfo {
  std::vector<double> time; // absolute sample time [s]
  std::vector<double> rotX; // integrated rotation about X [rad]
  std::vector<double> rotY;
  std::vector<double> rotZ;
  bool available = false;
};

// Cumulative translation of the vehicle across one scan, expressed in the
// vehicle (scan-start) frame as a sampled trajectory (position at each sample
// time, anchored to 0 at the first in-window sample). Any translation source
// (GNSS / odom / IMU accel) produces one of these; findTranslation()
// interpolates it per point. Mirrors odomIncreX/Y/Z in odomDeskewInfo() but
// keeps the full intra-scan profile instead of a single increment.
struct TranslationInfo {
  std::vector<double> time;              // absolute sample time [s]
  std::vector<Eigen::Vector3d> pos;      // cumulative translation [m]
  bool available = false;
};

// Integrate the gyro over the scan window into per-sample cumulative rotation.
ImuRotationInfo integrateImuGyro(const std::vector<ImuSample> &imu,
                                 double timeScanCur, double timeScanEnd);

// Translation from GNSS positions: relative motion over the scan, rotated from
// the world frame into the vehicle frame by R_vehicle_from_world (pass the
// vehicle-start orientation; identity if GNSS is already vehicle-framed).
TranslationInfo buildTranslationGnss(
    const std::vector<GnssSample> &gnss, double timeScanCur, double timeScanEnd,
    const Eigen::Matrix3d &R_vehicle_from_world = Eigen::Matrix3d::Identity());

// Translation from an odometer: integrate the vehicle-frame velocity over the
// scan window (trapezoidal). Already in the vehicle frame, so no rotation.
TranslationInfo buildTranslationOdom(const std::vector<OdomSample> &odom,
                                     double timeScanCur, double timeScanEnd);

// Translation from the IMU accelerometer: double-integrate the gravity-removed
// vehicle-frame acceleration, seeded with v0_vehicle (velocity at scan start,
// which the accel cannot provide on its own).
TranslationInfo buildTranslationImuAcc(
    const std::vector<ImuSample> &imu, double timeScanCur, double timeScanEnd,
    const Eigen::Vector3d &v0_vehicle = Eigen::Vector3d::Zero());

// Deskew each point into the scan-start frame using gyro-derived rotation and
// the chosen translation source, interpolated to the point's own timestamp.
// Unused source vectors may be left empty. v0_vehicle seeds IMU_ACC_TRANS.
void executeMotionComensation(
    CloudType::Ptr &cloud, const std::vector<double> &timestamps,
    const std::vector<ImuSample> &imu, MotionMethod method,
    const std::vector<GnssSample> &gnss = std::vector<GnssSample>(),
    const std::vector<OdomSample> &odom = std::vector<OdomSample>(),
    const Eigen::Matrix3d &R_vehicle_from_world = Eigen::Matrix3d::Identity(),
    const Eigen::Vector3d &v0_vehicle = Eigen::Vector3d::Zero());

// Convert a WGS84 geodetic coordinate (lat/lon [deg], height [m]) to local ENU
// metres (East, North, Up) relative to a reference origin (lat0/lon0/h0 [deg,
// deg, m]), via an ECEF difference and rotation into the origin's tangent plane.
Eigen::Vector3d executeWgs84ToEnu(double lat_deg, double lon_deg, double h,
                                  double lat0_deg, double lon0_deg, double h0);

// Finite-difference the GNSS trajectory to estimate world-frame velocity [m/s]
// at time t (used to seed IMU_ACC_TRANS). Returns zero if there is not enough
// data.
Eigen::Vector3d estimateGnssVelocity(const std::vector<GnssSample> &gnss,
                                     double t);

} // namespace internal
} // namespace lidar_utils