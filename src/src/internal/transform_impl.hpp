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
// Motion compensation (deskew) from raw IMU gyro + GNSS position.
//
// Reference: LIO-SAM imageProjection.cpp
//   integrateImuGyro       <-> imuDeskewInfo()  (integrate ang. vel. -> rot)
//   transformGnssToVehicle <-> odomDeskewInfo() (relative motion -> trans)
//   executeMotionComensation <-> deskewPoint()  (per-point transform)
// ---------------------------------------------------------------------------

// ImuSample and GnssSample (the public inputs) live in lidar_utils/types.hpp.

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

// Relative translation of the vehicle across one scan, expressed in the
// vehicle (scan-start) frame. Mirrors odomIncreX/Y/Z in odomDeskewInfo().
struct GnssMotionInfo {
  Eigen::Vector3d transIncre = Eigen::Vector3d::Zero();
  double timeScanCur = 0.0;
  double timeScanEnd = 0.0;
  bool available = false;
};

// Integrate the gyro over the scan window into per-sample cumulative rotation.
ImuRotationInfo integrateImuGyro(const std::vector<ImuSample> &imu,
                                 double timeScanCur, double timeScanEnd);

// Convert GNSS positions into the scan's relative translation, rotated from
// the world frame into the vehicle (scan-start) frame by R_vehicle_from_world.
// Pass the vehicle-start orientation (e.g. from IMU heading); identity means
// the GNSS positions are already expressed in the vehicle frame.
GnssMotionInfo transformGnssToVehicle(
    const std::vector<GnssSample> &gnss, double timeScanCur, double timeScanEnd,
    const Eigen::Matrix3d &R_vehicle_from_world = Eigen::Matrix3d::Identity());

// Deskew each point into the scan-start frame using gyro-derived rotation and
// GNSS-derived translation, interpolated to the point's own timestamp.
void executeMotionComensation(
    CloudType::Ptr &cloud, const std::vector<double> &timestamps,
    const std::vector<ImuSample> &imu, const std::vector<GnssSample> &gnss,
    const Eigen::Matrix3d &R_vehicle_from_world = Eigen::Matrix3d::Identity());

} // namespace internal
} // namespace lidar_utils