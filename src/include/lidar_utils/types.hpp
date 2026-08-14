#pragma once

#define PCL_NO_PRECOMPILE // Required for custom point types
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

namespace lidar_utils {

using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<PointType>;
using ConstCloudType = CloudType::ConstPtr;

// 支援的光達型號選單
enum class SensorType {
  OUSTER_OS1_128,
  OUSTER_OS1_32,
  VELODYNE_VLP16,
  VELODYNE_VLS128
};

enum class FileFormat { PCD_BINARY, PCD_ASCII, LAS };

enum class CoordinateType { WGS84, TWD97 };

enum class DCMOrder { ZYX, XYZ };

struct LidarContent {
  double timestamp;
  std::string filename;
};

// Motion-compensation (deskew) method. Rotation always comes from the IMU
// gyro; only the TRANSLATION source differs between methods.
enum class MotionMethod {
  GNSS_TRANS,    // translation from GNSS position delta   + gyro rotation
  ODOM_TRANS,    // translation from odometer velocity     + gyro rotation
  IMU_ACC_TRANS  // translation from IMU accel (double-integrated, seeded) + gyro
};

// A single IMU sample for motion compensation. The gyro (and accelerometer,
// when used) is assumed to be ALREADY rotated into the vehicle/LiDAR frame with
// matching axes, so no axis correction is applied downstream. Angular velocity
// is location independent, so no lever arm is needed for rotation.
struct ImuSample {
  double time = 0.0;  // absolute timestamp [s]
  double gyroX = 0.0; // angular velocity about X [rad/s]
  double gyroY = 0.0; // angular velocity about Y [rad/s]
  double gyroZ = 0.0; // angular velocity about Z [rad/s]
  // Linear ("free") acceleration, gravity-removed, in the vehicle frame
  // [m/s^2]. Only used by MotionMethod::IMU_ACC_TRANS. Log the MTi-670 "free
  // acceleration" output here (already gravity-compensated).
  double accX = 0.0;
  double accY = 0.0;
  double accZ = 0.0;
};

// A single GNSS position sample in a local/world frame (e.g. ENU / TWD97).
struct GnssSample {
  double time = 0.0; // absolute timestamp [s]
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

// A single odometer sample: linear velocity in the vehicle (body/FLU) frame
// [m/s]. For a forward-only wheel odometer, set vy = vz = 0.
struct OdomSample {
  double time = 0.0; // absolute timestamp [s]
  double vx = 0.0;
  double vy = 0.0;
  double vz = 0.0;
};

struct PointXYZIT {
  PCL_ADD_POINT4D;   // x, y, z, intensity, ring
  PCL_ADD_INTENSITY; // Add ring index
  double time;       // Add timestamp
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct RegistrationConfig {
  float score_threshold = 2.0f;
  int max_iterations = 100;
  float max_correspondence_distance = 5.0f;

  float rotation_eps = 1e-3f;
  float translation_eps = 1e-3f;
  int num_threads = 0; // 0 means use maximum available threads

  // NDT specific
  float ndt_resolution = 1.0f;
  float ndt_step_size = 0.1f;
  float ndt_transformation_epsilon = 0.01f;
  float ndt_oulier_ratio = 0.2f;
};

} // namespace lidar_utils

// This macro MUST be outside any namespaces
POINT_CLOUD_REGISTER_POINT_STRUCT(
    lidar_utils::PointXYZIT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(double, time, time))
