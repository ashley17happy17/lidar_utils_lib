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
