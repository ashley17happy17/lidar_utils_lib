#pragma once

#define PCL_NO_PRECOMPILE // Required for custom point types
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

namespace lidar_utils {

// 支援的光達型號選單
enum class SensorType {
  OUSTER_OS1_128,
  OUSTER_OS1_32,
  VELODYNE_VLP16,
  VELODYNE_VLS128
};

enum class FileFormat { PCD_BINARY, PCD_ASCII, LAS };

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

} // namespace lidar_utils

// This macro MUST be outside any namespaces
POINT_CLOUD_REGISTER_POINT_STRUCT(
    lidar_utils::PointXYZIT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(double, time, time))
