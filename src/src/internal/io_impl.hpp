#pragma once
#include "lidar_utils/types.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_utils {
namespace internal {

void executeReadContent(const std::string &filepath,
                        std::vector<LidarContent> &file_list);

void executeReadFile(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                     std::string &filepath, FileFormat format,
                     std::vector<double> *timestamps = nullptr);

void executeSaveFile(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                     std::string &filepath, FileFormat format);

} // namespace internal
} // namespace lidar_utils