#include "internal/io_impl.hpp"
#include <iostream>
#include <pcl/io/pcd_io.h>

namespace lidar_utils {
namespace internal {

void executeReadFile(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                     std::string &filepath, FileFormat format,
                     std::vector<double>* timestamps) {
  // If the user doesn't care about timestamps, load directly into XYZI!
  if (!timestamps) {
    switch (format) {
    case FileFormat::PCD_BINARY:
    case FileFormat::PCD_ASCII:
      pcl::io::loadPCDFile(filepath, *cloud);
      break;
    default:
      pcl::io::loadPCDFile(filepath, *cloud);
      std::cerr << "Warning [executeReadFile]: Unknown file encode format\n";
      break;
    }
    return;
  }

  // If the user wants timestamps, load into the custom IT structure
  pcl::PointCloud<lidar_utils::PointXYZIT> cloud_with_time;
  switch (format) {
  case FileFormat::PCD_BINARY:
  case FileFormat::PCD_ASCII:
    if (pcl::io::loadPCDFile(filepath, cloud_with_time) == -1) {
      std::cerr << "Failed to load PCD file.\n";
      return;
    }
    break;
  default:
    if (pcl::io::loadPCDFile(filepath, cloud_with_time) == -1) {
      std::cerr << "Warning [executeReadFile]: Unknown file format.\n";
      return;
    }
    break;
  }

  // Copy into output cloud and extract timestamps
  cloud->clear();
  timestamps->clear();
  cloud->reserve(cloud_with_time.size());
  timestamps->reserve(cloud_with_time.size());

  for (const auto& pt : cloud_with_time.points) {
    pcl::PointXYZI p_xyzi;
    p_xyzi.x = pt.x;
    p_xyzi.y = pt.y;
    p_xyzi.z = pt.z;
    p_xyzi.intensity = pt.intensity;
    cloud->push_back(p_xyzi);
    timestamps->push_back(pt.time);
  }
}

void executeSaveFile(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                     std::string &filepath, FileFormat format) {
  switch (format) {
  case FileFormat::PCD_BINARY:
    pcl::io::savePCDFileBinary(filepath, *cloud);
    break;
  case FileFormat::PCD_ASCII:
    pcl::io::savePCDFileASCII(filepath, *cloud);
    break;
    //   case FileFormat::LAS:
    //     pcl::io::savePCDFileLAS(filepath, *cloud);
    //     break;
  default:
    pcl::io::savePCDFileBinary(filepath, *cloud);
    std::cerr << "Warning [executeSavePCD]: Unknown file encode format, using "
                 "PCD_BINARY\n";
    break;
  }
}
} // namespace internal
} // namespace lidar_utils