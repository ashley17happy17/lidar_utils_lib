#include "internal/io_impl.hpp"
#include <filesystem>
#include <iostream>
#include <pcl/io/pcd_io.h>

namespace fs = std::filesystem;
namespace lidar_utils {
namespace internal {

void executeReadContent(const std::string &filepath,
                        std::vector<LidarContent> &file_list) {
  if (fs::exists(filepath) && fs::is_directory(filepath)) {
    for (const auto &entry : fs::directory_iterator(filepath)) {
      if (entry.is_regular_file()) {
        std::string ext = entry.path().extension().string();
        // Support standard point cloud file extensions
        if (ext == ".pcd" || ext == ".las") {
          std::string stem = entry.path().stem().string();
          try {
            double ts = std::stod(stem) * 1e-3; // Convert from ms to sec
            file_list.push_back({ts, entry.path().string()});
          } catch (...) {
            std::cerr << "[WARN] Failed to parse timestamp from filename: "
                      << entry.path().string() << std::endl;
          }
        }
      }
    }
  } else {
    std::cerr << "[ERROR] Lidar path does not exist or is not a directory: "
              << filepath << std::endl;
  }

  // Sort files sequentially by timestamp to allow fast lookup for
  // synchronization
  std::sort(file_list.begin(), file_list.end(),
            [](const LidarContent &a, const LidarContent &b) {
              return a.timestamp < b.timestamp;
            });
}

void executeReadFile(CloudType::Ptr &cloud, std::string &filepath,
                     FileFormat format, std::vector<double> *timestamps) {
  // If the user doesn't care about timestamps, load directly into XYZI!
  if (!timestamps) {
    switch (format) {
    case FileFormat::PCD_BINARY:
    case FileFormat::PCD_ASCII:
      pcl::io::loadPCDFile(filepath, *cloud);
      break;
    default:
      pcl::io::loadPCDFile(filepath, *cloud);
      std::cerr << "[WARN]: Unknown file encode format\n";
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
      std::cerr << "[ERROR] Failed to load PCD file.\n";
      return;
    }
    break;
  default:
    if (pcl::io::loadPCDFile(filepath, cloud_with_time) == -1) {
      std::cerr << "[ERROR]: Unknown file format.\n";
      return;
    }
    break;
  }

  // Copy into output cloud and extract timestamps
  cloud->clear();
  timestamps->clear();
  cloud->reserve(cloud_with_time.size());
  timestamps->reserve(cloud_with_time.size());

  for (const auto &pt : cloud_with_time.points) {
    pcl::PointXYZI p_xyzi;
    p_xyzi.x = pt.x;
    p_xyzi.y = pt.y;
    p_xyzi.z = pt.z;
    p_xyzi.intensity = pt.intensity;
    cloud->push_back(p_xyzi);
    timestamps->push_back(pt.time);
  }
}

void executeSaveFile(CloudType::Ptr &cloud, std::string &filepath,
                     FileFormat format) {
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
    std::cerr << "[WARN]]: Unknown file encode format, using "
                 "PCD_BINARY\n";
    break;
  }
}
} // namespace internal
} // namespace lidar_utils