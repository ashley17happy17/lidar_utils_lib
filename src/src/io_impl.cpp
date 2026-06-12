#include "internal/io_impl.hpp"
#include <filesystem>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Reader.hpp>
#include <pdal/StageFactory.hpp>

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
  case FileFormat::LAS:
    if (loadLASFile(filepath, cloud_with_time) == -1) {
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
    PointType p_xyzi;
    p_xyzi.x = pt.x;
    p_xyzi.y = pt.y;
    p_xyzi.z = pt.z;
    p_xyzi.intensity = pt.intensity;
    cloud->push_back(p_xyzi);
    timestamps->push_back(pt.time);
  }
}

int loadLASFile(std::string &filepath,
                pcl::PointCloud<lidar_utils::PointXYZIT> cloud) {
  ::std::ifstream inputFile(filepath, ::std::ios::in | ::std::ios::binary);
  if (!inputFile.is_open()) {
    std::cerr << "[ERROR] Failed to load LAS file.\n";
    return -1;
  }

  auto start = std::chrono::steady_clock::now();

  pdal::PointTable table;
  pdal::Options options;
  options.add("filename", filepath);

  pdal::StageFactory stageFactory;
  pdal::Stage *reader = stageFactory.createStage("readers.las");
  if (!reader) {
    std::cerr << "[ERROR] Failed to create LAS reader.\n";
    return -1;
  }

  reader->setOptions(options);
  reader->prepare(table);
  pdal::PointViewSet viewSet = reader->execute(table);
  pdal::PointViewPtr view = *viewSet.begin();

  for (pdal::PointId id = 0; id < view->size(); ++id) {
    lidar_utils::PointXYZIT p_xyzit;
    p_xyzit.x = view->getFieldAs<double>(pdal::Dimension::Id::X, id);
    p_xyzit.y = view->getFieldAs<double>(pdal::Dimension::Id::Y, id);
    p_xyzit.z = view->getFieldAs<double>(pdal::Dimension::Id::Z, id);
    double intensity_raw =
        view->getFieldAs<double>(pdal::Dimension::Id::Intensity, id);
    p_xyzit.intensity = std::min(intensity_raw / 65535.0, 1.0);
    p_xyzit.time = view->getFieldAs<double>(pdal::Dimension::Id::GpsTime, id);
    cloud.push_back(p_xyzit);
  }
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "[INFO] Read LAS File: " << filepath
            << " Elapsed time: " << elapsed.count() << "s\n";
  return 1;
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