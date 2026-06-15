# LiDAR_Utils_Lib

## Introduction
This library provides common functions for LiDAR point cloud processing.

## Content
- [Common](#common)
    - [Merge](#merge)
- [EOP](#eop)
    - [EOP Calibration](#eop-calibration)
    - [Get Extrinsics](#get-extrinsics)
- [Filter](#filter)
    - [Crop](#crop)
    - [Denoise](#denoise)
    - [Remove Artifact](#remove-artifact)
    - [Downsample](#downsample)
    - [Remove NaN](#remove-nan)
- [IO](#io)
    - [Read Content](#read-content)
    - [Read File](#read-file)
    - [Save File](#save-file)
- [Registration](#registration)
    - [GICP](#gicp)
    - [NDT](#ndt)
- [Transform](#transform)
    - [Direct Georeference](#direct-georeference)
    - [Motion Compensate And DG](#motion-compensate-and-dg)

## Functions
- ### Common
    - #### [Merge](#merge)
        - Intro: Merge two point clouds with voxelization.
        - Usage: 
            - `void mergeCloud(CloudType::Ptr &base_cloud, const CloudType::ConstPtr &other_cloud, float voxelSize)`
- ### EOP
    - #### [EOP Calibration](#eop-calibration)
        - Intro: EOP calibration of GNSS and LiDAR.
        - Usage: 
            - `Eigen::Matrix4d eopCalib(const std::vector<Eigen::Matrix4d> &gps_relative_motions, const std::vector<Eigen::Matrix4d> &lidar_relative_motions)`
    - #### [Transform EOP](#transform-eop)
        - Intro: Transform an EOP calibration matrix by an external offset (Leverarm in meters, Boresight in degrees). Useful for shifting the base frame (e.g., GNSS -> LiDAR to VehicleCenter -> LiDAR).
        - Usage:
            - `Eigen::Matrix4d transformEOP(const Eigen::Matrix4d &eop, const Eigen::Vector3d &out_la, const Eigen::Vector3d &out_bs)`
    - #### [Print EOP](#print-eop)
        - Intro: Print the EOP parameters (Leverarm, Boresight, Quaternion, Rotation Matrix) cleanly to the console.
        - Usage:
            - `void printEOP(const Eigen::Matrix4d &eop, const std::string &sensor_name = "LiDAR")`        
    - #### [Get Extrinsics](#get-extrinsics)
        - Intro: Get the transformation matrix between two sensors.
        - Usage: 
            - `Eigen::Matrix4d getExtrinsics(const std::string &type, const Eigen::Vector3d &trans, const Eigen::Vector3d &rot)`
        - Parameter:
            - `type`: The type of the sensor.
            - `trans`: The translation between the two sensors.
            - `rot`: The rotation between the two sensors.
- ### Filter
    - #### [Crop](#crop)
        - Intro: Cuboid geometric cropping.
        - Usage: 
            - `void cropCloud(CloudType::Ptr &cloud, const Eigen::Vector3f &minBound, const Eigen::Vector3f &maxBound)`
            - `void cropCloud(CloudType::Ptr &cloud, std::vector<double> &timestamps, const Eigen::Vector3f &minBound, const Eigen::Vector3f &maxBound)`
    - #### [Denoise](#denoise)
        - Intro: Guided Filter for denoising.
        - Usage: 
            - `void denoiseCloud(CloudType::Ptr &cloud, float radius, float epsilon)`
    - #### [Remove Artifact](#remove-artifact)
        - Intro: Removes ghost artifacts.
        - Usage: 
            - `void removeArtifactCloud(CloudType::Ptr &cloud)`
            - `void removeArtifactCloud(CloudType::Ptr &cloud, std::vector<double> &timestamps)`
    - #### [Downsample](#downsample)
        - Intro: Downsample the point cloud with voxel size.
        - Usage:
            - `void downsampleCloud(CloudType::Ptr &cloud, float voxelSize)`
            - `void downsampleCloud(CloudType::Ptr &cloud, std::vector<double> &timestamps, float voxelSize)`
    - #### [Remove NaN](#remove-nan)
        - Intro: Remove NaN points from the point cloud.
        - Usage: 
            - `void removeNaNCloud(CloudType::Ptr &cloud)`
            - `void removeNaNCloud(CloudType::Ptr &cloud, std::vector<double> &timestamps)`
- ### IO
    - #### [Read Content](#read-content)
        - Intro: Read the content of the directory.
        - Usage: 
            - `void readContent(const std::string &path, std::vector<LidarContent> &file_list)`
    - #### [Read File](#read-file)
        - Intro: Read the point cloud from a file.
        - Usage:
            - `void readFile(CloudType::Ptr &cloud, std::string &filepath, FileFormat format)`
            - `void readFile(CloudType::Ptr &cloud, std::vector<double> &timestamps, std::string &filepath, FileFormat format)`
    - #### [Save File](#save-file)
        - Intro: Save the point cloud to a file.
        - Usage:
            - `void saveFile(CloudType::Ptr &cloud, std::string &filepath, FileFormat format)`
            - `void saveFile(CloudType::Ptr &cloud, std::vector<double> &timestamps, std::string &filepath, FileFormat format)`
- ### Registration
    - #### [GICP](#gicp)
        - Intro: Iterative Closest Point (ICP) based scan-to-map matching.
        - Usage:
            - `int GICP(CloudType::Ptr &cloud, CloudType::Ptr &map, Eigen::Matrix4d &in_transform, Eigen::Matrix4d &out_transform, const RegistrationConfig &config)`
    - #### [NDT](#ndt)
        - Intro: Iterative Closest Point (ICP) based scan-to-scan matching.
        - Usage:
            - `int NDT(CloudType::Ptr &cloud, CloudType::Ptr &map, Eigen::Matrix4d &in_transform, Eigen::Matrix4d &out_transform, const RegistrationConfig &config)`
- ### Transform
    - #### [Direct Georeference](#direct-georeference)
        - Intro: Direct Georeference of the point cloud.
        - Usage:
            - `void directGeoreference(CloudType::Ptr &cloud, Eigen::Matrix4d &trans)`
    - #### [Motion Compensate And DG](#motion-compensate-and-dg)
        - Intro: Motion compensation and direct georeferencing of the point cloud.
        - Usage:
            - `void motionCompensateAndDG(CloudType::Ptr &cloud, const std::vector<double> &timestamps, const Eigen::VectorXd &posCurr, const Eigen::VectorXd &posNext, bool motionEnable)`

## Reference
1. PCL documentation: https://pointclouds.org/documentation/
2. PDAL documentation: https://pdal.io/en/latest/download.html
3. OMP documentation: https://www.openmp.org/documentation/
4. Point Cloud Denoise: https://github.com/aipiano/guided-filter-point-cloud-denoise.git
5. Small GICP lib: https://github.com/koide3/small_gicp.git
