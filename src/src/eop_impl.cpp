#include "internal/eop_impl.hpp"
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_utils {
namespace internal {
void executeEOPCalib(const Eigen::MatrixXd &la, const Eigen::MatrixXd &bs,
                     const Eigen::Vector3d &laCalib,
                     const Eigen::Vector3d &bsCalib) {}

Eigen::Matrix4d executeGetExtrinsics(const std::string &type,
                                     const Eigen::Vector3d &trans,
                                     const Eigen::Vector3d &rot) {
    Eigen::Matrix4d ext = Eigen::Matrix4d::Identity();

    // 1. Build the Rotation Matrix. 
    // Although the parameter is labeled "FRD", standard LiDAR calibration software 
    // actually computes the Boresight Euler angles in FLU (Forward, Left, Up) mathematically.
    double r = rot[0] * M_PI / 180.0;
    double p = rot[1] * M_PI / 180.0;
    double y = rot[2] * M_PI / 180.0;

    Eigen::AngleAxisd rollAngle(r, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(y, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d R_gnss_to_calib = (yawAngle * pitchAngle * rollAngle).matrix();

    // The Boresight parameters from the calibration tool are given as the 
    // direct multiplier to map from Sensor to GNSS. NO transpose is needed!
    Eigen::Matrix3d R_calib_to_gnss = R_gnss_to_calib; 

    // 2. Handle Leverarm. 
    // Leverarm IS physically measured in FRD. We must convert it to FLU.
    // X_flu = X_frd, Y_flu = -Y_frd, Z_flu = -Z_frd
    Eigen::Vector3d t_gnss_flu(trans[0], -trans[1], -trans[2]);

    // 3. Handle Sensor Native Frame -> Calibration Frame differences
    Eigen::Matrix3d C_in = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d C_out = Eigen::Matrix3d::Identity();

    if (type.find("OUSTER") != std::string::npos) {
        // Ouster calibration was done when data was BRU. Data is now FLU.
        // Convert FLU points back to BRU so the calibration matrix works.
        C_in(0, 0) = -1.0; C_in(1, 1) = -1.0; C_in(2, 2) = 1.0;

        // Output is exactly BRU (180 deg opposite of FLU).
        // We map BRU to the vehicle's FLU frame by rotating 180 degrees in XY.
        // X_flu (Forward) = Y_bru (Wait, mapping from the pre-rotated frame)
        // Mathematically, we flip the X and Y axes of the previous C_out.
        C_out(0, 0) = 0.0; C_out(0, 1) = 1.0; C_out(0, 2) = 0.0;
        C_out(1, 0) = -1.0; C_out(1, 1) = 0.0; C_out(1, 2) = 0.0;
        C_out(2, 0) = 0.0; C_out(2, 1) = 0.0; C_out(2, 2) = 1.0;
    } else if (type.find("VELODYNE") != std::string::npos) {
        // Velodyne calibration was done in RFU, and point clouds are still RFU.
        C_in = Eigen::Matrix3d::Identity();

        // Output is perfectly aligned RFU. We must map RFU to the vehicle's FLU frame.
        // X_flu (Forward) = Y_rfu
        // Y_flu (Left) = -X_rfu
        // Z_flu (Up) = Z_rfu
        C_out(0, 0) = 0.0; C_out(0, 1) = 1.0; C_out(0, 2) = 0.0;
        C_out(1, 0) = -1.0; C_out(1, 1) = 0.0; C_out(1, 2) = 0.0;
        C_out(2, 0) = 0.0; C_out(2, 1) = 0.0; C_out(2, 2) = 1.0;
    }

    // 4. Combine transformations
    Eigen::Matrix3d R_final = C_out * R_calib_to_gnss * C_in;
    Eigen::Vector3d t_final = t_gnss_flu;

    ext.block<3, 3>(0, 0) = R_final;
    ext.block<3, 1>(0, 3) = t_final;
    return ext;
}

} // namespace internal
} // namespace lidar_utils