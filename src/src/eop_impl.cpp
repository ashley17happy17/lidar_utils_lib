#include "internal/eop_impl.hpp"
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_utils {
namespace internal {
Eigen::Matrix4d
executeEOPCalib(const std::vector<Eigen::Matrix4d> &gps_relative_motions,
                const std::vector<Eigen::Matrix4d> &lidar_relative_motions) {
  if (gps_relative_motions.size() != lidar_relative_motions.size() ||
      gps_relative_motions.empty()) {
    std::cerr << "[ERROR] EOP Calibration requires equal and non-empty motion "
                 "sequences."
              << std::endl;
    return Eigen::Matrix4d::Identity();
  }

  size_t n = gps_relative_motions.size();

  // ---------------------------------------------------------
  // Step 1: Solve for Boresight (Rotation R_X)
  // Equation: R_A * R_X = R_X * R_B  =>  (I \otimes R_A - R_B^T \otimes I) *
  // vec(R_X) = 0
  // ---------------------------------------------------------
  Eigen::MatrixXd M(9 * n, 9);
  M.setZero();
  Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

  for (size_t i = 0; i < n; ++i) {
    Eigen::Matrix3d R_A = gps_relative_motions[i].block<3, 3>(0, 0);
    Eigen::Matrix3d R_B = lidar_relative_motions[i].block<3, 3>(0, 0);

    Eigen::MatrixXd K1(9, 9);
    K1.setZero();
    Eigen::MatrixXd K2(9, 9);
    K2.setZero();
    Eigen::Matrix3d R_B_T = R_B.transpose();

    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        K1.block<3, 3>(3 * r, 3 * c) = I3(r, c) * R_A;
        K2.block<3, 3>(3 * r, 3 * c) = R_B_T(r, c) * I3;
      }
    }
    M.block<9, 9>(9 * i, 0) = K1 - K2;
  }

  // Solve M * x = 0 via SVD
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU |
                                               Eigen::ComputeThinV);
  Eigen::VectorXd x = svd.matrixV().col(
      8); // Null space corresponds to the smallest singular value

  // Reshape column-major vector to 3x3 matrix
  Eigen::Matrix3d R_X_est;
  R_X_est << x(0), x(3), x(6), x(1), x(4), x(7), x(2), x(5), x(8);

  // Project onto SO(3) to guarantee a valid rotation matrix
  Eigen::JacobiSVD<Eigen::Matrix3d> svd_proj(R_X_est, Eigen::ComputeFullU |
                                                          Eigen::ComputeFullV);
  Eigen::Matrix3d R_X = svd_proj.matrixU() * svd_proj.matrixV().transpose();
  if (R_X.determinant() < 0) {
    Eigen::Matrix3d V = svd_proj.matrixV();
    V.col(2) *= -1;
    R_X = svd_proj.matrixU() * V.transpose();
  }

  // ---------------------------------------------------------
  // Step 2: Solve for Lever Arm (Translation t_X)
  // Equation: (R_A - I) * t_X = R_X * t_B - t_A
  // ---------------------------------------------------------
  Eigen::MatrixXd C(3 * n, 3);
  Eigen::VectorXd d(3 * n);

  for (size_t i = 0; i < n; ++i) {
    Eigen::Matrix3d R_A = gps_relative_motions[i].block<3, 3>(0, 0);
    Eigen::Vector3d t_A = gps_relative_motions[i].block<3, 1>(0, 3);
    Eigen::Vector3d t_B = lidar_relative_motions[i].block<3, 1>(0, 3);

    C.block<3, 3>(3 * i, 0) = R_A - I3;
    d.segment<3>(3 * i) = R_X * t_B - t_A;
  }

  // Solve C * t_X = d via SVD
  Eigen::Vector3d t_X =
      C.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(d);

  // ---------------------------------------------------------
  // Step 3: Combine and return
  // ---------------------------------------------------------
  Eigen::Matrix4d EOP = Eigen::Matrix4d::Identity();
  EOP.block<3, 3>(0, 0) = R_X;
  EOP.block<3, 1>(0, 3) = t_X;

  return EOP;
}

Eigen::Matrix4d executeTransformEOP(const Eigen::Matrix4d &eop,
                                    const Eigen::Vector3d &out_la,
                                    const Eigen::Vector3d &out_bs) {
  // Build Transform from out_la and out_bs
  Eigen::Matrix4d T_offset = Eigen::Matrix4d::Identity();

  double r = out_bs.x() * M_PI / 180.0; // Roll
  double p = out_bs.y() * M_PI / 180.0; // Pitch
  double y = out_bs.z() * M_PI / 180.0; // Yaw

  Eigen::AngleAxisd rollAngle(r, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(y, Eigen::Vector3d::UnitZ());

  // ZYX Euler sequence convention
  Eigen::Matrix3d R_offset = (yawAngle * pitchAngle * rollAngle).matrix();

  T_offset.block<3, 3>(0, 0) = R_offset;
  T_offset.block<3, 1>(0, 3) = out_la;

  // Assuming eop is T_{GNSS -> LiDAR} and offset is T_{Vehicle -> GNSS}
  // Then T_{Vehicle -> LiDAR} = T_{Vehicle -> GNSS} * T_{GNSS -> LiDAR}
  // Or if offset is just an arbitrary base frame shift.
  return T_offset * eop;
}

void executePrintEOP(const Eigen::Matrix4d &eop,
                     const std::string &sensor_name) {
  Eigen::Matrix3d R = eop.block<3, 3>(0, 0);
  Eigen::Vector3d t = eop.block<3, 1>(0, 3);
  Eigen::Quaterniond q(R);

  // Get euler angles (ZYX convention: Yaw, Pitch, Roll)
  Eigen::Vector3d euler = R.eulerAngles(2, 1, 0);

  // Convert to degrees
  euler *= 180.0 / M_PI;

  std::cout << "\n============================================================"
            << std::endl;
  std::cout << "[EOP Calibration Result] GNSS to " << sensor_name << std::endl;
  std::cout << "============================================================"
            << std::endl;
  std::cout << "Leverarm (m)    : [X: " << t.x() << ", Y: " << t.y()
            << ", Z: " << t.z() << "]" << std::endl;
  std::cout << "Boresight (deg) : [Roll: " << euler.z()
            << ", Pitch: " << euler.y() << ", Yaw: " << euler.x() << "]"
            << std::endl;
  std::cout << "Quaternion      : [w: " << q.w() << ", x: " << q.x()
            << ", y: " << q.y() << ", z: " << q.z() << "]" << std::endl;
  std::cout << "Rotation Matrix :\n" << R << std::endl;
  std::cout << "============================================================\n"
            << std::endl;
}

Eigen::Matrix4d executeGetExtrinsics(const std::string &type,
                                     const Eigen::Vector3d &trans,
                                     const Eigen::Vector3d &rot) {
  Eigen::Matrix4d ext = Eigen::Matrix4d::Identity();

  // 1. Build the Rotation Matrix.
  // Although the parameter is labeled "FRD", standard LiDAR calibration
  // software actually computes the Boresight Euler angles in FLU (Forward,
  // Left, Up) mathematically.
  double r = rot[0] * M_PI / 180.0;
  double p = rot[1] * M_PI / 180.0;
  double y = rot[2] * M_PI / 180.0;

  Eigen::AngleAxisd rollAngle(r, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(y, Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d R_gnss_to_calib =
      (yawAngle * pitchAngle * rollAngle).matrix();

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
    C_in(0, 0) = -1.0;
    C_in(1, 1) = -1.0;
    C_in(2, 2) = 1.0;

    // Output is exactly BRU (180 deg opposite of FLU).
    // We map BRU to the vehicle's FLU frame by rotating 180 degrees in XY.
    // X_flu (Forward) = Y_bru (Wait, mapping from the pre-rotated frame)
    // Mathematically, we flip the X and Y axes of the previous C_out.
    C_out(0, 0) = 0.0;
    C_out(0, 1) = 1.0;
    C_out(0, 2) = 0.0;
    C_out(1, 0) = -1.0;
    C_out(1, 1) = 0.0;
    C_out(1, 2) = 0.0;
    C_out(2, 0) = 0.0;
    C_out(2, 1) = 0.0;
    C_out(2, 2) = 1.0;
  } else if (type.find("VELODYNE") != std::string::npos) {
    // Velodyne calibration was done in RFU, and point clouds are still RFU.
    C_in = Eigen::Matrix3d::Identity();

    // Output is perfectly aligned RFU. We must map RFU to the vehicle's FLU
    // frame. X_flu (Forward) = Y_rfu Y_flu (Left) = -X_rfu Z_flu (Up) = Z_rfu
    C_out(0, 0) = 0.0;
    C_out(0, 1) = 1.0;
    C_out(0, 2) = 0.0;
    C_out(1, 0) = -1.0;
    C_out(1, 1) = 0.0;
    C_out(1, 2) = 0.0;
    C_out(2, 0) = 0.0;
    C_out(2, 1) = 0.0;
    C_out(2, 2) = 1.0;
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