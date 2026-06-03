#include "internal/motion_impl.hpp"
#include "lidar_utils/types.hpp"
#include <pcl/common/transforms.h>

namespace lidar_utils {
namespace internal {

void executeMotionAndDG(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                        const std::vector<double> &timestamps,
                        const Eigen::VectorXd &posPrev,
                        const Eigen::VectorXd &posCurr, bool motionEnable) {
  if (!cloud || cloud->empty())
    return;

  // Extract time
  double timePrev = posPrev(0);
  double timeCurr = posCurr(0);

  // Extract Translation
  Eigen::Vector3d t_prev(posPrev(1), posPrev(2), posPrev(3));
  Eigen::Vector3d t_curr(posCurr(1), posCurr(2), posCurr(3));

  // Extract Rotation (Convert degrees to radians)
  auto deg2rad = [](double deg) { return deg * M_PI / 180.0; };
  // ZYX Euler sequence convention
  Eigen::Quaterniond q_prev =
      Eigen::AngleAxisd(deg2rad(posPrev(6)), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(deg2rad(posPrev(5)), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(deg2rad(posPrev(4)), Eigen::Vector3d::UnitX());

  Eigen::Quaterniond q_curr =
      Eigen::AngleAxisd(deg2rad(posCurr(6)), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(deg2rad(posCurr(5)), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(deg2rad(posCurr(4)), Eigen::Vector3d::UnitX());

  // If motion compensation is disabled, just transform the whole cloud by the
  // current pose
  if (!motionEnable) {
    Eigen::Matrix4d T_curr = Eigen::Matrix4d::Identity();
    T_curr.block<3, 3>(0, 0) = q_curr.toRotationMatrix();
    T_curr.block<3, 1>(0, 3) = t_curr;
    pcl::transformPointCloud(*cloud, *cloud, T_curr);
    return;
  }

  size_t num_points = cloud->size();
  bool has_time = (timestamps.size() == num_points);

  // Point-by-point Interpolation
  for (size_t i = 0; i < num_points; ++i) {
    // 1. Calculate the exact interpolation ratio (s) using the point's real
    // timestamp
    double s = 0.0;
    if (has_time && timeCurr > timePrev) {
      double point_time = timestamps[i];
      s = (point_time - timePrev) / (timeCurr - timePrev);
    } else {
      s = static_cast<double>(i) / static_cast<double>(num_points - 1);
    }

    // Clamp s between 0.0 and 1.0 just in case there are slight timestamp
    // mismatches (allow extrapolation for points that fall outside the
    // [timePrev, timeCurr] range)
    s = std::max(0.0, std::min(1.0, s));

    // 2. Interpolate the pose for this exact microsecond
    Eigen::Quaterniond q_interp = q_prev.slerp(s, q_curr);
    Eigen::Vector3d t_interp = (1.0 - s) * t_prev + s * t_curr;

    // 3. Deskew!
    Eigen::Vector3d pt(cloud->points[i].x, cloud->points[i].y,
                       cloud->points[i].z);
    Eigen::Vector3d pt_transformed = q_interp * pt + t_interp;

    cloud->points[i].x = pt_transformed.x();
    cloud->points[i].y = pt_transformed.y();
    cloud->points[i].z = pt_transformed.z();
  }
}

} // namespace internal
} // namespace lidar_utils