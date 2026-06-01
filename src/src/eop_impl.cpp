#include "internal/eop_impl.hpp"
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_utils {
namespace internal {
void executeEOPCalib(const Eigen::MatrixXd &la, const Eigen::MatrixXd &bs,
                     const Eigen::Vector3d &laCalib,
                     const Eigen::Vector3d &bsCalib) {}

} // namespace internal
} // namespace lidar_utils