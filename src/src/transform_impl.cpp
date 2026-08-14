#include "internal/transform_impl.hpp"
#include "lidar_utils/types.hpp"
#include <algorithm>
#include <cmath>
#include <pcl/common/transforms.h>

namespace lidar_utils {
namespace internal {

void executeDirectGeoreference(CloudType::Ptr &inCloud,
                               CloudType::Ptr &outCloud,
                               Eigen::Matrix4d &trans) {
  if (!inCloud || inCloud->empty())
    return;
  pcl::transformPointCloud(*inCloud, *outCloud, trans);
}

void executeMotionAndDG(CloudType::Ptr &cloud,
                        const std::vector<double> &timestamps,
                        const Eigen::VectorXd &posCurr,
                        const Eigen::VectorXd &posNext, bool motionEnable) {
  if (!cloud || cloud->empty())
    return;

  // Extract time
  double timeCurr = posCurr(0);
  double timeNext = posNext(0);

  // Extract Translation
  Eigen::Vector3d t_curr(posCurr(1), posCurr(2), posCurr(3));
  Eigen::Vector3d t_next(posNext(1), posNext(2), posNext(3));

  // Extract Rotation (Convert degrees to radians)
  auto deg2rad = [](double deg) { return deg * M_PI / 180.0; };
  // ZYX Euler sequence convention
  Eigen::Quaterniond q_curr =
      Eigen::AngleAxisd(deg2rad(posCurr(6)), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(deg2rad(posCurr(5)), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(deg2rad(posCurr(4)), Eigen::Vector3d::UnitX());

  Eigen::Quaterniond q_next =
      Eigen::AngleAxisd(deg2rad(posNext(6)), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(deg2rad(posNext(5)), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(deg2rad(posNext(4)), Eigen::Vector3d::UnitX());

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
    if (has_time && timeNext > timeCurr) {
      double point_time = timestamps[i];
      s = (point_time - timeCurr) / (timeNext - timeCurr);
    } else {
      s = static_cast<double>(i) / static_cast<double>(num_points - 1);
    }

    // Clamp s between 0.0 and 1.0 just in case there are slight timestamp
    // mismatches (allow extrapolation for points that fall outside the
    // [timeCurr, timeNext] range)
    s = std::max(0.0, std::min(1.0, s));

    // 2. Interpolate the pose for this exact microsecond
    Eigen::Quaterniond q_interp = q_curr.slerp(s, q_next);
    Eigen::Vector3d t_interp = (1.0 - s) * t_curr + s * t_next;

    // 3. Deskew!
    Eigen::Vector3d pt(cloud->points[i].x, cloud->points[i].y,
                       cloud->points[i].z);
    Eigen::Vector3d pt_transformed = q_interp * pt + t_interp;

    cloud->points[i].x = pt_transformed.x();
    cloud->points[i].y = pt_transformed.y();
    cloud->points[i].z = pt_transformed.z();
  }
}

// ---------------------------------------------------------------------------
// Integrate the gyro into cumulative rotation over the scan window.
// Reference: imageProjection.cpp::imuDeskewInfo()
// ---------------------------------------------------------------------------
ImuRotationInfo integrateImuGyro(const std::vector<ImuSample> &imu,
                                 double timeScanCur, double timeScanEnd) {
  ImuRotationInfo info;
  if (imu.empty())
    return info;

  info.time.reserve(imu.size());
  info.rotX.reserve(imu.size());
  info.rotY.reserve(imu.size());
  info.rotZ.reserve(imu.size());

  int ptr = 0;
  for (const ImuSample &s : imu) {
    // Discard samples clearly before the scan (0.01 s guard, as in LIO-SAM).
    if (s.time < timeScanCur - 0.01)
      continue;
    // Stop shortly after the scan ends.
    if (s.time > timeScanEnd + 0.01)
      break;

    if (ptr == 0) {
      // First in-window sample anchors the integration at zero rotation.
      info.time.push_back(s.time);
      info.rotX.push_back(0.0);
      info.rotY.push_back(0.0);
      info.rotZ.push_back(0.0);
      ++ptr;
      continue;
    }

    // Rectangular integration of angular velocity: rot += omega * dt.
    double timeDiff = s.time - info.time[ptr - 1];
    info.rotX.push_back(info.rotX[ptr - 1] + s.gyroX * timeDiff);
    info.rotY.push_back(info.rotY[ptr - 1] + s.gyroY * timeDiff);
    info.rotZ.push_back(info.rotZ[ptr - 1] + s.gyroZ * timeDiff);
    info.time.push_back(s.time);
    ++ptr;
  }

  info.available = ptr > 1;
  return info;
}

// ---------------------------------------------------------------------------
// Translation source #1: GNSS position delta over the scan (linear).
// Reference: imageProjection.cpp::odomDeskewInfo()
// ---------------------------------------------------------------------------
TranslationInfo buildTranslationGnss(const std::vector<GnssSample> &gnss,
                                     double timeScanCur, double timeScanEnd,
                                     const Eigen::Matrix3d &R_vehicle_from_world) {
  TranslationInfo info;
  if (gnss.size() < 2 || timeScanEnd <= timeScanCur)
    return info;

  // Linearly interpolate the GNSS position at an arbitrary time. Returns false
  // if the requested time is outside the trajectory (beyond a 0.01 s guard),
  // so we never fabricate motion where no data exists.
  auto interpAt = [&](double t, Eigen::Vector3d &out) -> bool {
    if (t <= gnss.front().time) {
      out = Eigen::Vector3d(gnss.front().x, gnss.front().y, gnss.front().z);
      return t >= gnss.front().time - 0.01;
    }
    if (t >= gnss.back().time) {
      out = Eigen::Vector3d(gnss.back().x, gnss.back().y, gnss.back().z);
      return t <= gnss.back().time + 0.01;
    }
    for (size_t i = 1; i < gnss.size(); ++i) {
      if (gnss[i].time >= t) {
        const GnssSample &a = gnss[i - 1];
        const GnssSample &b = gnss[i];
        double denom = b.time - a.time;
        double r = denom > 0.0 ? (t - a.time) / denom : 0.0;
        out = Eigen::Vector3d(a.x + (b.x - a.x) * r, a.y + (b.y - a.y) * r,
                              a.z + (b.z - a.z) * r);
        return true;
      }
    }
    return false;
  };

  Eigen::Vector3d pBegin, pEnd;
  if (!interpAt(timeScanCur, pBegin) || !interpAt(timeScanEnd, pEnd))
    return info;

  // Two-sample trajectory: 0 at scan start, full delta (world -> vehicle) at
  // scan end. GNSS is typically ~10 Hz, so this is effectively linear across
  // the scan; findTranslation() interpolates it per point.
  info.time = {timeScanCur, timeScanEnd};
  info.pos = {Eigen::Vector3d::Zero(), R_vehicle_from_world * (pEnd - pBegin)};
  info.available = true;
  return info;
}

// ---------------------------------------------------------------------------
// Translation source #2: integrate odometer velocity (vehicle frame).
// ---------------------------------------------------------------------------
TranslationInfo buildTranslationOdom(const std::vector<OdomSample> &odom,
                                     double timeScanCur, double timeScanEnd) {
  TranslationInfo info;
  if (odom.empty())
    return info;

  int ptr = 0;
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  double prevT = 0.0;
  Eigen::Vector3d prevV = Eigen::Vector3d::Zero();

  for (const OdomSample &s : odom) {
    if (s.time < timeScanCur - 0.01)
      continue;
    if (s.time > timeScanEnd + 0.01)
      break;

    Eigen::Vector3d v(s.vx, s.vy, s.vz);
    if (ptr == 0) {
      info.time.push_back(s.time);
      info.pos.push_back(Eigen::Vector3d::Zero());
      prevT = s.time;
      prevV = v;
      ++ptr;
      continue;
    }

    // Trapezoidal integration of velocity into position.
    double dt = s.time - prevT;
    pos += 0.5 * (v + prevV) * dt;
    info.time.push_back(s.time);
    info.pos.push_back(pos);
    prevT = s.time;
    prevV = v;
    ++ptr;
  }

  info.available = ptr > 1;
  return info;
}

// ---------------------------------------------------------------------------
// Translation source #3: double-integrate IMU accel, seeded with v0.
// The accel is assumed gravity-removed and in the vehicle frame. The velocity
// seed v0 is required because the accelerometer alone cannot recover it.
// ---------------------------------------------------------------------------
TranslationInfo buildTranslationImuAcc(const std::vector<ImuSample> &imu,
                                       double timeScanCur, double timeScanEnd,
                                       const Eigen::Vector3d &v0_vehicle) {
  TranslationInfo info;
  if (imu.empty())
    return info;

  int ptr = 0;
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = v0_vehicle;
  double prevT = 0.0;
  Eigen::Vector3d prevA = Eigen::Vector3d::Zero();

  for (const ImuSample &s : imu) {
    if (s.time < timeScanCur - 0.01)
      continue;
    if (s.time > timeScanEnd + 0.01)
      break;

    Eigen::Vector3d a(s.accX, s.accY, s.accZ);
    if (ptr == 0) {
      info.time.push_back(s.time);
      info.pos.push_back(Eigen::Vector3d::Zero());
      prevT = s.time;
      prevA = a;
      ++ptr;
      continue;
    }

    // p += v*dt + 0.5*a*dt^2 ;  v += 0.5*(a+a_prev)*dt  (trapezoidal velocity).
    double dt = s.time - prevT;
    pos += vel * dt + 0.5 * prevA * dt * dt;
    vel += 0.5 * (a + prevA) * dt;
    info.time.push_back(s.time);
    info.pos.push_back(pos);
    prevT = s.time;
    prevA = a;
    ++ptr;
  }

  info.available = ptr > 1;
  return info;
}

// ---------------------------------------------------------------------------
// Interpolate the integrated gyro rotation at an absolute point time.
// Reference: imageProjection.cpp::findRotation()
// ---------------------------------------------------------------------------
static void findRotation(const ImuRotationInfo &imuInfo, double pointTime,
                         double &rotXCur, double &rotYCur, double &rotZCur) {
  rotXCur = 0.0;
  rotYCur = 0.0;
  rotZCur = 0.0;

  const int n = static_cast<int>(imuInfo.time.size());
  int front = 0;
  while (front < n) {
    if (pointTime < imuInfo.time[front])
      break;
    ++front;
  }

  if (front == 0 || front == n) {
    // Before the first / after the last sample: clamp to the nearest end.
    int idx = std::min(front, n - 1);
    rotXCur = imuInfo.rotX[idx];
    rotYCur = imuInfo.rotY[idx];
    rotZCur = imuInfo.rotZ[idx];
  } else {
    // Interpolate between the bracketing samples.
    int back = front - 1;
    double denom = imuInfo.time[front] - imuInfo.time[back];
    double ratioFront = denom > 0.0 ? (pointTime - imuInfo.time[back]) / denom : 0.0;
    double ratioBack = 1.0 - ratioFront;
    rotXCur = imuInfo.rotX[front] * ratioFront + imuInfo.rotX[back] * ratioBack;
    rotYCur = imuInfo.rotY[front] * ratioFront + imuInfo.rotY[back] * ratioBack;
    rotZCur = imuInfo.rotZ[front] * ratioFront + imuInfo.rotZ[back] * ratioBack;
  }
}

// ---------------------------------------------------------------------------
// Interpolate the translation trajectory at an absolute point time.
// Reference: imageProjection.cpp::findPosition()
// ---------------------------------------------------------------------------
static Eigen::Vector3d findTranslation(const TranslationInfo &info,
                                       double pointTime) {
  const int n = static_cast<int>(info.time.size());
  if (n == 0)
    return Eigen::Vector3d::Zero();

  int front = 0;
  while (front < n) {
    if (pointTime < info.time[front])
      break;
    ++front;
  }

  if (front == 0 || front == n) {
    // Before the first / after the last sample: clamp to the nearest end.
    return info.pos[std::min(front, n - 1)];
  }

  int back = front - 1;
  double denom = info.time[front] - info.time[back];
  double ratioFront = denom > 0.0 ? (pointTime - info.time[back]) / denom : 0.0;
  return info.pos[back] + (info.pos[front] - info.pos[back]) * ratioFront;
}

// Build a 4x4 transform from ZYX Euler angles (matching executeMotionAndDG's
// convention) and a translation.
static Eigen::Matrix4d makeTransform(double rotX, double rotY, double rotZ,
                                     double tx, double ty, double tz) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) =
      (Eigen::AngleAxisd(rotZ, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(rotY, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(rotX, Eigen::Vector3d::UnitX()))
          .matrix();
  T(0, 3) = tx;
  T(1, 3) = ty;
  T(2, 3) = tz;
  return T;
}

// ---------------------------------------------------------------------------
// Deskew a cloud: bring every point into the scan-start frame.
// Reference: imageProjection.cpp::deskewPoint()
// ---------------------------------------------------------------------------
void executeMotionComensation(CloudType::Ptr &cloud,
                              const std::vector<double> &timestamps,
                              const std::vector<ImuSample> &imu,
                              MotionMethod method,
                              const std::vector<GnssSample> &gnss,
                              const std::vector<OdomSample> &odom,
                              const Eigen::Matrix3d &R_vehicle_from_world,
                              const Eigen::Vector3d &v0_vehicle) {
  if (!cloud || cloud->empty())
    return;

  const size_t num_points = cloud->size();
  const bool has_time = (timestamps.size() == num_points);
  if (!has_time)
    // Without per-point timestamps there is nothing to deskew against.
    return;

  // Scan time span. Points are not assumed ordered, so scan for min/max.
  double timeScanCur = timestamps.front();
  double timeScanEnd = timestamps.front();
  for (double t : timestamps) {
    timeScanCur = std::min(timeScanCur, t);
    timeScanEnd = std::max(timeScanEnd, t);
  }

  // 1. Rotation is always the integrated gyro (imuDeskewInfo).
  ImuRotationInfo rotInfo = integrateImuGyro(imu, timeScanCur, timeScanEnd);

  // 2. Translation comes from the selected source (odomDeskewInfo analog).
  TranslationInfo transInfo;
  switch (method) {
  case MotionMethod::GNSS_TRANS:
    transInfo = buildTranslationGnss(gnss, timeScanCur, timeScanEnd,
                                     R_vehicle_from_world);
    break;
  case MotionMethod::ODOM_TRANS:
    transInfo = buildTranslationOdom(odom, timeScanCur, timeScanEnd);
    break;
  case MotionMethod::IMU_ACC_TRANS:
    transInfo = buildTranslationImuAcc(imu, timeScanCur, timeScanEnd,
                                       v0_vehicle);
    break;
  }

  // If neither source is usable, leave the cloud untouched.
  if (!rotInfo.available && !transInfo.available)
    return;

  // Reference = pose at the very start of the scan. Every point is re-based
  // onto this frame, so the deskewed cloud is defined at the scan-start pose
  // (ready for subsequent direct georeferencing).
  double rot0X = 0.0, rot0Y = 0.0, rot0Z = 0.0;
  if (rotInfo.available)
    findRotation(rotInfo, timeScanCur, rot0X, rot0Y, rot0Z);
  const Eigen::Vector3d pos0 =
      transInfo.available ? findTranslation(transInfo, timeScanCur)
                          : Eigen::Vector3d::Zero();
  const Eigen::Matrix4d transStartInverse =
      makeTransform(rot0X, rot0Y, rot0Z, pos0.x(), pos0.y(), pos0.z())
          .inverse();

  // Point-by-point Interpolation
  for (size_t i = 0; i < num_points; ++i) {
    const double pointTime = timestamps[i];

    // Rotation Matrix Preparation (integrated gyro, interpolated to pointTime).
    double rotXCur = 0.0, rotYCur = 0.0, rotZCur = 0.0;
    if (rotInfo.available)
      findRotation(rotInfo, pointTime, rotXCur, rotYCur, rotZCur);

    // Translation Vector Preparation (chosen source, interpolated to pointTime).
    Eigen::Vector3d pos = transInfo.available
                              ? findTranslation(transInfo, pointTime)
                              : Eigen::Vector3d::Zero();

    // Form the Transformation Matrix (this point's pose within the scan),
    // then express it relative to the scan-start pose.
    const Eigen::Matrix4d transFinal =
        makeTransform(rotXCur, rotYCur, rotZCur, pos.x(), pos.y(), pos.z());
    const Eigen::Matrix4d transBt = transStartInverse * transFinal;

    // Direct Gereferencing (deskew this point into the scan-start frame).
    Eigen::Vector3d pt(cloud->points[i].x, cloud->points[i].y,
                       cloud->points[i].z);
    Eigen::Vector3d ptd =
        transBt.block<3, 3>(0, 0) * pt + transBt.block<3, 1>(0, 3);
    cloud->points[i].x = ptd.x();
    cloud->points[i].y = ptd.y();
    cloud->points[i].z = ptd.z();
  }
}

// ---------------------------------------------------------------------------
// WGS84 geodetic -> local ENU (East, North, Up) metres about a reference.
// Raw GNSS (lat/lon/h) must be projected to a metric Cartesian frame before it
// can be used for GNSS_TRANS motion compensation; ENU is the natural choice.
// ---------------------------------------------------------------------------
Eigen::Vector3d executeWgs84ToEnu(double lat_deg, double lon_deg, double h,
                                  double lat0_deg, double lon0_deg, double h0) {
  constexpr double a = 6378137.0;           // WGS84 semi-major axis [m]
  constexpr double f = 1.0 / 298.257223563; // flattening
  const double e2 = f * (2.0 - f);          // first eccentricity squared

  auto deg2rad = [](double d) { return d * M_PI / 180.0; };

  // Geodetic (rad, rad, m) -> ECEF (m).
  auto geodeticToEcef = [&](double lat, double lon, double alt) {
    double slat = std::sin(lat), clat = std::cos(lat);
    double slon = std::sin(lon), clon = std::cos(lon);
    double N = a / std::sqrt(1.0 - e2 * slat * slat);
    return Eigen::Vector3d((N + alt) * clat * clon, (N + alt) * clat * slon,
                           (N * (1.0 - e2) + alt) * slat);
  };

  const double lat = deg2rad(lat_deg), lon = deg2rad(lon_deg);
  const double lat0 = deg2rad(lat0_deg), lon0 = deg2rad(lon0_deg);

  const Eigen::Vector3d d =
      geodeticToEcef(lat, lon, h) - geodeticToEcef(lat0, lon0, h0);

  const double slat0 = std::sin(lat0), clat0 = std::cos(lat0);
  const double slon0 = std::sin(lon0), clon0 = std::cos(lon0);

  // Rotate the ECEF difference into the origin's local tangent plane (ENU).
  Eigen::Matrix3d R;
  R << -slon0,          clon0,         0.0,
       -slat0 * clon0, -slat0 * slon0, clat0,
        clat0 * clon0,  clat0 * slon0, slat0;

  return R * d; // (East, North, Up) [m]
}

// ---------------------------------------------------------------------------
// Estimate world-frame velocity at time t by finite-differencing the GNSS
// trajectory (used to seed IMU_ACC_TRANS, whose accel cannot recover v0).
// ---------------------------------------------------------------------------
Eigen::Vector3d estimateGnssVelocity(const std::vector<GnssSample> &gnss,
                                     double t) {
  if (gnss.size() < 2)
    return Eigen::Vector3d::Zero();

  // Use the pair of samples straddling t (clamped to the ends).
  size_t i = 1;
  while (i < gnss.size() && gnss[i].time < t)
    ++i;
  if (i >= gnss.size())
    i = gnss.size() - 1;

  const GnssSample &a = gnss[i - 1];
  const GnssSample &b = gnss[i];
  double dt = b.time - a.time;
  if (dt <= 0.0)
    return Eigen::Vector3d::Zero();

  return Eigen::Vector3d((b.x - a.x) / dt, (b.y - a.y) / dt, (b.z - a.z) / dt);
}

} // namespace internal
} // namespace lidar_utils