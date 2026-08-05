#include "rtk_types.hpp"

#include <GeographicLib/LocalCartesian.hpp>
#include <Eigen/Geometry>
#include <cmath>

namespace lidar_imu_rtk_fusion {

class RtkCoordinateConverter {
 public:
  void InitOrigin(double latitude, double longitude, double altitude) {
    local_cartesian_.Reset(latitude, longitude, altitude);
    origin_latitude_ = latitude;
    origin_longitude_ = longitude;
    origin_altitude_ = altitude;
    initialized_ = true;
  }

  bool IsInitialized() const { return initialized_; }

  Eigen::Vector3d Wgs84ToEnu(double latitude, double longitude, double altitude) const {
    double east = 0.0;
    double north = 0.0;
    double up = 0.0;
    local_cartesian_.Forward(latitude, longitude, altitude, east, north, up);
    return {east, north, up};
  }

  void EnuToWgs84(const Eigen::Vector3d& enu, double& latitude, double& longitude,
                  double& altitude) const {
    local_cartesian_.Reverse(enu.x(), enu.y(), enu.z(), latitude, longitude, altitude);
  }

  Eigen::Vector3d ApplyGnssToLidarExtrinsic(const Eigen::Vector3d& enu) const {
    return gnss_R_lidar_ * enu + gnss_t_lidar_;
  }

  void SetGnssToLidarExtrinsic(const Eigen::Vector3d& translation,
                               const Eigen::Matrix3d& rotation) {
    gnss_t_lidar_ = translation;
    gnss_R_lidar_ = rotation;
  }

 private:
  bool initialized_ = false;
  double origin_latitude_ = 0.0;
  double origin_longitude_ = 0.0;
  double origin_altitude_ = 0.0;
  Eigen::Vector3d gnss_t_lidar_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d gnss_R_lidar_ = Eigen::Matrix3d::Identity();
  GeographicLib::LocalCartesian local_cartesian_;
};

bool IsValidLocalEnu(const Eigen::Vector3d& enu) {
  return std::isfinite(enu.x()) && std::isfinite(enu.y()) && std::isfinite(enu.z()) &&
         std::abs(enu.x()) <= 10000.0 && std::abs(enu.y()) <= 10000.0 &&
         std::abs(enu.z()) <= 1000.0;
}

}  // namespace lidar_imu_rtk_fusion
