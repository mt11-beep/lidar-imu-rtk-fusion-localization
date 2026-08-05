#include "rtk_types.hpp"

#include <cmath>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace lidar_imu_rtk_fusion {

RtkQuality EvaluateRtkFix(const sensor_msgs::msg::NavSatFix& fix, const RtkConfig& config) {
  RtkQuality quality;

  if (fix.status.status <= 0) {
    quality.reason = "invalid_status";
    return quality;
  }

  if (fix.latitude == 0.0 && fix.longitude == 0.0) {
    quality.reason = "zero_latitude_longitude";
    return quality;
  }

  if (!std::isfinite(fix.latitude) || !std::isfinite(fix.longitude) ||
      !std::isfinite(fix.altitude)) {
    quality.reason = "non_finite_coordinate";
    return quality;
  }

  if (fix.status.status == 4) {
    quality.status_text = "Fixed";
    quality.noise_xy = config.fixed_noise_xy;
    quality.noise_z = config.fixed_noise_z;
  } else if (fix.status.status == 5) {
    quality.status_text = "Float";
    quality.noise_xy = config.float_noise_xy;
    quality.noise_z = config.float_noise_z;
  } else if (fix.status.status == 1) {
    quality.reason = "single_status";
    return quality;
  } else {
    quality.reason = "unsupported_status";
    return quality;
  }

  const double cov_x = fix.position_covariance[0];
  const double cov_y = fix.position_covariance[4];
  const double cov_z = fix.position_covariance[8];
  quality.cov_trace = cov_x + cov_y + cov_z;
  const bool covariance_all_zero = cov_x == 0.0 && cov_y == 0.0 && cov_z == 0.0;

  if (!covariance_all_zero && quality.cov_trace > config.cov_trace_threshold) {
    quality.reason = "covariance_too_large";
    return quality;
  }

  quality.accepted = true;
  quality.reason = covariance_all_zero ? "accepted_zero_covariance" : "accepted";
  return quality;
}

}  // namespace lidar_imu_rtk_fusion
