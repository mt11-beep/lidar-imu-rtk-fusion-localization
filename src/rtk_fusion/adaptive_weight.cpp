#include "rtk_types.hpp"

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/Vector.h>

namespace lidar_imu_rtk_fusion {

gtsam::noiseModel::Diagonal::shared_ptr BuildAdaptiveRtkNoise(const RtkMeasurement& measurement) {
  const double var_x = measurement.noise_xy * measurement.noise_xy;
  const double var_y = measurement.noise_xy * measurement.noise_xy;
  const double var_z = measurement.noise_z * measurement.noise_z;
  return gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) << var_x, var_y, var_z).finished());
}

bool ShouldUseRtkForPoseCovariance(const Eigen::MatrixXd& pose_covariance, const RtkConfig& config) {
  if (!config.use_pose_covariance_gate) {
    return true;
  }
  if (pose_covariance.rows() < 6 || pose_covariance.cols() < 6) {
    return false;
  }
  if (!std::isfinite(pose_covariance(3, 3)) || !std::isfinite(pose_covariance(4, 4))) {
    return false;
  }
  return pose_covariance(3, 3) >= config.pose_covariance_threshold ||
         pose_covariance(4, 4) >= config.pose_covariance_threshold;
}

}  // namespace lidar_imu_rtk_fusion
