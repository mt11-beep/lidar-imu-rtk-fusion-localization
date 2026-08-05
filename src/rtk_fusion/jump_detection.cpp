#include "rtk_types.hpp"

#include <algorithm>
#include <cmath>

namespace lidar_imu_rtk_fusion {

bool RejectRtkJump(const Eigen::Vector3d& current_enu, double timestamp, const RtkConfig& config,
                   JumpState& state, double* delta_out, double* threshold_out) {
  if (!state.has_last) {
    state.has_last = true;
    state.last_enu = current_enu;
    state.last_time = timestamp;
    return false;
  }

  const double dt = timestamp - state.last_time;
  const double delta = (current_enu - state.last_enu).norm();
  const double threshold = config.jump_vmax * std::max(dt, 0.0) + config.jump_margin;

  if (delta_out != nullptr) *delta_out = delta;
  if (threshold_out != nullptr) *threshold_out = threshold;

  if (delta > threshold) {
    return true;
  }

  state.last_enu = current_enu;
  state.last_time = timestamp;
  return false;
}

}  // namespace lidar_imu_rtk_fusion
