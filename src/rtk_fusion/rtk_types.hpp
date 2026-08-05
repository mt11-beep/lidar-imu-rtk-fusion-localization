#pragma once

#include <Eigen/Core>
#include <nav_msgs/msg/odometry.hpp>
#include <string>

namespace lidar_imu_rtk_fusion {

struct RtkConfig {
  double cov_trace_threshold = 0.10;
  double jump_vmax = 1.0;
  double jump_margin = 0.5;
  double fixed_noise_xy = 0.05;
  double fixed_noise_z = 0.10;
  double float_noise_xy = 0.30;
  double float_noise_z = 0.50;
  double min_factor_distance = 3.0;
  bool use_pose_covariance_gate = false;
  double pose_covariance_threshold = 25.0;
  bool use_gps_elevation = false;
};

struct RtkMeasurement {
  nav_msgs::msg::Odometry odom;
  int status = 0;
  double cov_trace = 0.0;
  double noise_xy = 0.0;
  double noise_z = 0.0;
  std::string status_text;
};

struct RtkQuality {
  bool accepted = false;
  std::string reason;
  std::string status_text;
  double noise_xy = 0.0;
  double noise_z = 0.0;
  double cov_trace = 0.0;
};

struct JumpState {
  bool has_last = false;
  Eigen::Vector3d last_enu = Eigen::Vector3d::Zero();
  double last_time = -1.0;
};

}  // namespace lidar_imu_rtk_fusion
