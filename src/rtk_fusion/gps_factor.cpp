#include "rtk_types.hpp"

#include <cmath>
#include <deque>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/GPSFactor.h>
#include <pcl/point_types.h>
#include <rclcpp/time.hpp>

namespace lidar_imu_rtk_fusion {

gtsam::noiseModel::Diagonal::shared_ptr BuildAdaptiveRtkNoise(const RtkMeasurement& measurement);

bool AddRtkFactor(std::deque<RtkMeasurement>& rtk_buffer, double lidar_end_time,
                  size_t current_keyframe_index, const RtkConfig& config,
                  gtsam::NonlinearFactorGraph& graph, pcl::PointXYZ& last_rtk_factor_point,
                  bool& has_last_rtk_factor_point) {
  while (!rtk_buffer.empty()) {
    const RtkMeasurement measurement = rtk_buffer.front();
    const double rtk_time = rclcpp::Time(measurement.odom.header.stamp).seconds();

    if (rtk_time < lidar_end_time - 0.05) {
      rtk_buffer.pop_front();
      continue;
    }
    if (rtk_time > lidar_end_time + 0.05) {
      return false;
    }

    rtk_buffer.pop_front();
    pcl::PointXYZ current;
    current.x = measurement.odom.pose.pose.position.x;
    current.y = measurement.odom.pose.pose.position.y;
    current.z = config.use_gps_elevation ? measurement.odom.pose.pose.position.z : last_rtk_factor_point.z;

    const double dx = current.x - last_rtk_factor_point.x;
    const double dy = current.y - last_rtk_factor_point.y;
    const double dz = current.z - last_rtk_factor_point.z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (has_last_rtk_factor_point && distance < config.min_factor_distance) {
      continue;
    }

    graph.add(gtsam::GPSFactor(current_keyframe_index,
                               gtsam::Point3(current.x, current.y, current.z),
                               BuildAdaptiveRtkNoise(measurement)));
    last_rtk_factor_point = current;
    has_last_rtk_factor_point = true;
    return true;
  }

  return false;
}

}  // namespace lidar_imu_rtk_fusion
