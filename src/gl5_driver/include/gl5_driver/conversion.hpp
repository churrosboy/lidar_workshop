#pragma once
#include <soslabTypedef.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace gl5_driver {
struct ScanConfig {
  std::string frame_id = "laser";
  double range_min = 0.0;
  double range_max = 60.0;  // SDK parser upper bound, not a guaranteed device range.
  double angle_offset = 0.0;
  double fov_deg = 270.0;  // Horizontal FOV: GL5 = 270, GL3 = 180.
};
void validate_scan_config(const ScanConfig& config);

sensor_msgs::msg::LaserScan make_scan(const soslab::FrameData& frame, const ScanConfig& config,
                                      const builtin_interfaces::msg::Time& stamp, double scan_time);
sensor_msgs::msg::PointCloud2 make_cloud(const sensor_msgs::msg::LaserScan& scan);
}  // namespace gl5_driver
