#include "gl5_driver/conversion.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace gl5_driver {

// 프레임, 거리 범위, 각도 오프셋, 시야각 유효성 검사
void validate_scan_config(const ScanConfig& config) {
  if (config.frame_id.empty() || !std::isfinite(config.range_min) ||
      !std::isfinite(config.range_max) || config.range_min < 0 ||
      config.range_max <= config.range_min || config.range_max > 60 ||
      !std::isfinite(config.angle_offset) || !std::isfinite(config.fov_deg) ||
      config.fov_deg <= 0 || config.fov_deg > 360) {
    throw std::invalid_argument("Invalid scan configuration");
  }
}

// SDK 프레임 → LaserScan 변환 (mm→m, 시야각 기준 각도 계산)
sensor_msgs::msg::LaserScan make_scan(const soslab::FrameData& frame, const ScanConfig& config,
                                      const builtin_interfaces::msg::Time& stamp,
                                      double scan_time) {
  if (frame.depth.empty() || frame.depth[0].size() < 2 || frame.intensity.empty() ||
      frame.intensity[0].size() != frame.depth[0].size()) {
    throw std::invalid_argument("Empty or mismatched GL5 range/intensity arrays");
  }
  validate_scan_config(config);
  if (!std::isfinite(scan_time) || scan_time < 0) {
    throw std::invalid_argument("Invalid scan configuration");
  }
  constexpr double pi = 3.14159265358979323846;
  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp = stamp;
  scan.header.frame_id = config.frame_id;
  const size_t count = frame.depth[0].size();
  const double fov = config.fov_deg * pi / 180.0;
  scan.angle_min = -0.5 * fov + config.angle_offset;
  scan.angle_increment = fov / (count - 1);
  scan.angle_max = scan.angle_min + (count - 1) * scan.angle_increment;
  scan.scan_time = scan_time;
  scan.time_increment = 0.0;
  scan.range_min = config.range_min;
  scan.range_max = config.range_max;
  scan.ranges.reserve(count);
  scan.intensities.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    const float distance = frame.depth[0][i] * 0.001f;
    scan.ranges.push_back(frame.depth[0][i] > 0 && distance >= scan.range_min &&
                                  distance <= scan.range_max
                              ? distance
                              : std::numeric_limits<float>::infinity());
    scan.intensities.push_back(frame.intensity[0][i]);
  }
  return scan;
}

// LaserScan → xyz+intensity PointCloud2 변환
sensor_msgs::msg::PointCloud2 make_cloud(const sensor_msgs::msg::LaserScan& scan) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = scan.header;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
                                sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(scan.ranges.size());
  cloud.is_dense = false;
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z"),
      intensity(cloud, "intensity");
  for (size_t i = 0; i < scan.ranges.size(); ++i, ++x, ++y, ++z, ++intensity) {
    const double angle = scan.angle_min + i * scan.angle_increment;
    const bool valid = std::isfinite(scan.ranges[i]);
    *x = valid ? scan.ranges[i] * std::cos(angle) : std::numeric_limits<float>::quiet_NaN();
    *y = valid ? scan.ranges[i] * std::sin(angle) : std::numeric_limits<float>::quiet_NaN();
    *z = valid ? 0.0f : std::numeric_limits<float>::quiet_NaN();
    *intensity = i < scan.intensities.size() ? scan.intensities[i] : 0.0f;
  }
  return cloud;
}
}
