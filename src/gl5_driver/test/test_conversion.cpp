#include "gl5_driver/conversion.hpp"
#include <gtest/gtest.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>

static soslab::FrameData frame(std::vector<uint32_t> ranges) {
  soslab::FrameData f;
  f.depth[0] = ranges; f.intensity[0].assign(ranges.size(), 42);
  return f;
}
TEST(Conversion, UnitsAnglesAndCloud) {
  auto scan = gl5_driver::make_scan(frame({1000, 2000, 3000}), {}, builtin_interfaces::msg::Time(), 0.05);
  EXPECT_FLOAT_EQ(scan.ranges[1], 2.0f);
  EXPECT_NEAR(scan.angle_min, -3 * M_PI / 4, 1e-6);
  EXPECT_NEAR(scan.angle_max, 3 * M_PI / 4, 1e-6);
  EXPECT_NEAR(scan.angle_min + scan.angle_increment, 0, 1e-6);
  EXPECT_FLOAT_EQ(scan.scan_time, 0.05f);
  EXPECT_FLOAT_EQ(scan.time_increment, 0);
  auto cloud = gl5_driver::make_cloud(scan);
  EXPECT_EQ(cloud.width, 3u); EXPECT_EQ(cloud.height, 1u);
  EXPECT_EQ(cloud.row_step, 48u); EXPECT_EQ(cloud.header.frame_id, "laser");
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y");
  EXPECT_NEAR(*x, -std::sqrt(0.5), 1e-6); EXPECT_NEAR(*y, -std::sqrt(0.5), 1e-6);
  ++x; ++y; EXPECT_NEAR(*x, 2, 1e-6); EXPECT_NEAR(*y, 0, 1e-6);
}
TEST(Conversion, InvalidRangesBecomeNonfinite) {
  auto scan = gl5_driver::make_scan(frame({0, 1000, 60001}), {}, builtin_interfaces::msg::Time(), 0);
  EXPECT_TRUE(std::isinf(scan.ranges[0])); EXPECT_TRUE(std::isinf(scan.ranges[2]));
  auto cloud = gl5_driver::make_cloud(scan);
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
  EXPECT_TRUE(std::isnan(*x)); EXPECT_FALSE(cloud.is_dense);
}
TEST(Conversion, RejectsEmptyAndMismatchedArrays) {
  EXPECT_THROW(gl5_driver::make_scan(frame({}), {}, builtin_interfaces::msg::Time(), 0), std::invalid_argument);
  EXPECT_THROW(gl5_driver::make_scan(frame({1}), {}, builtin_interfaces::msg::Time(), 0), std::invalid_argument);
  auto f = frame({1, 2}); f.intensity[0].clear();
  EXPECT_THROW(gl5_driver::make_scan(f, {}, builtin_interfaces::msg::Time(), 0), std::invalid_argument);
}
TEST(Conversion, UsesActualArrayLengthAndOffset) {
  auto f = frame({1000, 1000, 1000, 1000, 1000}); f.cols = 1500;
  gl5_driver::ScanConfig config; config.angle_offset = 0.2;
  auto scan = gl5_driver::make_scan(f, config, builtin_interfaces::msg::Time(), 0);
  EXPECT_EQ(scan.ranges.size(), 5u);
  EXPECT_NEAR(scan.angle_min + 2 * scan.angle_increment, 0.2, 1e-6);
}
TEST(Conversion, RangeLimitsAndConfiguration) {
  gl5_driver::ScanConfig config; config.range_min = 0.1; config.range_max = 5.0;
  auto scan = gl5_driver::make_scan(frame({50, 1000, 5500}), config, builtin_interfaces::msg::Time(), 0);
  EXPECT_TRUE(std::isinf(scan.ranges.front())); EXPECT_TRUE(std::isinf(scan.ranges.back()));
  config.range_max = 0;
  EXPECT_THROW(gl5_driver::make_scan(frame({1, 2}), config, builtin_interfaces::msg::Time(), 0), std::invalid_argument);
}
TEST(Conversion, Gl3FovIs180Degrees) {
  gl5_driver::ScanConfig config; config.fov_deg = 180.0;
  auto scan = gl5_driver::make_scan(frame({1000, 2000, 3000}), config, builtin_interfaces::msg::Time(), 0);
  EXPECT_NEAR(scan.angle_min, -M_PI / 2, 1e-6);
  EXPECT_NEAR(scan.angle_max, M_PI / 2, 1e-6);
  EXPECT_NEAR(scan.angle_min + scan.angle_increment, 0, 1e-6);
  auto cloud = gl5_driver::make_cloud(scan);
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y");
  EXPECT_NEAR(*x, 0, 1e-6); EXPECT_NEAR(*y, -1, 1e-6);
  config.fov_deg = 0;
  EXPECT_THROW(gl5_driver::make_scan(frame({1, 2}), config, builtin_interfaces::msg::Time(), 0), std::invalid_argument);
}
