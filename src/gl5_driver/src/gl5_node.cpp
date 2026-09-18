#include "gl5_driver/conversion.hpp"
#include <Lidar.h>
#include <rclcpp/rclcpp.hpp>
#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

using Steady = std::chrono::steady_clock;
static int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Steady::now().time_since_epoch())
      .count();
}

class Gl5Node : public rclcpp::Node {
 public:
  Gl5Node() : Node("gl5_node") {
    configure_parameters();
    validate_network_parameters();
    gl5_driver::validate_scan_config(config_);
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("points", rclcpp::SensorDataQoS());
    lidar_ = std::make_unique<soslab::Lidar>(params_);
    lidar_->registerGetDataCallBack([this](auto frame) { receive(frame); });
  }
  ~Gl5Node() override {
    stop();
  }

  bool start() {
    RCLCPP_INFO(get_logger(), "%s UDP %s:%d -> PC %s:%d (FOV %.0f deg)", lidar_type_.c_str(),
                params_.lidarIP.c_str(), params_.lidarPort, params_.pcIP.c_str(), params_.pcPort,
                config_.fov_deg);
    if (!lidar_->connectLidar()) {
      RCLCPP_ERROR(get_logger(), "SDK connection failed");
      return false;
    }
    connected_ = true;
    last_frame_ns_ = steady_ns();
    if (!lidar_->streamStart()) {
      RCLCPP_ERROR(get_logger(), "GL5 stream command not acknowledged");
      return false;
    }
    RCLCPP_INFO(get_logger(),
                "Stream acknowledged; waiting for frames. Timestamps use PC receive time.");
    watchdog_ =
        create_wall_timer(std::chrono::seconds(1), std::bind(&Gl5Node::check_frame_timeout, this));
    return true;
  }
  bool failed() const {
    return failed_;
  }

  void stop() {
    stopping_ = true;
    if (watchdog_) {
      watchdog_->cancel();
    }
    if (connected_) {
      if (!lidar_->streamStop()) {
        RCLCPP_WARN(get_logger(), "Stream stop acknowledgement missing");
      }
      lidar_->disconnectLidar();  // Joins callback worker before node members are destroyed.
      lidar_->unregisterGetDataCallBack();
      connected_ = false;
    }
  }

 private:
  void configure_parameters() {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    // Sensor model: "GL5" (270 deg, 1500 pts) or "GL3" (180 deg, 1000 pts).
    lidar_type_ = declare_parameter<std::string>("lidar_type", "GL3", descriptor);
    if (lidar_type_ == "GL5") {
      params_.lidarTypeValue = soslab::lidarType::GL5;
      config_.fov_deg = 270.0;
    } else if (lidar_type_ == "GL3") {
      params_.lidarTypeValue = soslab::lidarType::GL3;
      config_.fov_deg = 180.0;
    } else {
      throw std::invalid_argument("lidar_type must be \"GL5\" or \"GL3\", got \"" + lidar_type_ +
                                  "\"");
    }
    params_.lidarIP = declare_parameter<std::string>("sensor_ip", "10.110.1.2", descriptor);
    params_.lidarPort = declare_parameter<int>("sensor_port", 2000, descriptor);
    params_.pcIP = declare_parameter<std::string>("pc_ip", "10.110.1.3", descriptor);
    params_.pcPort = declare_parameter<int>("pc_port", 3000, descriptor);
    config_.frame_id = declare_parameter<std::string>("frame_id", "laser", descriptor);
    config_.range_min = declare_parameter<double>("range_min", 0.0, descriptor);
    config_.range_max = declare_parameter<double>("range_max", 60.0, descriptor);
    config_.angle_offset = declare_parameter<double>("angle_offset", 0.0, descriptor);
  }

  void validate_network_parameters() const {
    in_addr address{};
    if (inet_pton(AF_INET, params_.lidarIP.c_str(), &address) != 1 ||
        inet_pton(AF_INET, params_.pcIP.c_str(), &address) != 1 || params_.lidarPort < 1 ||
        params_.lidarPort > 65535 || params_.pcPort < 0 || params_.pcPort > 65535) {
      throw std::invalid_argument("Invalid IPv4 address or UDP port");
    }
  }

  void check_frame_timeout() {
    if (steady_ns() - last_frame_ns_.load() > 5'000'000'000LL) {
      RCLCPP_ERROR(get_logger(), "No valid GL5 frames for 5 seconds; terminating");
      failed_ = true;
    } else {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Received %lu frames", frames_.load());
    }
  }

  void receive(const std::shared_ptr<const soslab::FrameData>& frame) {
    if (stopping_ || !rclcpp::ok() || !frame) {
      return;
    }
    try {
      const auto received = steady_ns();
      const double period = frames_ > 0 ? (received - last_frame_ns_.load()) * 1e-9 : 0.0;
      auto scan = gl5_driver::make_scan(*frame, config_, now(), period);
      auto cloud = gl5_driver::make_cloud(scan);
      if (stopping_ || !rclcpp::ok()) {
        return;
      }
      scan_pub_->publish(scan);
      cloud_pub_->publish(cloud);
      last_frame_ns_ = received;
      ++frames_;
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Frame rejected: %s", e.what());
    }
  }

  soslab::lidarParameters params_;
  std::string lidar_type_;
  gl5_driver::ScanConfig config_;
  std::unique_ptr<soslab::Lidar> lidar_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  std::atomic<int64_t> last_frame_ns_{0};
  std::atomic<unsigned long> frames_{0};
  std::atomic<bool> stopping_{false}, failed_{false};
  bool connected_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    auto node = std::make_shared<Gl5Node>();
    if (!node->start()) {
      code = 3;
    } else {
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node);
      while (rclcpp::ok() && !node->failed()) {
        executor.spin_once(std::chrono::milliseconds(100));
      }
      if (node->failed()) {
        code = 4;
      }
      executor.remove_node(node);
    }
    node->stop();
    node.reset();
  } catch (const std::exception& e) {
    std::cerr << "GL5 fatal: " << e.what() << '\n';
    code = 2;
  }
  rclcpp::shutdown();
  return code;
}
