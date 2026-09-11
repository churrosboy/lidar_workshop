#include <Lidar.h>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>
#include <limits>

namespace {
volatile std::sig_atomic_t stopped = 0;
void signal_handler(int) { stopped = 1; }
}

// gl5_receive SENSOR_IP SENSOR_PORT PC_IP PC_PORT [SECONDS] [OUTPUT_JSON]
int main(int argc, char** argv) {
  if (argc < 5 || argc > 7) {
    std::cerr << "Usage: gl5_receive SENSOR_IP SENSOR_PORT PC_IP PC_PORT [SECONDS=30] [OUTPUT_JSON]\n";
    return 2;
  }
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  try {
    soslab::lidarParameters params;
    params.lidarTypeValue = soslab::lidarType::GL5;
    params.lidarIP = argv[1]; params.lidarPort = std::stoi(argv[2]);
    params.pcIP = argv[3]; params.pcPort = std::stoi(argv[4]);
    const double seconds = argc > 5 ? std::stod(argv[5]) : 30.0;
    if (params.lidarPort < 1 || params.lidarPort > 65535 || params.pcPort < 0 ||
        params.pcPort > 65535 || !std::isfinite(seconds) || seconds <= 0) return 2;
    std::mutex mutex;
    size_t frames = 0, valid_samples = 0, last_count = 0;
    double nearest = std::numeric_limits<double>::infinity(), farthest = 0, max_gap = 0;
    using Clock = std::chrono::steady_clock;
    auto begin = Clock::now(), last = begin;
    std::vector<uint32_t> latest;
    // Register before connect: the SDK callback must not be changed while workers run.
    soslab::Lidar lidar(params);
    lidar.registerGetDataCallBack([&](std::shared_ptr<const soslab::FrameData> frame) {
      if (!frame || frame->depth.empty() || frame->depth[0].size() < 2) return;
      std::lock_guard<std::mutex> lock(mutex);
      auto now = Clock::now();
      max_gap = std::max(max_gap, std::chrono::duration<double>(now - last).count());
      last = now; ++frames; latest = frame->depth[0]; last_count = latest.size();
      for (auto d : latest) if (d > 0 && d <= 60000) {
        ++valid_samples; nearest = std::min(nearest, d / 1000.0); farthest = std::max(farthest, d / 1000.0);
      }
    });
    if (!lidar.connectLidar()) { std::cerr << "CONNECT_FAILED\n"; return 3; }
    { std::lock_guard<std::mutex> lock(mutex); begin = last = Clock::now(); }
    const bool started = lidar.streamStart();
    bool timeout = false;
    if (started) {
      while (!stopped && std::chrono::duration<double>(Clock::now() - begin).count() < seconds) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::lock_guard<std::mutex> lock(mutex);
        std::cout << "frames=" << frames << " samples=" << last_count
                  << " valid_total=" << valid_samples << std::endl;
        if (Clock::now() - last > std::chrono::seconds(5)) { timeout = true; break; }
      }
    }
    const double elapsed = std::chrono::duration<double>(Clock::now() - begin).count();
    // Try to stop even if the start acknowledgement was lost.
    const bool stop_ack = lidar.streamStop();
    lidar.disconnectLidar();
    lidar.unregisterGetDataCallBack();
    const bool passed = started && stop_ack && !timeout && !stopped && frames > 0 && valid_samples > 0;
    nlohmann::json report = {{"passed", passed}, {"sensor_ip", params.lidarIP},
      {"sensor_port", params.lidarPort}, {"pc_ip", params.pcIP}, {"pc_port", params.pcPort},
      {"start_ack", started}, {"stop_ack", stop_ack}, {"frames", frames},
      {"elapsed_seconds", elapsed}, {"average_hz", frames / elapsed},
      {"max_gap_seconds", max_gap}, {"samples_per_frame", last_count},
      {"valid_samples", valid_samples}, {"min_range_m", nearest}, {"max_range_m", farthest},
      {"last_ranges_mm", latest}, {"timestamp_source", "PC receive time"}};
    if (argc == 7) { std::ofstream out(argv[6]); if (!out) throw std::runtime_error("Cannot write report"); out << report.dump(2) << '\n'; }
    report.erase("last_ranges_mm");
    std::cout << report.dump(2) << std::endl;
    return passed ? 0 : 4;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 2; }
}
