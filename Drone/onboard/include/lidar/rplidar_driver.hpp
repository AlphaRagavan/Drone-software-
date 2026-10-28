#pragma once

#include "sl_lidar.h"
#include "lidar/lidar_types.hpp"
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>

namespace nidar {
namespace lidar {

class NidarRplidar {
public:
    using ScanCallback = std::function<void(const LaserScan&)>;

    NidarRplidar(const std::string& serial_port = "/dev/ttyUSB0", uint32_t baudrate = 115200);
    ~NidarRplidar();

    bool initialize();
    bool startScanning(ScanCallback callback = nullptr);
    void stop();

    bool isHealthy() const { return healthy_.load(); }
    LaserScan getLatestScan();

private:
    void captureWorker();
    void parseHqNodes(const sl_lidar_response_measurement_node_hq_t* nodes, size_t count, LaserScan& scan);

    std::string serial_port_;
    uint32_t baudrate_;

    sl::ILidarDriver* driver_{nullptr};
    sl::IChannel* channel_{nullptr};

    std::atomic<bool> running_{false};
    std::atomic<bool> healthy_{false};
    std::thread worker_thread_;

    std::mutex scan_mutex_;
    LaserScan latest_scan_;
    ScanCallback scan_callback_;
};

} // namespace lidar
} // namespace nidar
