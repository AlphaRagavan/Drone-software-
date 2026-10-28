#include "lidar/rplidar_driver.hpp"
#include <iostream>
#include <chrono>

namespace nidar {
namespace lidar {

NidarRplidar::NidarRplidar(const std::string& serial_port, uint32_t baudrate)
    : serial_port_(serial_port), baudrate_(baudrate) {}

NidarRplidar::~NidarRplidar() {
    stop();
}

bool NidarRplidar::initialize() {
    driver_ = sl::ILidarDriver::create();
    if (!driver_) {
        std::cerr << "[RPLIDAR] Failed to allocate ILidarDriver instance." << std::endl;
        return false;
    }

    channel_ = sl::createSerialPortChannel(serial_port_.c_str(), baudrate_);
    if (!channel_) {
        std::cerr << "[RPLIDAR] Failed to create serial channel." << std::endl;
        return false;
    }

    sl_result res = driver_->connect(channel_);
    if (SL_IS_FAIL(res)) {
        std::cerr << "[RPLIDAR] Connection to " << serial_port_ << " failed." << std::endl;
        return false;
    }

    // Health check
    sl_lidar_response_device_health_t health_info;
    res = driver_->getHealth(health_info);
    if (SL_IS_OK(res)) {
        if (health_info.status == SL_LIDAR_STATUS_ERROR) {
            std::cerr << "[RPLIDAR] Internal hardware error code: " << health_info.error_code << std::endl;
            healthy_.store(false);
            return false;
        }
        std::cout << "[RPLIDAR] Hardware health status OK (0x" << std::hex << (int)health_info.status << std::dec << ")." << std::endl;
        healthy_.store(true);
    } else {
        std::cerr << "[RPLIDAR] Failed to read health status." << std::endl;
        healthy_.store(false);
    }

    // Device info
    sl_lidar_response_device_info_t dev_info;
    if (SL_IS_OK(driver_->getDeviceInfo(dev_info))) {
        std::cout << "[RPLIDAR] Connected model: " << (int)dev_info.model 
                  << " Firmware: " << (dev_info.firmware_version >> 8) << "." << (dev_info.firmware_version & 0xFF)
                  << " Hardware rev: " << (int)dev_info.hardware_version << std::endl;
    }

    return true;
}

bool NidarRplidar::startScanning(ScanCallback callback) {
    if (!driver_ || running_.load()) return false;

    scan_callback_ = callback;

    // Start motor spin
    driver_->setMotorSpeed(600);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Start 360-degree scan
    sl_result res = driver_->startScan(false, true);
    if (SL_IS_FAIL(res)) {
        std::cerr << "[RPLIDAR] Failed to start scan mode." << std::endl;
        return false;
    }

    running_.store(true);
    worker_thread_ = std::thread(&NidarRplidar::captureWorker, this);
    std::cout << "[RPLIDAR] Continuous 360 HQ scan worker thread launched." << std::endl;
    return true;
}

void NidarRplidar::stop() {
    if (!running_.load()) return;
    running_.store(false);

    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }

    if (driver_) {
        driver_->stop();
        driver_->setMotorSpeed(0);
        driver_->disconnect();
        delete driver_;
        driver_ = nullptr;
    }

    std::cout << "[RPLIDAR] Driver safely halted." << std::endl;
}

void NidarRplidar::parseHqNodes(const sl_lidar_response_measurement_node_hq_t* nodes, size_t count, LaserScan& scan) {
    scan.points.clear();
    scan.points.reserve(count);

    auto now = std::chrono::steady_clock::now();
    scan.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();

    for (size_t i = 0; i < count; ++i) {
        const auto& node = nodes[i];

        // Raw angle parsing: angle_z_q14 format: degrees = angle_z_q14 * 90.0 / 16384.0
        float angle_deg = node.angle_z_q14 * 90.0f / 16384.0f;
        // Normalize to [-180, 180] degrees
        if (angle_deg > 180.0f) angle_deg -= 360.0f;
        float angle_rad = angle_deg * (3.141592653589793f / 180.0f);

        // Distance in mm = dist_mm_q2 / 4.0
        float dist_m = (node.dist_mm_q2 / 4.0f) / 1000.0f;

        // Filter invalid ranges (minimum blind zone 0.15m, maximum arena corridor range 12.0m)
        if (dist_m < scan.range_min || dist_m > scan.range_max) {
            continue;
        }

        // Quality check (0 indicates no return / low reflectivity)
        if (node.quality == 0 && dist_m > 0.0f) {
            continue;
        }

        LidarPoint pt;
        pt.angle_rad = angle_rad;
        pt.range_m = dist_m;
        pt.x = dist_m * std::cos(angle_rad);
        pt.y = dist_m * std::sin(angle_rad);
        pt.quality = node.quality;

        scan.points.push_back(pt);
    }
}

void NidarRplidar::captureWorker() {
    constexpr size_t MAX_NODES = 8192;
    std::vector<sl_lidar_response_measurement_node_hq_t> node_buffer(MAX_NODES);

    while (running_.load()) {
        size_t count = MAX_NODES;

        // Continuous high-quality 360-degree scan grabbing
        sl_result res = driver_->grabScanDataHq(node_buffer.data(), count, 1000);

        if (SL_IS_OK(res) && count > 0) {
            // Ascend scan data by angle for consistent sweep ordering
            driver_->ascendScanData(node_buffer.data(), count);

            LaserScan scan;
            parseHqNodes(node_buffer.data(), count, scan);

            {
                std::lock_guard<std::mutex> lock(scan_mutex_);
                latest_scan_ = scan;
            }

            if (scan_callback_) {
                scan_callback_(scan);
            }
        } else {
            // If physical device temporarily times out, sleep briefly
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

LaserScan NidarRplidar::getLatestScan() {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    return latest_scan_;
}

} // namespace lidar
} // namespace nidar
