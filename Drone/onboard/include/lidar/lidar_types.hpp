#pragma once

#include <vector>
#include <cstdint>
#include <cmath>

namespace nidar {
namespace lidar {

struct LidarPoint {
    float x;          // meters, forward positive
    float y;          // meters, left positive
    float angle_rad;  // radians [-pi, pi]
    float range_m;    // meters
    uint8_t quality;  // 0-255 signal strength
};

struct LaserScan {
    uint64_t timestamp_us{0};
    float angle_min{-3.14159265f};
    float angle_max{3.14159265f};
    float range_min{0.15f};
    float range_max{12.0f};
    std::vector<LidarPoint> points;
};

} // namespace lidar
} // namespace nidar
