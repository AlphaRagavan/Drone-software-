#pragma once

#include <vector>
#include <cstdint>
#include <cmath>
#include <tuple>

namespace nidar {
namespace slam {

// 8-bit cell occupancy classification
constexpr uint8_t CELL_UNKNOWN   = 127;
constexpr uint8_t CELL_FREE      = 0;
constexpr uint8_t CELL_OCCUPIED  = 255;
constexpr uint8_t CELL_INFLATED  = 200;

struct GridPose {
    float x{0.0f};       // World X in meters
    float y{0.0f};       // World Y in meters
    float yaw_rad{0.0f}; // Heading angle in radians
};

class OccupancyGrid {
public:
    OccupancyGrid(int width_cells = 360, int height_cells = 360, float resolution_m = 0.05f,
                  float origin_x_m = 0.0f, float origin_y_m = 0.0f);

    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    float getResolution() const { return resolution_; }
    float getOriginX() const { return origin_x_; }
    float getOriginY() const { return origin_y_; }

    void clear();

    // Coordinate conversions
    bool worldToGrid(float wx, float wy, int& gx, int& gy) const;
    void gridToWorld(int gx, int gy, float& wx, float& wy) const;
    bool isInside(int gx, int gy) const;

    // Direct cell access
    uint8_t getCell(int gx, int gy) const;
    void setCell(int gx, int gy, uint8_t val);

    // Continuous map sampling and gradients for Hector SLAM scan matcher
    float getProbability(float wx, float wy) const;
    std::pair<float, float> getGradient(float wx, float wy) const;

    // Ray-casting beam update
    void updateRay(float start_x, float start_y, float end_x, float end_y);

    // Costmap generation for pathfinding
    std::vector<uint8_t> generateInflatedCostmap(float inflation_radius_m = 0.25f) const;

    // Direct read of raw cell buffer
    const std::vector<uint8_t>& getRawCells() const { return cells_; }

    // Run-Length Encoding (RLE) compression for high-frequency TCP transmission to GCS
    std::vector<uint8_t> compressRLE() const;

private:
    int width_;
    int height_;
    float resolution_;
    float origin_x_;
    float origin_y_;

    // Internal log-odds grid for Bayesian update: 0 = unknown, negative = free, positive = occupied
    std::vector<int8_t> log_odds_;
    // Pre-computed 8-bit visualization grid: 0 (free), 127 (unknown), 255 (occupied)
    std::vector<uint8_t> cells_;
};

} // namespace slam
} // namespace nidar
