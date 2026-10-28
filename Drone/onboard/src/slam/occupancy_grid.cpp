#include "slam/occupancy_grid.hpp"
#include <algorithm>
#include <cstdlib>
#include <cmath>

namespace nidar {
namespace slam {

// Log-odds update values
constexpr int8_t LOG_ODDS_FREE_UPDATE = -3;
constexpr int8_t LOG_ODDS_OCC_UPDATE  = 18;
constexpr int8_t LOG_ODDS_MAX         = 120;
constexpr int8_t LOG_ODDS_MIN         = -120;

OccupancyGrid::OccupancyGrid(int width_cells, int height_cells, float resolution_m,
                             float origin_x_m, float origin_y_m)
    : width_(width_cells), height_(height_cells), resolution_(resolution_m),
      origin_x_(origin_x_m), origin_y_(origin_y_m) {
    clear();
}

void OccupancyGrid::clear() {
    size_t total = static_cast<size_t>(width_) * height_;
    log_odds_.assign(total, 0);
    cells_.assign(total, CELL_UNKNOWN);
}

bool OccupancyGrid::worldToGrid(float wx, float wy, int& gx, int& gy) const {
    gx = static_cast<int>((wx - origin_x_) / resolution_);
    gy = static_cast<int>((wy - origin_y_) / resolution_);
    return isInside(gx, gy);
}

void OccupancyGrid::gridToWorld(int gx, int gy, float& wx, float& wy) const {
    wx = origin_x_ + (gx + 0.5f) * resolution_;
    wy = origin_y_ + (gy + 0.5f) * resolution_;
}

bool OccupancyGrid::isInside(int gx, int gy) const {
    return (gx >= 0 && gx < width_ && gy >= 0 && gy < height_);
}

uint8_t OccupancyGrid::getCell(int gx, int gy) const {
    if (!isInside(gx, gy)) return CELL_UNKNOWN;
    return cells_[gy * width_ + gx];
}

void OccupancyGrid::setCell(int gx, int gy, uint8_t val) {
    if (isInside(gx, gy)) {
        cells_[gy * width_ + gx] = val;
    }
}

float OccupancyGrid::getProbability(float wx, float wy) const {
    float fx = (wx - origin_x_) / resolution_ - 0.5f;
    float fy = (wy - origin_y_) / resolution_ - 0.5f;

    int x0 = static_cast<int>(std::floor(fx));
    int y0 = static_cast<int>(std::floor(fy));
    int x1 = x0 + 1;
    int y1 = y0 + 1;

    if (x0 < 0 || x1 >= width_ || y0 < 0 || y1 >= height_) {
        return 0.5f; // Unknown prior
    }

    float dx = fx - x0;
    float dy = fy - y0;

    auto cellProb = [this](int x, int y) -> float {
        uint8_t c = cells_[y * width_ + x];
        if (c == CELL_OCCUPIED) return 1.0f;
        if (c == CELL_FREE) return 0.0f;
        return 0.5f;
    };

    float p00 = cellProb(x0, y0);
    float p10 = cellProb(x1, y0);
    float p01 = cellProb(x0, y1);
    float p11 = cellProb(x1, y1);

    // Bilinear interpolation
    return (1.0f - dx) * (1.0f - dy) * p00 +
           dx * (1.0f - dy) * p10 +
           (1.0f - dx) * dy * p01 +
           dx * dy * p11;
}

std::pair<float, float> OccupancyGrid::getGradient(float wx, float wy) const {
    float fx = (wx - origin_x_) / resolution_ - 0.5f;
    float fy = (wy - origin_y_) / resolution_ - 0.5f;

    int x0 = static_cast<int>(std::floor(fx));
    int y0 = static_cast<int>(std::floor(fy));
    int x1 = x0 + 1;
    int y1 = y0 + 1;

    if (x0 < 0 || x1 >= width_ || y0 < 0 || y1 >= height_) {
        return {0.0f, 0.0f};
    }

    float dx = fx - x0;
    float dy = fy - y0;

    auto cellProb = [this](int x, int y) -> float {
        uint8_t c = cells_[y * width_ + x];
        if (c == CELL_OCCUPIED) return 1.0f;
        if (c == CELL_FREE) return 0.0f;
        return 0.5f;
    };

    float p00 = cellProb(x0, y0);
    float p10 = cellProb(x1, y0);
    float p01 = cellProb(x0, y1);
    float p11 = cellProb(x1, y1);

    // Derivative along x and y divided by resolution
    float grad_x = ((1.0f - dy) * (p10 - p00) + dy * (p11 - p01)) / resolution_;
    float grad_y = ((1.0f - dx) * (p01 - p00) + dx * (p11 - p10)) / resolution_;

    return {grad_x, grad_y};
}

void OccupancyGrid::updateRay(float start_x, float start_y, float end_x, float end_y) {
    int x0, y0, x1, y1;
    if (!worldToGrid(start_x, start_y, x0, y0)) return;
    if (!worldToGrid(end_x, end_y, x1, y1)) return;

    // Bresenham's Line Algorithm
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    int curr_x = x0;
    int curr_y = y0;

    while (curr_x != x1 || curr_y != y1) {
        if (isInside(curr_x, curr_y)) {
            size_t idx = curr_y * width_ + curr_x;
            int new_val = log_odds_[idx] + LOG_ODDS_FREE_UPDATE;
            log_odds_[idx] = static_cast<int8_t>(std::max<int>(LOG_ODDS_MIN, new_val));
            cells_[idx] = (log_odds_[idx] < -10) ? CELL_FREE : CELL_UNKNOWN;
        }

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            curr_x += sx;
        }
        if (e2 < dx) {
            err += dx;
            curr_y += sy;
        }
    }

    // Endpoint is obstacle
    if (isInside(x1, y1)) {
        size_t idx = y1 * width_ + x1;
        int new_val = log_odds_[idx] + LOG_ODDS_OCC_UPDATE;
        log_odds_[idx] = static_cast<int8_t>(std::min<int>(LOG_ODDS_MAX, new_val));
        if (log_odds_[idx] > 10) {
            cells_[idx] = CELL_OCCUPIED;
        }
    }
}

std::vector<uint8_t> OccupancyGrid::generateInflatedCostmap(float inflation_radius_m) const {
    std::vector<uint8_t> costmap = cells_;
    int r_cells = static_cast<int>(std::ceil(inflation_radius_m / resolution_));

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (cells_[y * width_ + x] == CELL_OCCUPIED) {
                // Circular dilation
                for (int dy = -r_cells; dy <= r_cells; ++dy) {
                    for (int dx = -r_cells; dx <= r_cells; ++dx) {
                        if (dx * dx + dy * dy <= r_cells * r_cells) {
                            int nx = x + dx;
                            int ny = y + dy;
                            if (isInside(nx, ny)) {
                                size_t n_idx = ny * width_ + nx;
                                if (costmap[n_idx] != CELL_OCCUPIED) {
                                    costmap[n_idx] = CELL_INFLATED;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return costmap;
}

std::vector<uint8_t> OccupancyGrid::compressRLE() const {
    std::vector<uint8_t> compressed;
    if (cells_.empty()) return compressed;

    compressed.reserve(cells_.size() / 4);
    uint8_t current_val = cells_[0];
    uint8_t count = 0;

    for (size_t i = 0; i < cells_.size(); ++i) {
        if (cells_[i] == current_val && count < 255) {
            count++;
        } else {
            compressed.push_back(count);
            compressed.push_back(current_val);
            current_val = cells_[i];
            count = 1;
        }
    }
    compressed.push_back(count);
    compressed.push_back(current_val);

    return compressed;
}

} // namespace slam
} // namespace nidar
