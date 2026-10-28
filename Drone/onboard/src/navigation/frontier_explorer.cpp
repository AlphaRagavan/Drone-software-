#include "navigation/frontier_explorer.hpp"
#include <queue>
#include <algorithm>
#include <cmath>

namespace nidar {
namespace navigation {

FrontierExplorer::FrontierExplorer(float min_cluster_size_m)
    : min_cluster_size_m_(min_cluster_size_m) {}

bool FrontierExplorer::isFrontierCell(const slam::OccupancyGrid& grid, int gx, int gy) const {
    if (grid.getCell(gx, gy) != slam::CELL_FREE) {
        return false;
    }

    bool has_unknown_neighbor = false;

    // Check 8-connected neighbors
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            int nx = gx + dx;
            int ny = gy + dy;
            if (!grid.isInside(nx, ny)) continue;

            uint8_t neighbor = grid.getCell(nx, ny);
            if (neighbor == slam::CELL_OCCUPIED) {
                return false; // Reject frontiers immediately touching a wall
            }
            if (neighbor == slam::CELL_UNKNOWN) {
                has_unknown_neighbor = true;
            }
        }
    }
    return has_unknown_neighbor;
}

void FrontierExplorer::clusterFrontiers(const slam::OccupancyGrid& grid,
                                        const std::vector<std::pair<int, int>>& frontier_cells) {
    clusters_.clear();
    int w = grid.getWidth();
    int h = grid.getHeight();
    std::vector<bool> visited(static_cast<size_t>(w) * h, false);

    int cluster_counter = 0;
    int min_cell_count = static_cast<int>(min_cluster_size_m_ / grid.getResolution());

    for (const auto& cell : frontier_cells) {
        int start_gx = cell.first;
        int start_gy = cell.second;
        size_t idx = start_gy * w + start_gx;

        if (visited[idx]) continue;

        // BFS to extract connected frontier region
        std::vector<std::pair<int, int>> cluster;
        std::queue<std::pair<int, int>> q;

        q.push(cell);
        visited[idx] = true;

        while (!q.empty()) {
            auto curr = q.front();
            q.pop();
            cluster.push_back(curr);

            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = curr.first + dx;
                    int ny = curr.second + dy;
                    if (!grid.isInside(nx, ny)) continue;

                    size_t n_idx = ny * w + nx;
                    if (!visited[n_idx] && isFrontierCell(grid, nx, ny)) {
                        visited[n_idx] = true;
                        q.push({nx, ny});
                    }
                }
            }
        }

        if (static_cast<int>(cluster.size()) >= min_cell_count) {
            float sum_x = 0.0f;
            float sum_y = 0.0f;
            for (const auto& pt : cluster) {
                float wx, wy;
                grid.gridToWorld(pt.first, pt.second, wx, wy);
                sum_x += wx;
                sum_y += wy;
            }

            FrontierCluster fc;
            fc.id = ++cluster_counter;
            fc.size = static_cast<int>(cluster.size());
            fc.centroid_x = sum_x / cluster.size();
            fc.centroid_y = sum_y / cluster.size();
            fc.score = 0.0f;
            clusters_.push_back(fc);
        }
    }
}

bool FrontierExplorer::findBestFrontier(const slam::OccupancyGrid& grid,
                                        const slam::GridPose& current_pose,
                                        FrontierCluster& best_frontier) {
    std::vector<std::pair<int, int>> frontier_cells;

    // Scan occupancy grid for frontier boundary cells
    int w = grid.getWidth();
    int h = grid.getHeight();
    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            if (isFrontierCell(grid, x, y)) {
                frontier_cells.push_back({x, y});
            }
        }
    }

    if (frontier_cells.empty()) {
        return false;
    }

    clusterFrontiers(grid, frontier_cells);

    if (clusters_.empty()) {
        return false;
    }

    // Evaluate cost-utility scoring for each candidate
    // Utility = w_size * size - w_dist * distance - w_turn * heading_deviation
    float highest_score = -1e9f;
    int best_index = -1;

    for (size_t i = 0; i < clusters_.size(); ++i) {
        auto& fc = clusters_[i];

        float dx = fc.centroid_x - current_pose.x;
        float dy = fc.centroid_y - current_pose.y;
        float dist = std::hypot(dx, dy);

        // Don't select frontiers right on top of the drone
        if (dist < 0.40f) continue;

        float target_heading = std::atan2(dy, dx);
        float heading_diff = std::abs(target_heading - current_pose.yaw_rad);
        while (heading_diff > M_PI) heading_diff = 2.0f * M_PI - heading_diff;

        // Weight parameters
        constexpr float W_SIZE = 1.0f;
        constexpr float W_DIST = 1.5f;
        constexpr float W_TURN = 0.8f;

        fc.score = (W_SIZE * fc.size) - (W_DIST * dist) - (W_TURN * heading_diff);

        if (fc.score > highest_score) {
            highest_score = fc.score;
            best_index = static_cast<int>(i);
        }
    }

    if (best_index >= 0) {
        best_frontier = clusters_[best_index];
        return true;
    }

    return false;
}

} // namespace navigation
} // namespace nidar
