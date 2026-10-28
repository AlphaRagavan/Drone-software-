#include "navigation/path_planner.hpp"
#include <queue>
#include <cmath>
#include <algorithm>

namespace nidar {
namespace navigation {

struct Node {
    int gx, gy;
    float g_cost;
    float f_cost;

    bool operator>(const Node& other) const {
        return f_cost > other.f_cost;
    }
};

bool PathPlanner::planPath(const slam::OccupancyGrid& grid,
                           float start_x, float start_y,
                           float goal_x, float goal_y,
                           std::vector<Waypoint>& out_path) {
    out_path.clear();

    int start_gx, start_gy, goal_gx, goal_gy;
    if (!grid.worldToGrid(start_x, start_y, start_gx, start_gy) ||
        !grid.worldToGrid(goal_x, goal_y, goal_gx, goal_gy)) {
        return false;
    }

    int w = grid.getWidth();
    int h = grid.getHeight();

    // Generate inflated costmap to ensure 0.25m clearance in 1m corridors
    std::vector<uint8_t> costmap = grid.generateInflatedCostmap(0.25f);

    // If goal is inside obstacle, find closest free neighbor
    if (costmap[goal_gy * w + goal_gx] == slam::CELL_OCCUPIED) {
        bool found = false;
        for (int r = 1; r <= 5 && !found; ++r) {
            for (int dy = -r; dy <= r && !found; ++dy) {
                for (int dx = -r; dx <= r && !found; ++dx) {
                    int nx = goal_gx + dx;
                    int ny = goal_gy + dy;
                    if (grid.isInside(nx, ny) && costmap[ny * w + nx] != slam::CELL_OCCUPIED) {
                        goal_gx = nx;
                        goal_gy = ny;
                        found = true;
                    }
                }
            }
        }
        if (!found) return false;
    }

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
    std::vector<float> g_scores(static_cast<size_t>(w) * h, 1e9f);
    std::vector<int> parent(static_cast<size_t>(w) * h, -1);
    std::vector<bool> closed(static_cast<size_t>(w) * h, false);

    size_t start_idx = start_gy * w + start_gx;
    g_scores[start_idx] = 0.0f;
    float h_start = std::hypot(goal_gx - start_gx, goal_gy - start_gy);
    open_set.push({start_gx, start_gy, 0.0f, h_start});

    bool path_found = false;

    // 8-connected neighbor offsets
    const int dxs[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int dys[8] = {0, 0, 1, -1, 1, 1, -1, -1};
    const float move_costs[8] = {1.0f, 1.0f, 1.0f, 1.0f, 1.4142f, 1.4142f, 1.4142f, 1.4142f};

    while (!open_set.empty()) {
        Node curr = open_set.top();
        open_set.pop();

        if (curr.gx == goal_gx && curr.gy == goal_gy) {
            path_found = true;
            break;
        }

        size_t curr_idx = curr.gy * w + curr.gx;
        if (closed[curr_idx]) continue;
        closed[curr_idx] = true;

        for (int i = 0; i < 8; ++i) {
            int nx = curr.gx + dxs[i];
            int ny = curr.gy + dys[i];

            if (!grid.isInside(nx, ny)) continue;
            size_t n_idx = ny * w + nx;
            if (closed[n_idx]) continue;

            uint8_t cell_val = costmap[n_idx];
            if (cell_val == slam::CELL_OCCUPIED) continue; // Hard obstacle

            // Cost penalty for proximity to walls
            float cell_penalty = (cell_val == slam::CELL_INFLATED) ? 3.0f : 0.0f;
            float tentative_g = g_scores[curr_idx] + move_costs[i] + cell_penalty;

            if (tentative_g < g_scores[n_idx]) {
                g_scores[n_idx] = tentative_g;
                parent[n_idx] = static_cast<int>(curr_idx);
                float h_cost = std::hypot(goal_gx - nx, goal_gy - ny);
                open_set.push({nx, ny, tentative_g, tentative_g + h_cost});
            }
        }
    }

    if (!path_found) {
        return false;
    }

    // Reconstruct raw path from parent pointers
    std::vector<std::pair<int, int>> raw_path;
    int curr_idx = goal_gy * w + goal_gx;
    while (curr_idx != -1) {
        int gy = curr_idx / w;
        int gx = curr_idx % w;
        raw_path.push_back({gx, gy});
        if (gx == start_gx && gy == start_gy) break;
        curr_idx = parent[curr_idx];
    }
    std::reverse(raw_path.begin(), raw_path.end());

    // Prune path using line-of-sight shortcutting
    prunePath(grid, costmap, raw_path, out_path);
    return !out_path.empty();
}

bool PathPlanner::lineOfSight(const slam::OccupancyGrid& grid,
                              const std::vector<uint8_t>& costmap,
                              int x0, int y0, int x1, int y1) const {
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    int x = x0;
    int y = y0;
    int w = grid.getWidth();

    while (x != x1 || y != y1) {
        if (!grid.isInside(x, y) || costmap[y * w + x] == slam::CELL_OCCUPIED) {
            return false;
        }
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
    return true;
}

void PathPlanner::prunePath(const slam::OccupancyGrid& grid,
                            const std::vector<uint8_t>& costmap,
                            const std::vector<std::pair<int, int>>& raw_grid_path,
                            std::vector<Waypoint>& out_path) const {
    if (raw_grid_path.empty()) return;

    size_t current = 0;
    float wx, wy;
    grid.gridToWorld(raw_grid_path[0].first, raw_grid_path[0].second, wx, wy);
    out_path.push_back({wx, wy});

    while (current < raw_grid_path.size() - 1) {
        size_t next = raw_grid_path.size() - 1;
        while (next > current + 1) {
            if (lineOfSight(grid, costmap,
                            raw_grid_path[current].first, raw_grid_path[current].second,
                            raw_grid_path[next].first, raw_grid_path[next].second)) {
                break;
            }
            next--;
        }

        grid.gridToWorld(raw_grid_path[next].first, raw_grid_path[next].second, wx, wy);
        out_path.push_back({wx, wy});
        current = next;
    }
}

} // namespace navigation
} // namespace nidar
