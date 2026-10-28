#pragma once

#include "slam/occupancy_grid.hpp"
#include <vector>

namespace nidar {
namespace navigation {

struct Waypoint {
    float x;
    float y;
};

class PathPlanner {
public:
    PathPlanner() = default;

    // Plans an A* path from start to goal avoiding inflated walls
    bool planPath(const slam::OccupancyGrid& grid,
                  float start_x, float start_y,
                  float goal_x, float goal_y,
                  std::vector<Waypoint>& out_path);

private:
    bool lineOfSight(const slam::OccupancyGrid& grid,
                     const std::vector<uint8_t>& costmap,
                     int x0, int y0, int x1, int y1) const;

    void prunePath(const slam::OccupancyGrid& grid,
                   const std::vector<uint8_t>& costmap,
                   const std::vector<std::pair<int, int>>& raw_grid_path,
                   std::vector<Waypoint>& out_path) const;
};

} // namespace navigation
} // namespace nidar
