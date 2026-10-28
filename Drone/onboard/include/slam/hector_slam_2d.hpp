#pragma once

#include "slam/occupancy_grid.hpp"
#include "lidar/lidar_types.hpp"
#include <mutex>
#include <memory>

namespace nidar {
namespace slam {

class HectorSlam2D {
public:
    HectorSlam2D(std::shared_ptr<OccupancyGrid> grid);
    ~HectorSlam2D() = default;

    // Process a new 360-degree laser scan from RPLIDAR
    GridPose update(const lidar::LaserScan& scan);

    GridPose getCurrentPose() const;
    void setCurrentPose(const GridPose& pose);

    std::shared_ptr<OccupancyGrid> getGrid() const { return grid_; }

private:
    // Gauss-Newton Scan-to-Map Matching optimization
    bool matchScan(const lidar::LaserScan& scan, GridPose& inout_pose);

    // Update map with raycasting using newly aligned pose
    void updateMap(const lidar::LaserScan& scan, const GridPose& pose);

    std::shared_ptr<OccupancyGrid> grid_;
    mutable std::mutex pose_mutex_;
    GridPose current_pose_;
    bool first_scan_{true};
};

} // namespace slam
} // namespace nidar
