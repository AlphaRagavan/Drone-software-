#pragma once

#include "navigation/path_planner.hpp"
#include "slam/occupancy_grid.hpp"
#include "lidar/lidar_types.hpp"
#include <vector>

namespace nidar {
namespace navigation {

struct TwistCommand {
    float vx{0.0f};      // Forward velocity (m/s) in body frame
    float vy{0.0f};      // Lateral velocity (m/s) in body frame (positive left)
    float yaw_rate{0.0f};// Angular velocity (rad/s) (positive CCW)
};

class KinematicsController {
public:
    KinematicsController(float max_vx = 0.5f, float max_yaw_rate = 0.8f);

    // Calculates velocity commands to follow waypoints while centering in 1m corridors
    TwistCommand computeCommand(const slam::GridPose& current_pose,
                                const std::vector<Waypoint>& path,
                                const lidar::LaserScan& scan,
                                size_t& current_waypoint_idx);

    bool isGoalReached(const slam::GridPose& current_pose,
                       const Waypoint& goal,
                       float tolerance_m = 0.35f) const;

private:
    void calculateWallClearances(const lidar::LaserScan& scan,
                                 float& out_front_m,
                                 float& out_left_m,
                                 float& out_right_m) const;

    float max_vx_;
    float max_yaw_rate_;
    float lookahead_dist_{0.60f};
};

} // namespace navigation
} // namespace nidar
