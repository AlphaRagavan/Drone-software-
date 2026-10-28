#include "navigation/kinematics_controller.hpp"
#include <cmath>
#include <algorithm>

namespace nidar {
namespace navigation {

KinematicsController::KinematicsController(float max_vx, float max_yaw_rate)
    : max_vx_(max_vx), max_yaw_rate_(max_yaw_rate) {}

void KinematicsController::calculateWallClearances(const lidar::LaserScan& scan,
                                                   float& out_front_m,
                                                   float& out_left_m,
                                                   float& out_right_m) const {
    out_front_m = 10.0f;
    out_left_m  = 10.0f;
    out_right_m = 10.0f;

    for (const auto& pt : scan.points) {
        float deg = pt.angle_rad * 180.0f / 3.14159265f;

        // Front sector: [-30 deg, +30 deg]
        if (std::abs(deg) <= 30.0f) {
            if (pt.range_m < out_front_m) out_front_m = pt.range_m;
        }
        // Left sector: [+60 deg, +120 deg]
        else if (deg >= 60.0f && deg <= 120.0f) {
            if (pt.range_m < out_left_m) out_left_m = pt.range_m;
        }
        // Right sector: [-120 deg, -60 deg]
        else if (deg >= -120.0f && deg <= -60.0f) {
            if (pt.range_m < out_right_m) out_right_m = pt.range_m;
        }
    }
}

TwistCommand KinematicsController::computeCommand(const slam::GridPose& current_pose,
                                                  const std::vector<Waypoint>& path,
                                                  const lidar::LaserScan& scan,
                                                  size_t& current_waypoint_idx) {
    TwistCommand cmd;
    if (path.empty() || current_waypoint_idx >= path.size()) {
        return cmd; // Halt
    }

    // Advance waypoint index if already close to current target
    while (current_waypoint_idx < path.size() - 1) {
        float dx = path[current_waypoint_idx].x - current_pose.x;
        float dy = path[current_waypoint_idx].y - current_pose.y;
        if (std::hypot(dx, dy) < lookahead_dist_) {
            current_waypoint_idx++;
        } else {
            break;
        }
    }

    const Waypoint& target = path[current_waypoint_idx];
    float dx = target.x - current_pose.x;
    float dy = target.y - current_pose.y;
    float dist_to_target = std::hypot(dx, dy);

    // Target heading in world frame
    float target_heading = std::atan2(dy, dx);
    float heading_error = target_heading - current_pose.yaw_rad;

    // Normalize error to [-pi, pi]
    while (heading_error > M_PI)  heading_error -= 2.0f * M_PI;
    while (heading_error < -M_PI) heading_error += 2.0f * M_PI;

    // Scan clearances for corridor centering and obstacle avoidance
    float front_clearance, left_clearance, right_clearance;
    calculateWallClearances(scan, front_clearance, left_clearance, right_clearance);

    // 1. Angular Velocity Control (Proportional Heading Tracking)
    constexpr float K_YAW = 1.2f;
    cmd.yaw_rate = std::clamp(K_YAW * heading_error, -max_yaw_rate_, max_yaw_rate_);

    // 2. Forward Velocity Control
    // If heading error is large, slow down forward motion to turn in place
    float heading_scale = std::max(0.0f, std::cos(heading_error));
    cmd.vx = max_vx_ * heading_scale;

    // Forward Obstacle Deceleration in Corridors
    // Critical safety threshold: 0.25m, deceleration zone: 0.45m
    if (front_clearance < 0.45f) {
        float obstacle_scale = std::clamp((front_clearance - 0.25f) / (0.45f - 0.25f), 0.0f, 1.0f);
        cmd.vx *= obstacle_scale;
        if (front_clearance < 0.25f) {
            cmd.vx = 0.0f; // Halt forward drive
        }
    }

    // 3. Corridor Wall Centering Control (Lateral Drift Compensation)
    // In a 1-meter corridor, both walls should ideally be ~0.50m from drone center
    if (left_clearance < 0.85f && right_clearance < 0.85f) {
        float wall_diff = left_clearance - right_clearance; // positive if closer to right wall
        constexpr float K_CENTER = 0.35f;
        cmd.vy = std::clamp(K_CENTER * wall_diff, -0.20f, 0.20f);
    } else {
        cmd.vy = 0.0f;
    }

    return cmd;
}

bool KinematicsController::isGoalReached(const slam::GridPose& current_pose,
                                          const Waypoint& goal,
                                          float tolerance_m) const {
    float dx = goal.x - current_pose.x;
    float dy = goal.y - current_pose.y;
    return (std::hypot(dx, dy) <= tolerance_m);
}

} // namespace navigation
} // namespace nidar
