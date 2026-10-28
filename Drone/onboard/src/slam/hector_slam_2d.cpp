#include "slam/hector_slam_2d.hpp"
#include <cmath>
#include <iostream>
#include <array>

namespace nidar {
namespace slam {

HectorSlam2D::HectorSlam2D(std::shared_ptr<OccupancyGrid> grid)
    : grid_(grid) {
    // Start pose near entrance of 15m x 15m maze (e.g. x = 1.0m, y = 1.0m, facing forward along corridor)
    current_pose_.x = 1.0f;
    current_pose_.y = 1.0f;
    current_pose_.yaw_rad = 0.0f;
}

GridPose HectorSlam2D::getCurrentPose() const {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return current_pose_;
}

void HectorSlam2D::setCurrentPose(const GridPose& pose) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_pose_ = pose;
}

GridPose HectorSlam2D::update(const lidar::LaserScan& scan) {
    if (scan.points.empty()) {
        return getCurrentPose();
    }

    GridPose estimated_pose = getCurrentPose();

    if (first_scan_) {
        // Seed initial map with first scan
        updateMap(scan, estimated_pose);
        first_scan_ = false;
        return estimated_pose;
    }

    // Perform Gauss-Newton scan-to-map alignment
    if (matchScan(scan, estimated_pose)) {
        setCurrentPose(estimated_pose);
        // Only update map if alignment converged smoothly
        updateMap(scan, estimated_pose);
    }

    return estimated_pose;
}

bool HectorSlam2D::matchScan(const lidar::LaserScan& scan, GridPose& inout_pose) {
    constexpr int MAX_ITERATIONS = 5;
    constexpr float CONVERGENCE_EPSILON = 1e-4f;
    constexpr float DAMPING_LAMBDA = 1e-3f;

    float px = inout_pose.x;
    float py = inout_pose.y;
    float ptheta = inout_pose.yaw_rad;

    for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
        float cos_th = std::cos(ptheta);
        float sin_th = std::sin(ptheta);

        // 3x3 Hessian matrix H and 3x1 residual gradient b
        std::array<float, 9> H = {0};
        std::array<float, 3> b = {0};

        int valid_points = 0;

        for (const auto& pt : scan.points) {
            // Transform lidar point from body frame to current world frame candidate
            float wx = cos_th * pt.x - sin_th * pt.y + px;
            float wy = sin_th * pt.x + cos_th * pt.y + py;

            // Sample map probability and spatial gradients
            float prob = grid_->getProbability(wx, wy);
            auto [grad_x, grad_y] = grid_->getGradient(wx, wy);

            // Residual: distance to target occupancy (1.0 = solid wall)
            float residual = 1.0f - prob;

            // Jacobian terms: d(w)/d(p)
            // d(wx)/d(px) = 1, d(wx)/d(py) = 0, d(wx)/d(theta) = -sin(th)*pt.x - cos(th)*pt.y
            // d(wy)/d(px) = 0, d(wy)/d(py) = 1, d(wy)/d(theta) =  cos(th)*pt.x - sin(th)*pt.y
            float d_theta_x = -sin_th * pt.x - cos_th * pt.y;
            float d_theta_y =  cos_th * pt.x - sin_th * pt.y;

            float j0 = grad_x;
            float j1 = grad_y;
            float j2 = grad_x * d_theta_x + grad_y * d_theta_y;

            // Accumulate H += J^T * J
            H[0] += j0 * j0; H[1] += j0 * j1; H[2] += j0 * j2;
            H[3] += j1 * j0; H[4] += j1 * j1; H[5] += j1 * j2;
            H[6] += j2 * j0; H[7] += j2 * j1; H[8] += j2 * j2;

            // Accumulate b += J^T * residual
            b[0] += j0 * residual;
            b[1] += j1 * residual;
            b[2] += j2 * residual;

            valid_points++;
        }

        if (valid_points < 30) {
            return false;
        }

        // Add Levenberg-Marquardt diagonal damping
        H[0] += DAMPING_LAMBDA;
        H[4] += DAMPING_LAMBDA;
        H[8] += DAMPING_LAMBDA;

        // Invert 3x3 symmetric matrix H using Cramer's rule
        float det = H[0] * (H[4] * H[8] - H[5] * H[7]) -
                    H[1] * (H[3] * H[8] - H[5] * H[6]) +
                    H[2] * (H[3] * H[7] - H[4] * H[6]);

        if (std::abs(det) < 1e-7f) {
            return false;
        }

        float inv_det = 1.0f / det;
        std::array<float, 9> invH;
        invH[0] = (H[4] * H[8] - H[5] * H[7]) * inv_det;
        invH[1] = (H[2] * H[7] - H[1] * H[8]) * inv_det;
        invH[2] = (H[1] * H[5] - H[2] * H[4]) * inv_det;

        invH[3] = (H[5] * H[6] - H[3] * H[8]) * inv_det;
        invH[4] = (H[0] * H[8] - H[2] * H[6]) * inv_det;
        invH[5] = (H[2] * H[3] - H[0] * H[5]) * inv_det;

        invH[6] = (H[3] * H[7] - H[4] * H[6]) * inv_det;
        invH[7] = (H[1] * H[6] - H[0] * H[7]) * inv_det;
        invH[8] = (H[0] * H[4] - H[1] * H[3]) * inv_det;

        // Delta step: delta_p = inv(H) * b
        float delta_x = invH[0] * b[0] + invH[1] * b[1] + invH[2] * b[2];
        float delta_y = invH[3] * b[0] + invH[4] * b[1] + invH[5] * b[2];
        float delta_th = invH[6] * b[0] + invH[7] * b[1] + invH[8] * b[2];

        // Apply state update
        px += delta_x;
        py += delta_y;
        ptheta += delta_th;

        // Wrap angle
        while (ptheta > M_PI)  ptheta -= 2.0f * M_PI;
        while (ptheta < -M_PI) ptheta += 2.0f * M_PI;

        if (delta_x * delta_x + delta_y * delta_y + delta_th * delta_th < CONVERGENCE_EPSILON) {
            break; // Converged
        }
    }

    inout_pose.x = px;
    inout_pose.y = py;
    inout_pose.yaw_rad = ptheta;
    return true;
}

void HectorSlam2D::updateMap(const lidar::LaserScan& scan, const GridPose& pose) {
    float cos_th = std::cos(pose.yaw_rad);
    float sin_th = std::sin(pose.yaw_rad);

    for (const auto& pt : scan.points) {
        float hit_x = cos_th * pt.x - sin_th * pt.y + pose.x;
        float hit_y = sin_th * pt.x + cos_th * pt.y + pose.y;

        grid_->updateRay(pose.x, pose.y, hit_x, hit_y);
    }
}

} // namespace slam
} // namespace nidar
