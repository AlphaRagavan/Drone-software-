#pragma once

#include "slam/occupancy_grid.hpp"
#include <vector>
#include <cmath>

namespace nidar {
namespace navigation {

struct FrontierCluster {
    int id{0};
    int size{0};
    float centroid_x{0.0f}; // meters
    float centroid_y{0.0f}; // meters
    float score{0.0f};
};

class FrontierExplorer {
public:
    FrontierExplorer(float min_cluster_size_m = 0.25f);

    // Detects all valid frontier clusters and returns the best candidate
    bool findBestFrontier(const slam::OccupancyGrid& grid,
                          const slam::GridPose& current_pose,
                          FrontierCluster& best_frontier);

    // Get all detected clusters for telemetry visualization
    const std::vector<FrontierCluster>& getLastClusters() const { return clusters_; }

private:
    bool isFrontierCell(const slam::OccupancyGrid& grid, int gx, int gy) const;
    void clusterFrontiers(const slam::OccupancyGrid& grid,
                          const std::vector<std::pair<int, int>>& frontier_cells);

    float min_cluster_size_m_;
    std::vector<FrontierCluster> clusters_;
};

} // namespace navigation
} // namespace nidar
