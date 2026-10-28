#pragma once

#include "slam/occupancy_grid.hpp"
#include "lidar/lidar_types.hpp"
#include <string>
#include <vector>
#include <memory>

#ifdef HAVE_OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#endif

namespace nidar {
namespace vision {

struct BoundingBox {
    float x;
    float y;
    float w;
    float h;
};

struct DetectionResult {
    BoundingBox bbox;
    float confidence{0.0f};
    int class_id{0};
};

struct TaggedSurvivor {
    std::string survivor_id;
    uint64_t timestamp_ms;
    float confidence;
    BoundingBox bbox;
    slam::GridPose drone_pose;
    float world_x;
    float world_y;
    int room_col;
    int room_row;
    std::string room_id;
    std::string json_payload;
};

class SurvivorDetector {
public:
    SurvivorDetector(const std::string& onnx_model_path = "models/yolov8n.onnx",
                     float conf_threshold = 0.60f,
                     float nms_threshold = 0.45f);

    bool initialize();

    // Process frame and perform sensor fusion with SLAM pose and LIDAR ranges
    bool processFrame(const cv::Mat& frame,
                      const slam::GridPose& current_pose,
                      const lidar::LaserScan& scan,
                      std::vector<TaggedSurvivor>& out_new_survivors);

    const std::vector<TaggedSurvivor>& getAllTaggedSurvivors() const { return confirmed_survivors_; }

private:
    std::vector<DetectionResult> runInference(const cv::Mat& frame);
    float estimateDistanceToTarget(const DetectionResult& det,
                                   const lidar::LaserScan& scan,
                                   float azimuth_rad);
    bool isDuplicateSurvivor(float world_x, float world_y, float min_dist_m = 1.20f) const;
    std::string formatSurvivorJson(const TaggedSurvivor& s) const;

    std::string model_path_;
    float conf_threshold_;
    float nms_threshold_;

    bool initialized_{false};
    uint32_t survivor_count_{0};

#ifdef HAVE_OPENCV
    cv::dnn::Net net_;
#endif

    std::vector<TaggedSurvivor> confirmed_survivors_;
};

} // namespace vision
} // namespace nidar
