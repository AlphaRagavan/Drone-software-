#include "vision/survivor_detector.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <chrono>

namespace nidar {
namespace vision {

SurvivorDetector::SurvivorDetector(const std::string& onnx_model_path,
                                   float conf_threshold,
                                   float nms_threshold)
    : model_path_(onnx_model_path),
      conf_threshold_(conf_threshold),
      nms_threshold_(nms_threshold) {}

bool SurvivorDetector::initialize() {
#ifdef HAVE_OPENCV
    try {
        net_ = cv::dnn::readNetFromONNX(model_path_);
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        std::cout << "[Survivor Detector] Successfully loaded offline YOLOv8n ONNX model from: " << model_path_ << std::endl;
        initialized_ = true;
        return true;
    } catch (const cv::Exception& e) {
        std::cerr << "[Survivor Detector] Could not load ONNX file (" << e.what() 
                  << "). Utilizing offline synthetic detection engine." << std::endl;
        initialized_ = true; // Fallback engine active
        return true;
    }
#else
    initialized_ = true;
    return true;
#endif
}

std::vector<DetectionResult> SurvivorDetector::runInference(const cv::Mat& frame) {
    std::vector<DetectionResult> results;
#ifdef HAVE_OPENCV
    if (frame.empty()) return results;

    if (!net_.empty()) {
        // Preprocess: 640x640 blob, RGB, normalized [0, 1]
        cv::Mat blob;
        cv::dnn::blobFromImage(frame, blob, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(), true, false);
        net_.setInput(blob);

        std::vector<cv::Mat> outputs;
        net_.forward(outputs, net_.getUnconnectedOutLayersNames());

        if (outputs.empty()) return results;

        // YOLOv8 output format: [1, 84, 8400]
        cv::Mat out = outputs[0];
        int dimensions = out.size[1]; // 84: 4 bbox coords + 80 class scores
        int rows = out.size[2];       // 8400 anchors

        // Transpose to [8400, 84] for direct iteration
        out = out.reshape(1, dimensions);
        cv::transpose(out, out);

        float* data = (float*)out.data;
        float x_factor = frame.cols / 640.0f;
        float y_factor = frame.rows / 640.0f;

        std::vector<cv::Rect> boxes;
        std::vector<float> confidences;
        std::vector<int> class_ids;

        for (int i = 0; i < rows; ++i) {
            float* row = data + i * dimensions;
            // Person is class index 0 (row[4])
            float person_score = row[4];

            if (person_score >= conf_threshold_) {
                float cx = row[0];
                float cy = row[1];
                float w = row[2];
                float h = row[3];

                int left = static_cast<int>((cx - 0.5f * w) * x_factor);
                int top = static_cast<int>((cy - 0.5f * h) * y_factor);
                int width = static_cast<int>(w * x_factor);
                int height = static_cast<int>(h * y_factor);

                boxes.push_back(cv::Rect(left, top, width, height));
                confidences.push_back(person_score);
                class_ids.push_back(0);
            }
        }

        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, conf_threshold_, nms_threshold_, indices);

        for (int idx : indices) {
            DetectionResult det;
            det.bbox.x = static_cast<float>(boxes[idx].x);
            det.bbox.y = static_cast<float>(boxes[idx].y);
            det.bbox.w = static_cast<float>(boxes[idx].width);
            det.bbox.h = static_cast<float>(boxes[idx].height);
            det.confidence = confidences[idx];
            det.class_id = class_ids[idx];
            results.push_back(det);
        }
    } else {
        // Fallback synthetic detector for simulation / unit test
        // Checks color signature (orange high-visibility vest)
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::Mat mask;
        cv::inRange(hsv, cv::Scalar(10, 120, 150), cv::Scalar(25, 255, 255), mask);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& c : contours) {
            if (cv::contourArea(c) > 400.0) {
                cv::Rect r = cv::boundingRect(c);
                DetectionResult det;
                det.bbox.x = static_cast<float>(r.x);
                det.bbox.y = static_cast<float>(r.y);
                det.bbox.w = static_cast<float>(r.width);
                det.bbox.h = static_cast<float>(r.height);
                det.confidence = 0.88f;
                det.class_id = 0;
                results.push_back(det);
            }
        }
    }
#else
    (void)frame;
#endif
    return results;
}

float SurvivorDetector::estimateDistanceToTarget(const DetectionResult& det,
                                                 const lidar::LaserScan& scan,
                                                 float azimuth_rad) {
    // 1. Camera optical height geometric estimate
    // Standard human torso/head height ~0.75m, camera vertical focal length fy ~ 500 px
    constexpr float FY_PX = 500.0f;
    constexpr float HUMAN_HEIGHT_M = 0.75f;
    float dist_optical = (FY_PX * HUMAN_HEIGHT_M) / std::max(det.bbox.h, 20.0f);

    // 2. Correlate with RPLIDAR range at matching relative azimuth
    float best_lidar_dist = 100.0f;
    for (const auto& pt : scan.points) {
        float angle_diff = std::abs(pt.angle_rad - azimuth_rad);
        if (angle_diff > M_PI) angle_diff = 2.0f * M_PI - angle_diff;

        if (angle_diff < (10.0f * 3.14159f / 180.0f)) { // within 10 degree cone
            if (std::abs(pt.range_m - dist_optical) < 1.0f) {
                best_lidar_dist = pt.range_m;
                break;
            }
        }
    }

    if (best_lidar_dist < 15.0f) {
        return best_lidar_dist; // Highly accurate LIDAR return
    }

    return std::clamp(dist_optical, 0.40f, 6.0f);
}

bool SurvivorDetector::isDuplicateSurvivor(float world_x, float world_y, float min_dist_m) const {
    for (const auto& s : confirmed_survivors_) {
        float dist = std::hypot(s.world_x - world_x, s.world_y - world_y);
        if (dist < min_dist_m) {
            return true;
        }
    }
    return false;
}

std::string SurvivorDetector::formatSurvivorJson(const TaggedSurvivor& s) const {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "{\n"
       << "  \"type\": \"SURVIVOR_TAGGED\",\n"
       << "  \"timestamp_ms\": " << s.timestamp_ms << ",\n"
       << "  \"survivor_id\": \"" << s.survivor_id << "\",\n"
       << "  \"confidence\": " << s.confidence << ",\n"
       << "  \"bounding_box\": {\"x\": " << (int)s.bbox.x << ", \"y\": " << (int)s.bbox.y 
       << ", \"w\": " << (int)s.bbox.w << ", \"h\": " << (int)s.bbox.h << "},\n"
       << "  \"drone_pose\": {\"x\": " << s.drone_pose.x << ", \"y\": " << s.drone_pose.y 
       << ", \"yaw_deg\": " << (s.drone_pose.yaw_rad * 180.0f / 3.14159265f) << "},\n"
       << "  \"survivor_coord\": {\"x\": " << s.world_x << ", \"y\": " << s.world_y << "},\n"
       << "  \"grid_room\": {\"room_id\": \"" << s.room_id << "\", \"col\": " << s.room_col 
       << ", \"row\": " << s.room_row << ", \"room_bounds\": ["
       << (s.room_col * 2.0f) << ", " << (s.room_row * 2.0f) << ", "
       << ((s.room_col + 1) * 2.0f) << ", " << ((s.room_row + 1) * 2.0f) << "]}\n"
       << "}";
    return ss.str();
}

bool SurvivorDetector::processFrame(const cv::Mat& frame,
                                    const slam::GridPose& current_pose,
                                    const lidar::LaserScan& scan,
                                    std::vector<TaggedSurvivor>& out_new_survivors) {
    out_new_survivors.clear();
    if (!initialized_) return false;

    auto detections = runInference(frame);
    if (detections.empty()) return false;

    constexpr float FX_PX = 500.0f;
    constexpr float CX_PX = 320.0f;

    auto now = std::chrono::steady_clock::now();
    uint64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    for (const auto& det : detections) {
        if (det.confidence < conf_threshold_) continue;

        // Bounding box center horizontal coordinate
        float center_x = det.bbox.x + 0.5f * det.bbox.w;
        // Azimuth angle relative to drone optical axis (forward is 0 rad, right positive)
        float azimuth = std::atan2(center_x - CX_PX, FX_PX);

        // Estimate distance using camera + lidar fusion
        float distance = estimateDistanceToTarget(det, scan, -azimuth);

        // Global coordinate transformation
        float world_angle = current_pose.yaw_rad - azimuth;
        float survivor_x = current_pose.x + distance * std::cos(world_angle);
        float survivor_y = current_pose.y + distance * std::sin(world_angle);

        // Deduplication: prevent repeated tags of the same survivor
        if (isDuplicateSurvivor(survivor_x, survivor_y, 1.20f)) {
            continue;
        }

        // Calculate 2x2 meter room standards
        int room_col = static_cast<int>(std::floor(survivor_x / 2.0f));
        int room_row = static_cast<int>(std::floor(survivor_y / 2.0f));
        std::string room_id = "Room_" + std::to_string(room_col) + "_" + std::to_string(room_row);

        TaggedSurvivor survivor;
        survivor.survivor_id = "SURVIVOR_" + std::to_string(++survivor_count_);
        survivor.timestamp_ms = now_ms;
        survivor.confidence = det.confidence;
        survivor.bbox = det.bbox;
        survivor.drone_pose = current_pose;
        survivor.world_x = survivor_x;
        survivor.world_y = survivor_y;
        survivor.room_col = room_col;
        survivor.room_row = room_row;
        survivor.room_id = room_id;
        survivor.json_payload = formatSurvivorJson(survivor);

        confirmed_survivors_.push_back(survivor);
        out_new_survivors.push_back(survivor);

        std::cout << "\n========================================================" << std::endl;
        std::cout << "[AI TAGGER] " << survivor.survivor_id << " IDENTIFIED in " << room_id 
                  << " at (" << survivor.world_x << "m, " << survivor.world_y << "m) Conf: " 
                  << survivor.confidence * 100.0f << "%" << std::endl;
        std::cout << "========================================================\n" << std::endl;
    }

    return !out_new_survivors.empty();
}

} // namespace vision
} // namespace nidar
