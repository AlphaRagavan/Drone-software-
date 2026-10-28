#include "autonomy/mission_state_machine.hpp"
#include <iostream>
#include <chrono>

namespace nidar {
namespace autonomy {

MissionStateMachine::MissionStateMachine() = default;
MissionStateMachine::~MissionStateMachine() {
    if (fc_) fc_->emergencyKillSwitch();
}

std::string MissionStateMachine::getStateName() const {
    switch (current_state_) {
        case STATE_BOOT_AND_INIT:      return "BOOT_AND_INIT";
        case STATE_TAKEOFF_AND_ENTER:  return "TAKEOFF_AND_ENTER";
        case STATE_EXPLORE_MAZE:       return "EXPLORE_MAZE";
        case STATE_TAG_SURVIVOR:       return "TAG_SURVIVOR";
        case STATE_FIND_EXIT_AND_LAND: return "FIND_EXIT_AND_LAND";
        case STATE_EMERGENCY_ABORT:    return "EMERGENCY_ABORT";
        case STATE_MISSION_COMPLETED:  return "MISSION_COMPLETED";
        default:                       return "UNKNOWN";
    }
}

bool MissionStateMachine::initialize(const std::string& lidar_port,
                                     const std::string& model_path,
                                     const std::string& fc_port) {
    std::cout << "==========================================================" << std::endl;
    std::cout << " INITIALIZING NIDAR AUTONOMOUS SEARCH & RESCUE SYSTEM" << std::endl;
    std::cout << " Mode: 100% Offline, Zero-Connectivity AirMouse Challenge" << std::endl;
    std::cout << "==========================================================" << std::endl;

    // 1. Occupancy Grid: 360x360 cells @ 0.05m = 18m x 18m (covers 15x15m maze with buffer)
    grid_ = std::make_shared<slam::OccupancyGrid>(360, 360, 0.05f, 0.0f, 0.0f);
    slam_ = std::make_unique<slam::HectorSlam2D>(grid_);

    // 2. RPLIDAR Driver
    lidar_ = std::make_unique<lidar::NidarRplidar>(lidar_port, 115200);
    if (!lidar_->initialize()) {
        std::cerr << "[INIT] Warning: RPLIDAR hardware offline. Operating in simulation mode." << std::endl;
    }
    lidar_->startScanning();

    // 3. Navigation Modules
    explorer_ = std::make_unique<navigation::FrontierExplorer>(0.25f);
    planner_ = std::make_unique<navigation::PathPlanner>();
    controller_ = std::make_unique<navigation::KinematicsController>(0.45f, 0.80f);

    // 4. Vision & Camera
    camera_ = std::make_unique<vision::CameraStream>(0, 640, 480, 30);
    camera_->start();
    vision_ = std::make_unique<vision::SurvivorDetector>(model_path, 0.60f, 0.45f);
    vision_->initialize();

    // 5. Flight Controller HAL
    fc_ = std::make_unique<hal::FlightControllerInterface>(fc_port, 921600);
    fc_->initialize();

    // 6. Networking & Telemetry Sockets
    mdns_ = std::make_unique<net::MdnsBroadcaster>("nidar-drone", "_nidar-drone._tcp", 9001);
    mdns_->start("192.168.4.1");

    tcp_server_ = std::make_unique<net::TcpTelemetryServer>(9001);
    tcp_server_->start();

    udp_abort_ = std::make_unique<net::UdpAbortListener>(9999);
    udp_abort_->start([this](const std::string& reason) {
        this->triggerEmergencyAbort(reason);
    });

    video_streamer_ = std::make_unique<net::RtspStreamer>(8554, 8080);
    video_streamer_->start();

    current_state_ = STATE_BOOT_AND_INIT;
    std::cout << "[INIT] Subsystems initialized. State: " << getStateName() << std::endl;
    return true;
}

void MissionStateMachine::triggerEmergencyAbort(const std::string& reason) {
    current_state_ = STATE_EMERGENCY_ABORT;
    if (fc_) {
        fc_->emergencyKillSwitch();
    }
    if (tcp_server_) {
        tcp_server_->broadcastStateMessage("EMERGENCY_ABORT: " + reason);
    }
    std::cerr << "[AUTONOMY] System transition to EMERGENCY_ABORT. Reason: " << reason << std::endl;
}

void MissionStateMachine::step() {
    iteration_count_++;

    // 1. Process SLAM with latest LIDAR scan
    lidar::LaserScan scan = lidar_->getLatestScan();
    slam::GridPose pose = slam_->update(scan);

    // 2. Enforce 30-Minute Mission Timer
    if (current_state_ != STATE_BOOT_AND_INIT &&
        current_state_ != STATE_EMERGENCY_ABORT &&
        current_state_ != STATE_MISSION_COMPLETED) {
        
        auto now = std::chrono::steady_clock::now();
        uint32_t elapsed_sec = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::seconds>(now - mission_start_time_).count());

        // Hard timeout auto-land at 29.5 minutes (zero disqualification)
        if (elapsed_sec >= HARD_TIMEOUT_LAND_SEC) {
            std::cerr << "[MISSION TIMER] Hard 29.5-min threshold reached! Force landing immediately." << std::endl;
            current_state_ = STATE_FIND_EXIT_AND_LAND;
        }
        // Safe return-to-base threshold at 27 minutes
        else if (elapsed_sec >= RETURN_TO_HOME_TIME_SEC && current_state_ == STATE_EXPLORE_MAZE) {
            std::cout << "[MISSION TIMER] 27-Minute window reached. Initiating return to exit portal." << std::endl;
            current_state_ = STATE_FIND_EXIT_AND_LAND;
            current_path_.clear();
        }
    }

    // 3. Update Video Streamer with latest camera JPEG
    std::vector<uint8_t> jpeg_buf;
    if (camera_->getLatestJpeg(jpeg_buf)) {
        video_streamer_->updateFrame(jpeg_buf);
    }

    // 4. Run State Machine Execution
    switch (current_state_) {
        case STATE_BOOT_AND_INIT: {
            if (iteration_count_ > 10) { // Allow 0.5s for sensor stabilization
                mission_start_time_ = std::chrono::steady_clock::now();
                current_state_ = STATE_TAKEOFF_AND_ENTER;
                std::cout << "[AUTONOMY] State change -> TAKEOFF_AND_ENTER" << std::endl;
            }
            break;
        }

        case STATE_TAKEOFF_AND_ENTER:
            handleTakeoffAndEnter();
            break;

        case STATE_EXPLORE_MAZE:
            handleExploreMaze();
            break;

        case STATE_TAG_SURVIVOR:
            handleTagSurvivor();
            break;

        case STATE_FIND_EXIT_AND_LAND:
            handleFindExitAndLand();
            break;

        case STATE_EMERGENCY_ABORT:
            handleEmergencyAbort();
            break;

        case STATE_MISSION_COMPLETED:
            break;
    }

    // 5. Broadcast real-time telemetry to Windows GCS (10 Hz)
    if (iteration_count_ % 2 == 0) {
        broadcastSystemTelemetry();
    }

    // 6. Broadcast SLAM occupancy grid map to Windows GCS (2 Hz)
    if (iteration_count_ % 10 == 0) {
        net::GridMapHeader map_hdr;
        map_hdr.width_cells = grid_->getWidth();
        map_hdr.height_cells = grid_->getHeight();
        map_hdr.resolution_m = grid_->getResolution();
        map_hdr.origin_x_m = grid_->getOriginX();
        map_hdr.origin_y_m = grid_->getOriginY();

        std::vector<uint8_t> rle_data = grid_->compressRLE();
        map_hdr.compressed_len = static_cast<uint32_t>(rle_data.size());

        tcp_server_->broadcastMap(map_hdr, rle_data);
    }
}

void MissionStateMachine::handleTakeoffAndEnter() {
    // 1. Takeoff to 1.0 meter (ceiling is 8ft ~ 2.43m, 1m gives 1.4m overhead clearance)
    fc_->takeoff(1.0f);

    // 2. Advance 1.0m into the entrance of the 15x15m maze corridor
    slam::GridPose pose = slam_->getCurrentPose();
    navigation::TwistCommand cmd;
    cmd.vx = 0.30f;
    cmd.vy = 0.0f;
    cmd.yaw_rate = 0.0f;
    fc_->sendVelocity(cmd);

    // Once clear of entrance portal, transition to autonomous exploration
    if (pose.x >= entrance_point_.x + 0.8f) {
        current_state_ = STATE_EXPLORE_MAZE;
        current_path_.clear();
        std::cout << "[AUTONOMY] Maze corridor penetrated. Transition to EXPLORE_MAZE." << std::endl;
    }
}

void MissionStateMachine::handleExploreMaze() {
    slam::GridPose pose = slam_->getCurrentPose();
    lidar::LaserScan scan = lidar_->getLatestScan();

    // 1. Continuous AI Survivor Detection
    cv::Mat frame;
    if (camera_->getLatestFrame(frame)) {
        std::vector<vision::TaggedSurvivor> new_survivors;
        if (vision_->processFrame(frame, pose, scan, new_survivors)) {
            // New candidate survivor identified!
            candidate_survivor_ = new_survivors.front();
            survivor_confirmation_counter_ = 0;
            current_state_ = STATE_TAG_SURVIVOR;
            std::cout << "[AUTONOMY] Candidate survivor detected! Pausing exploration to TAG_SURVIVOR." << std::endl;
            return;
        }
    }

    // 2. Frontier Evaluation & Path Planning
    if (current_path_.empty() || current_waypoint_idx_ >= current_path_.size()) {
        navigation::FrontierCluster best_frontier;
        if (explorer_->findBestFrontier(*grid_, pose, best_frontier)) {
            current_frontier_ = best_frontier;
            std::cout << "[FRONTIER] Targeting frontier #" << best_frontier.id 
                      << " at (" << best_frontier.centroid_x << "m, " << best_frontier.centroid_y 
                      << "m), Size: " << best_frontier.size << std::endl;

            if (planner_->planPath(*grid_, pose.x, pose.y,
                                  best_frontier.centroid_x, best_frontier.centroid_y,
                                  current_path_)) {
                current_waypoint_idx_ = 0;
            } else {
                std::cout << "[PLANNER] Path directly blocked. Searching alternative frontier..." << std::endl;
            }
        } else {
            // All frontiers explored! Entire 15x15m arena mapped.
            std::cout << "[AUTONOMY] Full arena explored! Returning to exit." << std::endl;
            current_state_ = STATE_FIND_EXIT_AND_LAND;
            current_path_.clear();
            return;
        }
    }

    // 3. Corridor Kinematics & Velocity Control
    navigation::TwistCommand cmd = controller_->computeCommand(pose, current_path_, scan, current_waypoint_idx_);
    fc_->sendVelocity(cmd);
}

void MissionStateMachine::handleTagSurvivor() {
    // 1. Hold position and hover stably
    navigation::TwistCommand hover_cmd;
    hover_cmd.vx = 0.0f;
    hover_cmd.vy = 0.0f;
    hover_cmd.yaw_rate = 0.0f;
    fc_->sendVelocity(hover_cmd);

    survivor_confirmation_counter_++;

    // Hover for 1.0 second (20 cycles @ 20Hz) to confirm visual tag
    if (survivor_confirmation_counter_ >= 20) {
        // Broadcast survivor JSON payload to Windows GCS
        tcp_server_->broadcastSurvivorTag(candidate_survivor_.json_payload);
        std::cout << "[AUTONOMY] Survivor tag transmitted to GCS: " 
                  << candidate_survivor_.survivor_id << " in " << candidate_survivor_.room_id << std::endl;

        // Resume exploration
        current_state_ = STATE_EXPLORE_MAZE;
        current_path_.clear(); // Re-plan path from current position
    }
}

void MissionStateMachine::handleFindExitAndLand() {
    slam::GridPose pose = slam_->getCurrentPose();
    lidar::LaserScan scan = lidar_->getLatestScan();

    // 1. Plan path back to entrance waypoint (1.0, 1.0)
    if (current_path_.empty()) {
        if (!planner_->planPath(*grid_, pose.x, pose.y,
                                entrance_point_.x, entrance_point_.y,
                                current_path_)) {
            // If path blocked, land directly at safe clearing
            std::cerr << "[AUTONOMY] Path to exit obstructed. Landing in current corridor clearing." << std::endl;
            fc_->land();
            current_state_ = STATE_MISSION_COMPLETED;
            return;
        }
        current_waypoint_idx_ = 0;
        std::cout << "[AUTONOMY] Planned return path to entrance (" << entrance_point_.x << ", " << entrance_point_.y << ")." << std::endl;
    }

    // 2. Check if entrance reached
    if (controller_->isGoalReached(pose, entrance_point_, 0.40f)) {
        std::cout << "[AUTONOMY] Entrance portal reached! Commencing landing and motor disarm." << std::endl;
        fc_->land();
        fc_->disarm();
        current_state_ = STATE_MISSION_COMPLETED;
        return;
    }

    // 3. Follow path
    navigation::TwistCommand cmd = controller_->computeCommand(pose, current_path_, scan, current_waypoint_idx_);
    fc_->sendVelocity(cmd);
}

void MissionStateMachine::handleEmergencyAbort() {
    // Hard disarm motors immediately
    fc_->emergencyKillSwitch();
}

void MissionStateMachine::broadcastSystemTelemetry() {
    slam::GridPose pose = slam_->getCurrentPose();
    auto now = std::chrono::steady_clock::now();
    uint64_t timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    uint32_t elapsed_sec = (current_state_ != STATE_BOOT_AND_INIT) ?
        static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::seconds>(now - mission_start_time_).count()) : 0;

    net::TelemetryPayload telem;
    telem.timestamp_ms = timestamp_ms;
    telem.pose_x = pose.x;
    telem.pose_y = pose.y;
    telem.pose_yaw_deg = pose.yaw_rad * 180.0f / 3.14159265f;
    telem.vel_x = 0.0f;
    telem.vel_y = 0.0f;
    telem.vel_yaw_deg_s = 0.0f;
    telem.battery_volts = fc_->getBatteryVoltage();
    telem.flight_state = static_cast<uint8_t>(current_state_);
    telem.mission_timer_sec = elapsed_sec;
    telem.survivors_found = static_cast<uint32_t>(vision_->getAllTaggedSurvivors().size());

    tcp_server_->broadcastTelemetry(telem);
}

} // namespace autonomy
} // namespace nidar
