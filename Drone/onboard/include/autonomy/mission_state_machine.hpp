#pragma once

#include "slam/occupancy_grid.hpp"
#include "slam/hector_slam_2d.hpp"
#include "lidar/rplidar_driver.hpp"
#include "navigation/frontier_explorer.hpp"
#include "navigation/path_planner.hpp"
#include "navigation/kinematics_controller.hpp"
#include "vision/camera_stream.hpp"
#include "vision/survivor_detector.hpp"
#include "networking/tcp_telemetry_server.hpp"
#include "networking/udp_abort_listener.hpp"
#include "networking/mdns_broadcaster.hpp"
#include "networking/rtsp_streamer.hpp"
#include "hal/flight_controller_interface.hpp"

#include <memory>
#include <chrono>
#include <string>

namespace nidar {
namespace autonomy {

enum MissionState {
    STATE_BOOT_AND_INIT       = 0,
    STATE_TAKEOFF_AND_ENTER   = 1,
    STATE_EXPLORE_MAZE        = 2,
    STATE_TAG_SURVIVOR        = 3,
    STATE_FIND_EXIT_AND_LAND  = 4,
    STATE_EMERGENCY_ABORT     = 5,
    STATE_MISSION_COMPLETED   = 6
};

class MissionStateMachine {
public:
    MissionStateMachine();
    ~MissionStateMachine();

    bool initialize(const std::string& lidar_port = "/dev/ttyUSB0",
                    const std::string& model_path = "models/yolov8n.onnx",
                    const std::string& fc_port = "/dev/ttyS0");

    // Single step iteration of the 20 Hz state machine
    void step();

    void triggerEmergencyAbort(const std::string& reason);
    MissionState getCurrentState() const { return current_state_; }
    std::string getStateName() const;

    bool isRunning() const { return current_state_ != STATE_MISSION_COMPLETED &&
                                    current_state_ != STATE_EMERGENCY_ABORT; }

private:
    void handleTakeoffAndEnter();
    void handleExploreMaze();
    void handleTagSurvivor();
    void handleFindExitAndLand();
    void handleEmergencyAbort();

    void broadcastSystemTelemetry();

    MissionState current_state_{STATE_BOOT_AND_INIT};

    // Subsystem components
    std::shared_ptr<slam::OccupancyGrid> grid_;
    std::unique_ptr<slam::HectorSlam2D> slam_;
    std::unique_ptr<lidar::NidarRplidar> lidar_;
    std::unique_ptr<navigation::FrontierExplorer> explorer_;
    std::unique_ptr<navigation::PathPlanner> planner_;
    std::unique_ptr<navigation::KinematicsController> controller_;
    std::unique_ptr<vision::CameraStream> camera_;
    std::unique_ptr<vision::SurvivorDetector> vision_;
    std::unique_ptr<hal::FlightControllerInterface> fc_;

    // Networking
    std::unique_ptr<net::MdnsBroadcaster> mdns_;
    std::unique_ptr<net::TcpTelemetryServer> tcp_server_;
    std::unique_ptr<net::UdpAbortListener> udp_abort_;
    std::unique_ptr<net::RtspStreamer> video_streamer_;

    // Mission Timing (30-minute competition ceiling)
    std::chrono::steady_clock::time_point mission_start_time_;
    static constexpr uint32_t MAX_MISSION_TIME_SEC = 1800;      // 30 minutes
    static constexpr uint32_t RETURN_TO_HOME_TIME_SEC = 1620;   // 27 minutes (3m safe return window)
    static constexpr uint32_t HARD_TIMEOUT_LAND_SEC = 1770;     // 29.5 minutes hard auto-land

    // Navigation and Tagging State
    navigation::Waypoint entrance_point_{1.0f, 1.0f};
    navigation::FrontierCluster current_frontier_;
    std::vector<navigation::Waypoint> current_path_;
    size_t current_waypoint_idx_{0};
    uint64_t state_entry_time_ms_{0};

    // Staged survivor tag
    vision::TaggedSurvivor candidate_survivor_;
    int survivor_confirmation_counter_{0};

    uint64_t iteration_count_{0};
};

} // namespace autonomy
} // namespace nidar
