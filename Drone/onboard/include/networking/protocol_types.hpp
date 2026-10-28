#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace nidar {
namespace net {

#pragma pack(push, 1)

constexpr uint16_t NIDAR_PACKET_MAGIC = 0xAA55;

enum PacketType : uint8_t {
    PKT_HEARTBEAT       = 0x01,
    PKT_TELEMETRY       = 0x02,
    PKT_OCCUPANCY_GRID  = 0x03,
    PKT_SURVIVOR_TAG    = 0x04,
    PKT_FRONTIER_GOAL   = 0x05,
    PKT_MISSION_STATE   = 0x06,
    PKT_EMERGENCY_ABORT = 0xFF
};

struct PacketHeader {
    uint16_t magic;       // 0xAA55
    uint8_t  type;        // PacketType
    uint32_t payload_len; // Size of payload following header
};

struct TelemetryPayload {
    uint64_t timestamp_ms;
    float    pose_x;           // meters in arena frame
    float    pose_y;           // meters in arena frame
    float    pose_yaw_deg;     // degrees [-180, 180]
    float    vel_x;            // m/s
    float    vel_y;            // m/s
    float    vel_yaw_deg_s;    // deg/s
    float    battery_volts;    // LiPo voltage
    uint8_t  flight_state;     // State enum value
    uint32_t mission_timer_sec;// Mission elapsed time
    uint32_t survivors_found;  // Total confirmed survivor count
};

struct GridMapHeader {
    uint32_t width_cells;     // e.g. 320
    uint32_t height_cells;    // e.g. 320
    float    resolution_m;    // e.g. 0.05 (5cm)
    float    origin_x_m;      // e.g. 0.0
    float    origin_y_m;      // e.g. 0.0
    uint32_t compressed_len;  // RLE compressed byte size
};

#pragma pack(pop)

// Abort command token
constexpr const char* EMERGENCY_ABORT_TOKEN = "EMERGENCY_ABORT_NIDAR_HALT";
constexpr uint16_t UDP_ABORT_PORT = 9999;
constexpr uint16_t TCP_TELEMETRY_PORT = 9001;
constexpr uint16_t RTSP_STREAM_PORT = 8554;
constexpr uint16_t HTTP_MJPEG_PORT = 8080;
constexpr const char* MDNS_HOSTNAME = "nidar-drone.local";
constexpr const char* MDNS_SERVICE = "_nidar-drone._tcp.local";

} // namespace net
} // namespace nidar
