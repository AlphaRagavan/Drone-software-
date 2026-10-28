#pragma once

#include "networking/protocol_types.hpp"
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>

namespace nidar {
namespace net {

class TcpTelemetryServer {
public:
    TcpTelemetryServer(uint16_t port = TCP_TELEMETRY_PORT);
    ~TcpTelemetryServer();

    bool start();
    void stop();
    bool isRunning() const { return running_.load(); }

    // Broadcast methods for real-time telemetry stream
    void broadcastTelemetry(const TelemetryPayload& telemetry);
    void broadcastMap(const GridMapHeader& header, const std::vector<uint8_t>& map_data);
    void broadcastSurvivorTag(const std::string& survivor_json);
    void broadcastStateMessage(const std::string& state_text);

    size_t getConnectedClientsCount() const;

private:
    void acceptLoop();
    void broadcastRawPacket(PacketType type, const void* payload, uint32_t payload_len);
    void closeClient(int client_fd);

    uint16_t port_;
    int server_fd_{-1};
    std::atomic<bool> running_{false};
    std::thread accept_thread_;

    mutable std::mutex clients_mutex_;
    std::vector<int> client_fds_;
};

} // namespace net
} // namespace nidar
