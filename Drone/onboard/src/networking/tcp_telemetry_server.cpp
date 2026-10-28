#include "networking/tcp_telemetry_server.hpp"
#include <iostream>
#include <cstring>
#include <algorithm>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#endif

namespace nidar {
namespace net {

TcpTelemetryServer::TcpTelemetryServer(uint16_t port)
    : port_(port) {}

TcpTelemetryServer::~TcpTelemetryServer() {
    stop();
}

bool TcpTelemetryServer::start() {
    if (running_.load()) return true;

#ifndef _WIN32
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
        std::cerr << "[TCP Server] Error creating socket." << std::endl;
        return false;
    }

    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in address;
    std::memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port_);

    if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) {
        std::cerr << "[TCP Server] Error binding to port " << port_ << std::endl;
        close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    if (listen(server_fd_, 5) < 0) {
        std::cerr << "[TCP Server] Error listening on port " << port_ << std::endl;
        close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    // Set non-blocking on server socket so accept can exit cleanly on stop
    int flags = fcntl(server_fd_, F_GETFL, 0);
    fcntl(server_fd_, F_SETFL, flags | O_NONBLOCK);
#endif

    running_.store(true);
    accept_thread_ = std::thread(&TcpTelemetryServer::acceptLoop, this);
    std::cout << "[TCP Server] Telemetry and Map server listening on port " << port_ << std::endl;
    return true;
}

void TcpTelemetryServer::stop() {
    if (!running_.load()) return;
    running_.store(false);

    if (accept_thread_.joinable()) {
        accept_thread_.join();
    }

    std::lock_guard<std::mutex> lock(clients_mutex_);
#ifndef _WIN32
    for (int fd : client_fds_) {
        close(fd);
    }
    client_fds_.clear();

    if (server_fd_ >= 0) {
        close(server_fd_);
        server_fd_ = -1;
    }
#endif
    std::cout << "[TCP Server] Telemetry server stopped." << std::endl;
}

void TcpTelemetryServer::acceptLoop() {
    while (running_.load()) {
#ifndef _WIN32
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        int client_fd = accept(server_fd_, (struct sockaddr*)&client_addr, &addr_len);

        if (client_fd >= 0) {
            // Enable TCP_NODELAY for ultra-low latency real-time telemetry
            int flag = 1;
            setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, (char*)&flag, sizeof(int));

            char client_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
            std::cout << "[TCP Server] GCS Client connected from " << client_ip << std::endl;

            std::lock_guard<std::mutex> lock(clients_mutex_);
            client_fds_.push_back(client_fd);
        }
#endif
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void TcpTelemetryServer::closeClient(int client_fd) {
#ifndef _WIN32
    close(client_fd);
#endif
}

void TcpTelemetryServer::broadcastRawPacket(PacketType type, const void* payload, uint32_t payload_len) {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    if (client_fds_.empty()) return;

    PacketHeader header;
    header.magic = NIDAR_PACKET_MAGIC;
    header.type = static_cast<uint8_t>(type);
    header.payload_len = payload_len;

    std::vector<uint8_t> buffer(sizeof(header) + payload_len);
    std::memcpy(buffer.data(), &header, sizeof(header));
    if (payload && payload_len > 0) {
        std::memcpy(buffer.data() + sizeof(header), payload, payload_len);
    }

    std::vector<int> dead_clients;

    for (int fd : client_fds_) {
#ifndef _WIN32
        ssize_t sent = send(fd, buffer.data(), buffer.size(), MSG_NOSIGNAL);
        if (sent < 0) {
            dead_clients.push_back(fd);
        }
#endif
    }

    // Clean up disconnected clients
    for (int dead_fd : dead_clients) {
        std::cout << "[TCP Server] GCS client disconnected." << std::endl;
        closeClient(dead_fd);
        client_fds_.erase(std::remove(client_fds_.begin(), client_fds_.end(), dead_fd), client_fds_.end());
    }
}

void TcpTelemetryServer::broadcastTelemetry(const TelemetryPayload& telemetry) {
    broadcastRawPacket(PKT_TELEMETRY, &telemetry, sizeof(telemetry));
}

void TcpTelemetryServer::broadcastMap(const GridMapHeader& header, const std::vector<uint8_t>& map_data) {
    std::vector<uint8_t> payload(sizeof(header) + map_data.size());
    std::memcpy(payload.data(), &header, sizeof(header));
    std::memcpy(payload.data() + sizeof(header), map_data.data(), map_data.size());
    broadcastRawPacket(PKT_OCCUPANCY_GRID, payload.data(), static_cast<uint32_t>(payload.size()));
}

void TcpTelemetryServer::broadcastSurvivorTag(const std::string& survivor_json) {
    broadcastRawPacket(PKT_SURVIVOR_TAG, survivor_json.data(), static_cast<uint32_t>(survivor_json.size()));
}

void TcpTelemetryServer::broadcastStateMessage(const std::string& state_text) {
    broadcastRawPacket(PKT_MISSION_STATE, state_text.data(), static_cast<uint32_t>(state_text.size()));
}

size_t TcpTelemetryServer::getConnectedClientsCount() const {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    return client_fds_.size();
}

} // namespace net
} // namespace nidar
