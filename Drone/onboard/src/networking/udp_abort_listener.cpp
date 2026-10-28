#include "networking/udp_abort_listener.hpp"
#include <iostream>
#include <cstring>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#endif

namespace nidar {
namespace net {

UdpAbortListener::UdpAbortListener(uint16_t port)
    : port_(port) {}

UdpAbortListener::~UdpAbortListener() {
    stop();
}

bool UdpAbortListener::start(AbortCallback callback) {
    if (running_.load()) return true;

    abort_callback_ = callback;
    abort_triggered_.store(false);

#ifndef _WIN32
    sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        std::cerr << "[Abort Listener] Socket creation failed." << std::endl;
        return false;
    }

    int opt = 1;
    setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Set receive timeout so the listener can loop cleanly and exit
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100ms timeout
    setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port_);

    if (bind(sock_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "[Abort Listener] Failed to bind to UDP port " << port_ << std::endl;
        close(sock_fd_);
        sock_fd_ = -1;
        return false;
    }
#endif

    running_.store(true);
    listener_thread_ = std::thread(&UdpAbortListener::listenerLoop, this);

#ifndef _WIN32
    // Elevate priority of abort listener thread
    sched_param sch_params;
    sch_params.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_setschedparam(listener_thread_.native_handle(), SCHED_FIFO, &sch_params);
#endif

    std::cout << "[Abort Listener] High-Priority UDP abort socket active on port " << port_ << std::endl;
    return true;
}

void UdpAbortListener::stop() {
    if (!running_.load()) return;
    running_.store(false);

    if (listener_thread_.joinable()) {
        listener_thread_.join();
    }

#ifndef _WIN32
    if (sock_fd_ >= 0) {
        close(sock_fd_);
        sock_fd_ = -1;
    }
#endif
    std::cout << "[Abort Listener] UDP abort listener stopped." << std::endl;
}

void UdpAbortListener::triggerManualAbort(const std::string& reason) {
    if (!abort_triggered_.exchange(true)) {
        std::cerr << "\n========================================================" << std::endl;
        std::cerr << "[CRITICAL] EMERGENCY ABORT TRIGGERED: " << reason << std::endl;
        std::cerr << "========================================================\n" << std::endl;
        if (abort_callback_) {
            abort_callback_(reason);
        }
    }
}

void UdpAbortListener::listenerLoop() {
    char buffer[512];
    while (running_.load()) {
#ifndef _WIN32
        struct sockaddr_in src_addr;
        socklen_t addr_len = sizeof(src_addr);
        ssize_t bytes = recvfrom(sock_fd_, buffer, sizeof(buffer) - 1, 0,
                                 (struct sockaddr*)&src_addr, &addr_len);

        if (bytes > 0) {
            buffer[bytes] = '\0';
            std::string msg(buffer);

            char sender_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &src_addr.sin_addr, sender_ip, INET_ADDRSTRLEN);

            // Check for emergency abort token or abort packet
            if (msg.find(EMERGENCY_ABORT_TOKEN) != std::string::npos ||
                (bytes >= 3 && static_cast<uint8_t>(buffer[2]) == PKT_EMERGENCY_ABORT)) {
                
                std::string reason = "UDP command received from GCS at " + std::string(sender_ip);
                triggerManualAbort(reason);
                break;
            }
        }
#endif
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

} // namespace net
} // namespace nidar
