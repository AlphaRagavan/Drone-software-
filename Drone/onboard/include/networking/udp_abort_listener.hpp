#pragma once

#include "networking/protocol_types.hpp"
#include <string>
#include <thread>
#include <atomic>
#include <functional>

namespace nidar {
namespace net {

class UdpAbortListener {
public:
    using AbortCallback = std::function<void(const std::string& reason)>;

    UdpAbortListener(uint16_t port = UDP_ABORT_PORT);
    ~UdpAbortListener();

    bool start(AbortCallback callback);
    void stop();
    bool isAbortTriggered() const { return abort_triggered_.load(); }
    void triggerManualAbort(const std::string& reason);

private:
    void listenerLoop();

    uint16_t port_;
    int sock_fd_{-1};
    std::atomic<bool> running_{false};
    std::atomic<bool> abort_triggered_{false};
    std::thread listener_thread_;
    AbortCallback abort_callback_;
};

} // namespace net
} // namespace nidar
