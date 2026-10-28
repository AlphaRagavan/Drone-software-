#pragma once

#include <string>
#include <thread>
#include <atomic>
#include <cstdint>

namespace nidar {
namespace net {

class MdnsBroadcaster {
public:
    MdnsBroadcaster(const std::string& hostname = "nidar-drone",
                    const std::string& service_name = "_nidar-drone._tcp",
                    uint16_t port = 9001);
    ~MdnsBroadcaster();

    bool start(const std::string& interface_ip = "192.168.4.1");
    void stop();
    bool isRunning() const { return running_.load(); }

private:
    void broadcastLoop();
    void sendGratuitousAnnouncement();
    void handleIncomingQueries(int sock_fd);

    std::string hostname_;
    std::string service_name_;
    uint16_t service_port_;
    std::string interface_ip_;

    std::atomic<bool> running_{false};
    std::thread worker_thread_;
};

} // namespace net
} // namespace nidar
