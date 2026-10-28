#include "networking/mdns_broadcaster.hpp"
#include <iostream>
#include <vector>
#include <cstring>
#include <chrono>

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
#include <net/if.h>
#endif

namespace nidar {
namespace net {

static const char* MDNS_MULTICAST_ADDR = "224.0.0.251";
static const uint16_t MDNS_PORT = 5353;

MdnsBroadcaster::MdnsBroadcaster(const std::string& hostname,
                                 const std::string& service_name,
                                 uint16_t port)
    : hostname_(hostname), service_name_(service_name), service_port_(port) {}

MdnsBroadcaster::~MdnsBroadcaster() {
    stop();
}

bool MdnsBroadcaster::start(const std::string& interface_ip) {
    if (running_.load()) return true;

    interface_ip_ = interface_ip;
    running_.store(true);
    worker_thread_ = std::thread(&MdnsBroadcaster::broadcastLoop, this);
    std::cout << "[mDNS] Broadcaster active: " << hostname_ << ".local (" << interface_ip_ 
              << ") advertising " << service_name_ << " on port " << service_port_ << std::endl;
    return true;
}

void MdnsBroadcaster::stop() {
    if (!running_.load()) return;
    running_.store(false);
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
    std::cout << "[mDNS] Broadcaster stopped." << std::endl;
}

static void encodeDnsName(const std::string& name, std::vector<uint8_t>& buf) {
    size_t start = 0;
    while (start < name.size()) {
        size_t dot = name.find('.', start);
        if (dot == std::string::npos) dot = name.size();
        uint8_t len = static_cast<uint8_t>(dot - start);
        buf.push_back(len);
        for (size_t i = start; i < dot; ++i) {
            buf.push_back(static_cast<uint8_t>(name[i]));
        }
        start = dot + 1;
    }
    buf.push_back(0); // terminating zero length label
}

void MdnsBroadcaster::sendGratuitousAnnouncement() {
#ifndef _WIN32
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return;

    unsigned char ttl = 255;
    setsockopt(sock, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));

    struct in_addr if_addr;
    if_addr.s_addr = inet_addr(interface_ip_.c_str());
    setsockopt(sock, IPPROTO_IP, IP_MULTICAST_IF, &if_addr, sizeof(if_addr));

    struct sockaddr_in dest_addr;
    std::memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(MDNS_PORT);
    dest_addr.sin_addr.s_addr = inet_addr(MDNS_MULTICAST_ADDR);

    // Build standard DNS-SD / mDNS unsolicited response packet
    std::vector<uint8_t> pkt;
    // ID = 0, Flags = 0x8400 (Standard response, Authoritative Answer)
    pkt.push_back(0x00); pkt.push_back(0x00);
    pkt.push_back(0x84); pkt.push_back(0x00);
    // Questions = 0, Answer RRs = 2, Authority RRs = 0, Additional RRs = 0
    pkt.push_back(0x00); pkt.push_back(0x00); // QDCOUNT
    pkt.push_back(0x00); pkt.push_back(0x02); // ANCOUNT = 2 (A record + PTR)
    pkt.push_back(0x00); pkt.push_back(0x00); // NSCOUNT
    pkt.push_back(0x00); pkt.push_back(0x00); // ARCOUNT

    // Record 1: A record for hostname.local
    encodeDnsName(hostname_ + ".local", pkt);
    // Type = A (1), Class = IN (1) with Cache-Flush bit (0x8001)
    pkt.push_back(0x00); pkt.push_back(0x01);
    pkt.push_back(0x80); pkt.push_back(0x01);
    // TTL = 120 seconds
    pkt.push_back(0x00); pkt.push_back(0x00); pkt.push_back(0x00); pkt.push_back(0x78);
    // RDLENGTH = 4 bytes (IPv4)
    pkt.push_back(0x00); pkt.push_back(0x04);
    uint32_t ip = inet_addr(interface_ip_.c_str());
    const uint8_t* ip_bytes = reinterpret_cast<const uint8_t*>(&ip);
    pkt.push_back(ip_bytes[0]); pkt.push_back(ip_bytes[1]);
    pkt.push_back(ip_bytes[2]); pkt.push_back(ip_bytes[3]);

    // Record 2: PTR record for service
    encodeDnsName(service_name_ + ".local", pkt);
    pkt.push_back(0x00); pkt.push_back(0x0C); // Type PTR (12)
    pkt.push_back(0x00); pkt.push_back(0x01); // Class IN
    pkt.push_back(0x00); pkt.push_back(0x00); pkt.push_back(0x11); pkt.push_back(0x94); // TTL 4500s
    std::vector<uint8_t> target_name;
    encodeDnsName(hostname_ + "." + service_name_ + ".local", target_name);
    pkt.push_back(static_cast<uint8_t>(target_name.size() >> 8));
    pkt.push_back(static_cast<uint8_t>(target_name.size() & 0xFF));
    pkt.insert(pkt.end(), target_name.begin(), target_name.end());

    sendto(sock, reinterpret_cast<const char*>(pkt.data()), pkt.size(), 0,
           reinterpret_cast<struct sockaddr*>(&dest_addr), sizeof(dest_addr));
    close(sock);
#endif
}

void MdnsBroadcaster::broadcastLoop() {
    while (running_.load()) {
        sendGratuitousAnnouncement();
        // Periodically announce every 2 seconds on the 5GHz Wi-Fi link
        for (int i = 0; i < 20 && running_.load(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

} // namespace net
} // namespace nidar
