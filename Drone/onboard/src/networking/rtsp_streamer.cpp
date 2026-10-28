#include "networking/rtsp_streamer.hpp"
#include <iostream>
#include <sstream>
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
#endif

namespace nidar {
namespace net {

RtspStreamer::RtspStreamer(uint16_t rtsp_port, uint16_t http_port)
    : rtsp_port_(rtsp_port), http_port_(http_port) {}

RtspStreamer::~RtspStreamer() {
    stop();
}

bool RtspStreamer::start() {
    if (running_.load()) return true;
    running_.store(true);

#ifndef _WIN32
    // Setup RTSP socket
    rtsp_server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (rtsp_server_fd_ >= 0) {
        int opt = 1;
        setsockopt(rtsp_server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        struct sockaddr_in addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(rtsp_port_);
        if (bind(rtsp_server_fd_, (struct sockaddr*)&addr, sizeof(addr)) == 0) {
            listen(rtsp_server_fd_, 5);
            int flags = fcntl(rtsp_server_fd_, F_GETFL, 0);
            fcntl(rtsp_server_fd_, F_SETFL, flags | O_NONBLOCK);
            rtsp_thread_ = std::thread(&RtspStreamer::rtspServerLoop, this);
            std::cout << "[RTSP Streamer] Live RTSP endpoint: rtsp://0.0.0.0:" << rtsp_port_ << "/live" << std::endl;
        }
    }

    // Setup HTTP/MJPEG socket
    http_server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (http_server_fd_ >= 0) {
        int opt = 1;
        setsockopt(http_server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        struct sockaddr_in addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(http_port_);
        if (bind(http_server_fd_, (struct sockaddr*)&addr, sizeof(addr)) == 0) {
            listen(http_server_fd_, 5);
            int flags = fcntl(http_server_fd_, F_GETFL, 0);
            fcntl(http_server_fd_, F_SETFL, flags | O_NONBLOCK);
            http_thread_ = std::thread(&RtspStreamer::mjpegServerLoop, this);
            std::cout << "[Video Streamer] HTTP MJPEG stream: http://0.0.0.0:" << http_port_ << "/live.mjpg" << std::endl;
        }
    }
#endif

    return true;
}

void RtspStreamer::stop() {
    if (!running_.load()) return;
    running_.store(false);

    if (rtsp_thread_.joinable()) rtsp_thread_.join();
    if (http_thread_.joinable()) http_thread_.join();

#ifndef _WIN32
    if (rtsp_server_fd_ >= 0) {
        close(rtsp_server_fd_);
        rtsp_server_fd_ = -1;
    }
    if (http_server_fd_ >= 0) {
        close(http_server_fd_);
        http_server_fd_ = -1;
    }
#endif
    std::cout << "[Video Streamer] Sockets stopped." << std::endl;
}

void RtspStreamer::updateFrame(const std::vector<uint8_t>& jpeg_buffer) {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_jpeg_ = jpeg_buffer;
    frame_counter_++;
}

void RtspStreamer::rtspServerLoop() {
    while (running_.load()) {
#ifndef _WIN32
        struct sockaddr_in client_addr;
        socklen_t len = sizeof(client_addr);
        int client_fd = accept(rtsp_server_fd_, (struct sockaddr*)&client_addr, &len);
        if (client_fd >= 0) {
            std::thread(&RtspStreamer::handleRtspClient, this, client_fd).detach();
        }
#endif
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void RtspStreamer::handleRtspClient(int client_fd) {
#ifndef _WIN32
    char buffer[2048];
    while (running_.load()) {
        ssize_t n = recv(client_fd, buffer, sizeof(buffer) - 1, 0);
        if (n <= 0) break;
        buffer[n] = '\0';
        std::string req(buffer);

        std::string cseq = "1";
        size_t cseq_pos = req.find("CSeq: ");
        if (cseq_pos != std::string::npos) {
            size_t end = req.find("\r\n", cseq_pos);
            cseq = req.substr(cseq_pos + 6, end - (cseq_pos + 6));
        }

        std::ostringstream resp;
        if (req.find("OPTIONS") == 0) {
            resp << "RTSP/1.0 200 OK\r\n"
                 << "CSeq: " << cseq << "\r\n"
                 << "Public: OPTIONS, DESCRIBE, SETUP, TEARDOWN, PLAY\r\n\r\n";
        } else if (req.find("DESCRIBE") == 0) {
            std::string sdp = "v=0\r\no=- 0 0 IN IP4 0.0.0.0\r\ns=NIDAR Search & Rescue Stream\r\nt=0 0\r\nm=video 0 RTP/AVP 26\r\nc=IN IP4 0.0.0.0\r\n";
            resp << "RTSP/1.0 200 OK\r\n"
                 << "CSeq: " << cseq << "\r\n"
                 << "Content-Type: application/sdp\r\n"
                 << "Content-Length: " << sdp.length() << "\r\n\r\n"
                 << sdp;
        } else if (req.find("SETUP") == 0) {
            resp << "RTSP/1.0 200 OK\r\n"
                 << "CSeq: " << cseq << "\r\n"
                 << "Session: 12345678\r\n"
                 << "Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n\r\n";
        } else if (req.find("PLAY") == 0) {
            resp << "RTSP/1.0 200 OK\r\n"
                 << "CSeq: " << cseq << "\r\n"
                 << "Session: 12345678\r\n"
                 << "Range: npt=0.000-\r\n\r\n";
            send(client_fd, resp.str().c_str(), resp.str().length(), 0);
            break; // Transition to RTP stream or keep-alive
        } else if (req.find("TEARDOWN") == 0) {
            resp << "RTSP/1.0 200 OK\r\n"
                 << "CSeq: " << cseq << "\r\n\r\n";
            send(client_fd, resp.str().c_str(), resp.str().length(), 0);
            break;
        } else {
            resp << "RTSP/1.0 400 Bad Request\r\n\r\n";
        }

        send(client_fd, resp.str().c_str(), resp.str().length(), 0);
    }
    close(client_fd);
#endif
}

void RtspStreamer::mjpegServerLoop() {
    while (running_.load()) {
#ifndef _WIN32
        struct sockaddr_in client_addr;
        socklen_t len = sizeof(client_addr);
        int client_fd = accept(http_server_fd_, (struct sockaddr*)&client_addr, &len);
        if (client_fd >= 0) {
            std::thread(&RtspStreamer::handleMjpegClient, this, client_fd).detach();
        }
#endif
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void RtspStreamer::handleMjpegClient(int client_fd) {
#ifndef _WIN32
    std::string header = 
        "HTTP/1.0 200 OK\r\n"
        "Server: NIDAR-Streamer\r\n"
        "Connection: close\r\n"
        "Max-Age: 0\r\n"
        "Expires: 0\r\n"
        "Cache-Control: no-cache, private\r\n"
        "Pragma: no-cache\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=--frame\r\n\r\n";

    if (send(client_fd, header.c_str(), header.length(), MSG_NOSIGNAL) < 0) {
        close(client_fd);
        return;
    }

    uint64_t last_sent_frame = 0;
    while (running_.load()) {
        std::vector<uint8_t> frame_copy;
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (frame_counter_ != last_sent_frame && !latest_jpeg_.empty()) {
                frame_copy = latest_jpeg_;
                last_sent_frame = frame_counter_;
            }
        }

        if (!frame_copy.empty()) {
            std::ostringstream part;
            part << "--frame\r\n"
                 << "Content-Type: image/jpeg\r\n"
                 << "Content-Length: " << frame_copy.size() << "\r\n\r\n";
            std::string part_head = part.str();

            if (send(client_fd, part_head.c_str(), part_head.length(), MSG_NOSIGNAL) < 0) break;
            if (send(client_fd, frame_copy.data(), frame_copy.size(), MSG_NOSIGNAL) < 0) break;
            if (send(client_fd, "\r\n", 2, MSG_NOSIGNAL) < 0) break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
    }
    close(client_fd);
#endif
}

} // namespace net
} // namespace nidar
