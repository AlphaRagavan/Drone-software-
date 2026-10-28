#pragma once

#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstdint>

namespace cv {
    class Mat;
}

namespace nidar {
namespace net {

class RtspStreamer {
public:
    RtspStreamer(uint16_t rtsp_port = 8554, uint16_t http_mjpeg_port = 8080);
    ~RtspStreamer();

    bool start();
    void stop();
    bool isRunning() const { return running_.load(); }

    // Push latest OpenCV frame into stream pipeline
    void updateFrame(const std::vector<uint8_t>& jpeg_buffer);

private:
    void rtspServerLoop();
    void mjpegServerLoop();
    void handleRtspClient(int client_fd);
    void handleMjpegClient(int client_fd);

    uint16_t rtsp_port_;
    uint16_t http_port_;
    int rtsp_server_fd_{-1};
    int http_server_fd_{-1};

    std::atomic<bool> running_{false};
    std::thread rtsp_thread_;
    std::thread http_thread_;

    std::mutex frame_mutex_;
    std::vector<uint8_t> latest_jpeg_;
    uint64_t frame_counter_{0};
};

} // namespace net
} // namespace nidar
