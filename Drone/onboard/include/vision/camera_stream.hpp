#pragma once

#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>

#ifdef HAVE_OPENCV
#include <opencv2/opencv.hpp>
#else
// Fallback structure if OpenCV headers are missing
namespace cv {
    class Mat {
    public:
        int rows{480};
        int cols{640};
        bool empty() const { return false; }
    };
}
#endif

namespace nidar {
namespace vision {

class CameraStream {
public:
    CameraStream(int device_id = 0, int width = 640, int height = 480, int fps = 30);
    ~CameraStream();

    bool start();
    void stop();
    bool isRunning() const { return running_.load(); }

    bool getLatestFrame(cv::Mat& out_frame);
    bool getLatestJpeg(std::vector<uint8_t>& out_jpeg);

private:
    void captureWorker();
    void generateSimulatedFrame(cv::Mat& frame, uint64_t step);

    int device_id_;
    int width_;
    int height_;
    int fps_;

    std::atomic<bool> running_{false};
    std::thread capture_thread_;

    std::mutex frame_mutex_;
    cv::Mat latest_frame_;
    std::vector<uint8_t> latest_jpeg_;
    uint64_t frame_count_{0};
};

} // namespace vision
} // namespace nidar
