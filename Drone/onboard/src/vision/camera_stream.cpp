#include "vision/camera_stream.hpp"
#include <iostream>
#include <chrono>

#ifdef HAVE_OPENCV
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#endif

namespace nidar {
namespace vision {

CameraStream::CameraStream(int device_id, int width, int height, int fps)
    : device_id_(device_id), width_(width), height_(height), fps_(fps) {}

CameraStream::~CameraStream() {
    stop();
}

bool CameraStream::start() {
    if (running_.load()) return true;
    running_.store(true);
    capture_thread_ = std::thread(&CameraStream::captureWorker, this);
    std::cout << "[Camera] Camera stream pipeline started (" << width_ << "x" << height_ << " @ " << fps_ << "fps)." << std::endl;
    return true;
}

void CameraStream::stop() {
    if (!running_.load()) return;
    running_.store(false);
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    std::cout << "[Camera] Stream pipeline stopped." << std::endl;
}

void CameraStream::generateSimulatedFrame(cv::Mat& frame, uint64_t step) {
#ifdef HAVE_OPENCV
    frame = cv::Mat(height_, width_, CV_8UC3, cv::Scalar(30, 30, 35));

    // Draw indoor 1-meter corridor perspective lines
    cv::line(frame, cv::Point(0, height_), cv::Point(width_ / 2 - 40, height_ / 2), cv::Scalar(80, 80, 90), 2);
    cv::line(frame, cv::Point(width_, height_), cv::Point(width_ / 2 + 40, height_ / 2), cv::Scalar(80, 80, 90), 2);
    cv::line(frame, cv::Point(0, 0), cv::Point(width_ / 2 - 40, height_ / 2 - 40), cv::Scalar(60, 60, 70), 2);
    cv::line(frame, cv::Point(width_, 0), cv::Point(width_ / 2 + 40, height_ / 2 - 40), cv::Scalar(60, 60, 70), 2);

    // Overlay simulated survivor dummy at regular intervals
    if ((step / 90) % 2 == 1) {
        int x = width_ / 2 + static_cast<int>(30.0f * std::sin(step * 0.05f));
        int y = height_ / 2 + 40;
        // Head
        cv::circle(frame, cv::Point(x, y - 30), 18, cv::Scalar(0, 140, 255), -1);
        // Torso (high-visibility search & rescue dummy vest)
        cv::rectangle(frame, cv::Point(x - 25, y - 10), cv::Point(x + 25, y + 55), cv::Scalar(0, 200, 240), -1);
        cv::putText(frame, "RESCUE TARGET", cv::Point(x - 45, y - 55), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 0), 1);
    }

    // Telemetry HUD overlay
    std::string hud = "CAM01 [NIDAR AIRMOUSE] 100% OFFLINE";
    cv::putText(frame, hud, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 128), 1);
#else
    (void)frame;
    (void)step;
#endif
}

void CameraStream::captureWorker() {
#ifdef HAVE_OPENCV
    cv::VideoCapture cap;
    // Attempt opening physical V4L2 device
    cap.open(device_id_, cv::CAP_V4L2);
    if (cap.isOpened()) {
        cap.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        cap.set(cv::CAP_PROP_FPS, fps_);
        std::cout << "[Camera] Successfully opened physical camera /dev/video" << device_id_ << std::endl;
    } else {
        std::cout << "[Camera] Physical camera offline. Utilizing autonomous indoor vision simulator." << std::endl;
    }

    cv::Mat frame;
    std::vector<uint8_t> jpeg_buf;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 75};

    while (running_.load()) {
        if (cap.isOpened()) {
            if (!cap.read(frame) || frame.empty()) {
                generateSimulatedFrame(frame, frame_count_);
            }
        } else {
            generateSimulatedFrame(frame, frame_count_);
        }

        cv::imencode(".jpg", frame, jpeg_buf, params);

        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_frame_ = frame.clone();
            latest_jpeg_ = jpeg_buf;
            frame_count_++;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / fps_));
    }

    if (cap.isOpened()) {
        cap.release();
    }
#else
    while (running_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
#endif
}

bool CameraStream::getLatestFrame(cv::Mat& out_frame) {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (latest_frame_.empty()) return false;
    out_frame = latest_frame_.clone();
    return true;
}

bool CameraStream::getLatestJpeg(std::vector<uint8_t>& out_jpeg) {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (latest_jpeg_.empty()) return false;
    out_jpeg = latest_jpeg_;
    return true;
}

} // namespace vision
} // namespace nidar
