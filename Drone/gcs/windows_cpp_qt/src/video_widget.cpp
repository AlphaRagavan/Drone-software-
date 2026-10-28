#include "video_widget.hpp"
#include <QPainter>
#include <QPaintEvent>
#include <QDebug>

VideoCaptureWorker::VideoCaptureWorker(const QString& stream_url, QObject* parent)
    : QThread(parent), stream_url_(stream_url) {}

VideoCaptureWorker::~VideoCaptureWorker() {
    stop();
}

void VideoCaptureWorker::stop() {
    running_.store(false);
    wait(2000);
}

void VideoCaptureWorker::run() {
    running_.store(true);

    cv::VideoCapture cap;
    // Set network buffering and timeout options for real-time video
    cap.open(stream_url_.toStdString(), cv::CAP_ANY);

    if (!cap.isOpened()) {
        qWarning() << "[Video Worker] Could not connect to stream:" << stream_url_;
    } else {
        qDebug() << "[Video Worker] Streaming live from:" << stream_url_;
    }

    cv::Mat mat, rgb;
    while (running_.load()) {
        if (cap.isOpened() && cap.read(mat) && !mat.empty()) {
            cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
            QImage img(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step), QImage::Format_RGB888);
            emit frameCaptured(img.copy());
        } else {
            // Reconnect attempt or fallback test pattern
            msleep(50);
            if (!cap.isOpened()) {
                cap.open(stream_url_.toStdString(), cv::CAP_ANY);
            }
        }
    }

    if (cap.isOpened()) {
        cap.release();
    }
}

VideoWidget::VideoWidget(QWidget* parent)
    : QWidget(parent) {
    setAttribute(Qt::WA_OpaquePaintEvent);
}

VideoWidget::~VideoWidget() {
    stopStream();
}

void VideoWidget::startStream(const QString& stream_url) {
    stopStream();
    worker_ = new VideoCaptureWorker(stream_url, this);
    connect(worker_, &VideoCaptureWorker::frameCaptured, this, &VideoWidget::onFrameReceived);
    worker_->start();
}

void VideoWidget::stopStream() {
    if (worker_) {
        worker_->stop();
        delete worker_;
        worker_ = nullptr;
    }
}

void VideoWidget::onFrameReceived(const QImage& frame) {
    {
        QMutexLocker lock(&frame_mutex_);
        latest_frame_ = frame;
    }
    update();
}

void VideoWidget::setDetectedSurvivorOverlay(const QString& survivor_id, float conf) {
    active_survivor_ = survivor_id;
    survivor_conf_ = conf;
    alert_flash_timer_ = 60; // flash for 2 seconds
    update();
}

void VideoWidget::paintEvent(QPaintEvent* /*event*/) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillRect(rect(), QColor(10, 14, 20));

    QImage frame_copy;
    {
        QMutexLocker lock(&frame_mutex_);
        if (!latest_frame_.isNull()) {
            frame_copy = latest_frame_;
        }
    }

    if (!frame_copy.isNull()) {
        // Draw frame with aspect ratio preserved
        QSize scaled_sz = frame_copy.size().scaled(size(), Qt::KeepAspectRatio);
        int ox = (width() - scaled_sz.width()) / 2;
        int oy = (height() - scaled_sz.height()) / 2;
        QRect target_rect(ox, oy, scaled_sz.width(), scaled_sz.height());

        p.drawImage(target_rect, frame_copy);

        // Flash survivor target HUD if active
        if (alert_flash_timer_ > 0) {
            alert_flash_timer_--;
            p.setPen(QPen(QColor(255, 45, 85, 220), 3));
            p.setBrush(Qt::NoBrush);
            p.drawRect(target_rect.adjusted(20, 20, -20, -20));

            p.setFont(QFont("Segoe UI", 12, QFont::Bold));
            p.setPen(QColor(255, 45, 85));
            p.drawText(ox + 30, oy + 50, QString("TARGET CONFIRMED: %1 (%2%)")
                       .arg(active_survivor_).arg((int)(survivor_conf_ * 100)));
        }
    } else {
        // Standby radar screen
        p.setFont(QFont("Segoe UI", 11, QFont::Bold));
        p.setPen(QColor(100, 120, 150));
        p.drawText(rect(), Qt::AlignCenter, "CONNECTING TO LIVE VIDEO STREAM...\nrtsp://nidar-drone.local:8554/live");
    }

    // Video status overlay
    p.setFont(QFont("Segoe UI", 9, QFont::Bold));
    p.setPen(QColor(0, 255, 140));
    p.drawText(15, 25, "LIVE ONBOARD OPTICAL CAM [RTSP H.264 / MJPEG]");
}
