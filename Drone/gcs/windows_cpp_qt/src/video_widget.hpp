#pragma once

#include <QWidget>
#include <QImage>
#include <QTimer>
#include <QThread>
#include <QString>
#include <QMutex>

#include <opencv2/opencv.hpp>

class VideoCaptureWorker : public QThread {
    Q_OBJECT
public:
    explicit VideoCaptureWorker(const QString& stream_url, QObject* parent = nullptr);
    ~VideoCaptureWorker();

    void stop();

signals:
    void frameCaptured(const QImage& frame);

protected:
    void run() override;

private:
    QString stream_url_;
    std::atomic<bool> running_{false};
};

class VideoWidget : public QWidget {
    Q_OBJECT
public:
    explicit VideoWidget(QWidget* parent = nullptr);
    ~VideoWidget();

    void startStream(const QString& stream_url);
    void stopStream();

public slots:
    void onFrameReceived(const QImage& frame);
    void setDetectedSurvivorOverlay(const QString& survivor_id, float conf);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    VideoCaptureWorker* worker_{nullptr};
    QImage latest_frame_;
    QMutex frame_mutex_;

    QString active_survivor_;
    float survivor_conf_{0.0f};
    int alert_flash_timer_{0};
};
