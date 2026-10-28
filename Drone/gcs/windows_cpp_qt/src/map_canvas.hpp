#pragma once

#include <QWidget>
#include <QImage>
#include <QVector>
#include <QPointF>
#include <QString>
#include <QTimer>

struct SurvivorMarker {
    float x;
    float y;
    QString label;
    QString room_id;
    float confidence;
    uint64_t timestamp;
};

class MapCanvas : public QWidget {
    Q_OBJECT
public:
    explicit MapCanvas(QWidget* parent = nullptr);
    ~MapCanvas() = default;

    void updateMap(int width, int height, float resolution, float origin_x, float origin_y,
                   const QVector<uint8_t>& cells);
    void updateDronePose(float x, float y, float yaw_deg);
    void dropSurvivorMarker(float x, float y, const QString& label, const QString& room_id, float confidence);
    void clearMarkers();

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    QPointF worldToScreen(float wx, float wy) const;

    int map_w_{0};
    int map_h_{0};
    float resolution_{0.05f};
    float origin_x_{0.0f};
    float origin_y_{0.0f};

    QImage map_image_;
    float drone_x_{1.0f};
    float drone_y_{1.0f};
    float drone_yaw_{0.0f};
    QVector<QPointF> drone_trail_;

    QVector<SurvivorMarker> survivor_markers_;

    // Viewport transform (pan & zoom)
    float scale_{30.0f}; // pixels per meter
    QPointF pan_offset_{50.0f, 50.0f};
    QPoint last_mouse_pos_;
    bool dragging_{false};

    float pulse_phase_{0.0f};
    QTimer* anim_timer_{nullptr};
};
