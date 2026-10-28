#include "map_canvas.hpp"
#include <QPainter>
#include <QPaintEvent>
#include <QMouseEvent>
#include <QWheelEvent>
#include <cmath>

MapCanvas::MapCanvas(QWidget* parent)
    : QWidget(parent) {
    setAttribute(Qt::WA_OpaquePaintEvent);
    setMouseTracking(true);

    anim_timer_ = new QTimer(this);
    connect(anim_timer_, &QTimer::timeout, this, [this]() {
        pulse_phase_ += 0.1f;
        if (pulse_phase_ > 6.28f) pulse_phase_ = 0.0f;
        update();
    });
    anim_timer_->start(50); // 20 FPS animation
}

void MapCanvas::updateMap(int width, int height, float resolution, float origin_x, float origin_y,
                          const QVector<uint8_t>& cells) {
    if (width <= 0 || height <= 0 || cells.size() != width * height) return;

    map_w_ = width;
    map_h_ = height;
    resolution_ = resolution;
    origin_x_ = origin_x;
    origin_y_ = origin_y;

    if (map_image_.size() != QSize(width, height)) {
        map_image_ = QImage(width, height, QImage::Format_ARGB32);
    }

    // Convert occupancy cells to high-contrast tactical display palette
    for (int y = 0; y < height; ++y) {
        QRgb* scanline = reinterpret_cast<QRgb*>(map_image_.scanLine(y));
        for (int x = 0; x < width; ++x) {
            uint8_t cell = cells[y * width_ + x];
            if (cell == 0) {
                // Free corridor space: slate dark gray
                scanline[x] = qRgba(35, 42, 54, 255);
            } else if (cell == 255) {
                // Obstacle wall: crisp luminous cyan
                scanline[x] = qRgba(0, 240, 255, 255);
            } else if (cell == 200) {
                // Inflated collision safety cushion: subtle navy
                scanline[x] = qRgba(20, 30, 45, 255);
            } else {
                // Unknown / unexplored: deep void black
                scanline[x] = qRgba(12, 16, 23, 255);
            }
        }
    }

    update();
}

void MapCanvas::updateDronePose(float x, float y, float yaw_deg) {
    drone_x_ = x;
    drone_y_ = y;
    drone_yaw_ = yaw_deg;

    if (drone_trail_.isEmpty() || std::hypot(drone_trail_.last().x() - x, drone_trail_.last().y() - y) > 0.10) {
        drone_trail_.append(QPointF(x, y));
        if (drone_trail_.size() > 1000) drone_trail_.removeFirst();
    }
    update();
}

void MapCanvas::dropSurvivorMarker(float x, float y, const QString& label, const QString& room_id, float confidence) {
    SurvivorMarker marker;
    marker.x = x;
    marker.y = y;
    marker.label = label;
    marker.room_id = room_id;
    marker.confidence = confidence;
    marker.timestamp = 0;
    survivor_markers_.append(marker);
    update();
}

void MapCanvas::clearMarkers() {
    survivor_markers_.clear();
    drone_trail_.clear();
    update();
}

QPointF MapCanvas::worldToScreen(float wx, float wy) const {
    // Invert Y so upward on screen corresponds to +Y in world
    float sx = pan_offset_.x() + (wx - origin_x_) * scale_;
    float sy = height() - (pan_offset_.y() + (wy - origin_y_) * scale_);
    return QPointF(sx, sy);
}

void MapCanvas::paintEvent(QPaintEvent* /*event*/) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setRenderHint(QPainter::SmoothPixmapTransform, true);

    // Dark HUD background
    p.fillRect(rect(), QColor(10, 14, 20));

    // 1. Draw SLAM Occupancy Grid Map
    if (!map_image_.isNull()) {
        QPointF top_left_world(origin_x_, origin_y_ + map_h_ * resolution_);
        QPointF top_left_screen = worldToScreen(top_left_world.x(), top_left_world.y());
        float screen_w = map_w_ * resolution_ * scale_;
        float screen_h = map_h_ * resolution_ * scale_;
        QRectF dest_rect(top_left_screen.x(), top_left_screen.y(), screen_w, screen_h);
        p.drawImage(dest_rect, map_image_);
    }

    // 2. Draw 2x2 Meter Room Modular Grid Overlay
    p.setPen(QPen(QColor(60, 80, 110, 100), 1, Qt::DashLine));
    for (float rx = 0.0f; rx <= 16.0f; rx += 2.0f) {
        QPointF p1 = worldToScreen(rx, 0.0f);
        QPointF p2 = worldToScreen(rx, 16.0f);
        p.drawLine(p1, p2);
    }
    for (float ry = 0.0f; ry <= 16.0f; ry += 2.0f) {
        QPointF p1 = worldToScreen(0.0f, ry);
        QPointF p2 = worldToScreen(16.0f, ry);
        p.drawLine(p1, p2);
    }

    // Room labels
    p.setFont(QFont("Consolas", 8));
    p.setPen(QColor(100, 130, 170, 140));
    for (int col = 0; col < 8; ++col) {
        for (int row = 0; row < 8; ++row) {
            QPointF center = worldToScreen(col * 2.0f + 0.2f, row * 2.0f + 1.8f);
            p.drawText(center, QString("R[%1,%2]").arg(col).arg(row));
        }
    }

    // 3. Draw Historic Flight Trajectory Trail
    if (drone_trail_.size() > 1) {
        p.setPen(QPen(QColor(0, 200, 255, 180), 2, Qt::SolidLine));
        for (int i = 0; i < drone_trail_.size() - 1; ++i) {
            p.drawLine(worldToScreen(drone_trail_[i].x(), drone_trail_[i].y()),
                       worldToScreen(drone_trail_[i + 1].x(), drone_trail_[i + 1].y()));
        }
    }

    // 4. Plot Survivor Markers (Instant drop on survivor JSON payload)
    float pulse_radius = 12.0f + 6.0f * std::sin(pulse_phase_);
    for (const auto& marker : survivor_markers_) {
        QPointF sp = worldToScreen(marker.x, marker.y);

        // Animated glowing pulse aura
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(255, 45, 85, 60));
        p.drawEllipse(sp, pulse_radius + 8.0f, pulse_radius + 8.0f);

        // Solid beacon ring
        p.setPen(QPen(QColor(255, 45, 85), 3));
        p.setBrush(QColor(255, 45, 85, 180));
        p.drawEllipse(sp, 9.0f, 9.0f);

        // Crosshairs
        p.setPen(QPen(Qt::white, 2));
        p.drawLine(sp.x() - 14, sp.y(), sp.x() + 14, sp.y());
        p.drawLine(sp.x(), sp.y() - 14, sp.x(), sp.y() + 14);

        // Callout Tag Card
        QRectF tag_rect(sp.x() + 14, sp.y() - 28, 110, 40);
        p.setPen(QPen(QColor(255, 45, 85), 1));
        p.setBrush(QColor(20, 24, 34, 230));
        p.drawRoundedRect(tag_rect, 4, 4);

        p.setFont(QFont("Segoe UI", 8, QFont::Bold));
        p.setPen(Qt::white);
        p.drawText(tag_rect.adjusted(6, 4, -4, -20), marker.label);

        p.setFont(QFont("Consolas", 7));
        p.setPen(QColor(0, 240, 255));
        p.drawText(tag_rect.adjusted(6, 20, -4, -2), QString("%1 (%2%)").arg(marker.room_id).arg((int)(marker.confidence * 100)));
    }

    // 5. Draw Autonomous Drone Icon & Heading
    QPointF dp = worldToScreen(drone_x_, drone_y_);
    float yaw_rad = drone_yaw_ * 3.14159265f / 180.0f;

    // Heading beam
    float head_len = 28.0f;
    QPointF head_pt(dp.x() + head_len * std::cos(yaw_rad), dp.y() - head_len * std::sin(yaw_rad));
    p.setPen(QPen(QColor(0, 255, 140, 200), 2, Qt::DashLine));
    p.drawLine(dp, head_pt);

    // Drone body circle
    p.setPen(QPen(QColor(0, 255, 140), 2));
    p.setBrush(QColor(10, 40, 30, 220));
    p.drawEllipse(dp, 10.0f, 10.0f);

    // Heading chevron
    QPolygonF chevron;
    chevron << QPointF(dp.x() + 12.0f * std::cos(yaw_rad), dp.y() - 12.0f * std::sin(yaw_rad))
            << QPointF(dp.x() + 8.0f * std::cos(yaw_rad + 2.5f), dp.y() - 8.0f * std::sin(yaw_rad + 2.5f))
            << QPointF(dp.x() + 8.0f * std::cos(yaw_rad - 2.5f), dp.y() - 8.0f * std::sin(yaw_rad - 2.5f));
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(0, 255, 140));
    p.drawPolygon(chevron);

    // Canvas Status watermark
    p.setFont(QFont("Segoe UI", 9, QFont::Bold));
    p.setPen(QColor(120, 140, 170, 160));
    p.drawText(15, 25, "ARENA MAP 15m x 15m [SLAM OCCUPANCY GRID - 5cm/cell]");
}

void MapCanvas::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        dragging_ = true;
        last_mouse_pos_ = event->pos();
    }
}

void MapCanvas::mouseMoveEvent(QMouseEvent* event) {
    if (dragging_) {
        QPoint delta = event->pos() - last_mouse_pos_;
        pan_offset_.rx() += delta.x();
        pan_offset_.ry() -= delta.y(); // invert Y
        last_mouse_pos_ = event->pos();
        update();
    }
}

void MapCanvas::wheelEvent(QWheelEvent* event) {
    float factor = (event->angleDelta().y() > 0) ? 1.15f : 0.85f;
    scale_ = std::clamp(scale_ * factor, 10.0f, 100.0f);
    update();
}
