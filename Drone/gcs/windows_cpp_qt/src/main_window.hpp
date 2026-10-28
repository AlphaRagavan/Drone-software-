#pragma once

#include <QMainWindow>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QProgressBar>

#include "bonjour_resolver.hpp"
#include "telemetry_client.hpp"
#include "abort_controller.hpp"
#include "map_canvas.hpp"
#include "video_widget.hpp"

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() = default;

private slots:
    void onDroneResolved(const QString& ip_address);
    void onTelemetryReceived(const nidar::net::TelemetryPayload& telem);
    void onMapReceived(int w, int h, float res, float ox, float oy, const QVector<uint8_t>& cells);
    void onSurvivorTagged(const QJsonObject& json);
    void onEmergencyAbortClicked();

private:
    void setupUi();

    BonjourResolver* bonjour_{nullptr};
    TelemetryClient* telemetry_{nullptr};
    AbortController* abort_{nullptr};

    // UI Widgets
    MapCanvas* map_canvas_{nullptr};
    VideoWidget* video_widget_{nullptr};
    QTableWidget* survivor_table_{nullptr};

    // Header Indicators
    QLabel* status_label_{nullptr};
    QLabel* ip_label_{nullptr};
    QLabel* timer_label_{nullptr};
    QLabel* battery_label_{nullptr};
    QLabel* state_label_{nullptr};
    QPushButton* abort_button_{nullptr};

    QString drone_ip_;
};
