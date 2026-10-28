#include "main_window.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSplitter>
#include <QHeaderView>
#include <QMessageBox>
#include <QDateTime>
#include <QDebug>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent) {
    setupUi();

    bonjour_ = new BonjourResolver("nidar-drone.local", this);
    telemetry_ = new TelemetryClient(this);
    abort_ = new AbortController(this);

    connect(bonjour_, &BonjourResolver::hostResolved, this, &MainWindow::onDroneResolved);
    connect(telemetry_, &TelemetryClient::telemetryReceived, this, &MainWindow::onTelemetryReceived);
    connect(telemetry_, &TelemetryClient::mapReceived, this, &MainWindow::onMapReceived);
    connect(telemetry_, &TelemetryClient::survivorTagged, this, &MainWindow::onSurvivorTagged);
    connect(abort_button_, &QPushButton::clicked, this, &MainWindow::onEmergencyAbortClicked);

    // Commence Bonjour discovery on isolated 5GHz Wi-Fi link
    bonjour_->startDiscovery();
}

void MainWindow::setupUi() {
    setWindowTitle("NIDAR AirMouse Ground Control Station (GCS) - Windows 10/11");
    resize(1440, 900);
    setStyleSheet("QMainWindow { background-color: #0b0f19; font-family: 'Segoe UI'; color: #e2e8f0; }");

    QWidget* central = new QWidget(this);
    setCentralWidget(central);
    QVBoxLayout* root_layout = new QVBoxLayout(central);
    root_layout->setContentsMargins(12, 12, 12, 12);
    root_layout->setSpacing(10);

    // 1. Top Mission Control Header
    QWidget* header = new QWidget(this);
    header->setStyleSheet("background-color: #151c2c; border: 1px solid #2d3748; border-radius: 8px; padding: 6px;");
    QHBoxLayout* header_layout = new QHBoxLayout(header);

    status_label_ = new QLabel("STATUS: SEARCHING BONJOUR...", this);
    status_label_->setStyleSheet("color: #f6ad55; font-weight: bold; font-size: 13px;");
    header_layout->addWidget(status_label_);

    ip_label_ = new QLabel("IP: DISCOVERING", this);
    ip_label_->setStyleSheet("color: #a0aec0; font-family: 'Consolas'; font-size: 13px;");
    header_layout->addWidget(ip_label_);

    header_layout->addStretch();

    state_label_ = new QLabel("FLIGHT STATE: BOOT_AND_INIT", this);
    state_label_->setStyleSheet("color: #63b3ed; font-weight: bold; font-size: 13px;");
    header_layout->addWidget(state_label_);

    timer_label_ = new QLabel("MISSION TIMER: 00:00 / 30:00", this);
    timer_label_->setStyleSheet("color: #48bb78; font-weight: bold; font-family: 'Consolas'; font-size: 14px;");
    header_layout->addWidget(timer_label_);

    battery_label_ = new QLabel("BATT: 15.8V", this);
    battery_label_->setStyleSheet("color: #68d391; font-weight: bold; font-size: 13px;");
    header_layout->addWidget(battery_label_);

    // CRITICAL HARD-CODED EMERGENCY ABORT BUTTON
    abort_button_ = new QPushButton("EMERGENCY ABORT", this);
    abort_button_->setMinimumSize(180, 42);
    abort_button_->setStyleSheet(
        "QPushButton { background-color: #e53e3e; color: white; font-weight: bold; "
        "font-size: 14px; border: 2px solid #fc8181; border-radius: 6px; } "
        "QPushButton:hover { background-color: #c53030; } "
        "QPushButton:pressed { background-color: #9b2c2c; }"
    );
    header_layout->addWidget(abort_button_);

    root_layout->addWidget(header);

    // 2. Middle Multi-Panel Splitter (Video Player + 2D Arena Canvas)
    QSplitter* main_splitter = new QSplitter(Qt::Horizontal, this);

    video_widget_ = new VideoWidget(this);
    video_widget_->setMinimumSize(540, 420);
    main_splitter->addWidget(video_widget_);

    map_canvas_ = new MapCanvas(this);
    map_canvas_->setMinimumSize(540, 420);
    main_splitter->addWidget(map_canvas_);

    main_splitter->setSizes({650, 750});
    root_layout->addWidget(main_splitter, 3);

    // 3. Bottom Survivor Telemetry Log Table
    QWidget* table_container = new QWidget(this);
    table_container->setStyleSheet("background-color: #151c2c; border: 1px solid #2d3748; border-radius: 8px;");
    QVBoxLayout* table_layout = new QVBoxLayout(table_container);
    table_layout->setContentsMargins(8, 8, 8, 8);

    QLabel* table_title = new QLabel("CONFIRMED SURVIVORS LOG (AIRMOUSE 2x2m MODULAR ROOM STANDARDS)", this);
    table_title->setStyleSheet("color: #fc8181; font-weight: bold; font-size: 12px;");
    table_layout->addWidget(table_title);

    survivor_table_ = new QTableWidget(this);
    survivor_table_->setColumnCount(6);
    survivor_table_->setHorizontalHeaderLabels({"SURVIVOR ID", "MODULAR ROOM", "COORDINATES (X, Y)", "CONFIDENCE", "DETECTION TIME", "STATUS"});
    survivor_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    survivor_table_->setStyleSheet(
        "QTableWidget { background-color: #0b0f19; gridline-color: #2d3748; color: #edf2f7; "
        "font-family: 'Consolas'; font-size: 12px; } "
        "QHeaderView::section { background-color: #1a202c; color: #a0aec0; padding: 4px; font-weight: bold; border: 1px solid #2d3748; }"
    );
    survivor_table_->setFixedHeight(160);
    table_layout->addWidget(survivor_table_);

    root_layout->addWidget(table_container, 1);
}

void MainWindow::onDroneResolved(const QString& ip_address) {
    drone_ip_ = ip_address;
    status_label_->setText("STATUS: ONLINE (5GHz ISOLATED LINK)");
    status_label_->setStyleSheet("color: #48bb78; font-weight: bold; font-size: 13px;");
    ip_label_->setText("IP: " + drone_ip_ + " (BONJOUR RESOLVED)");

    // Connect to TCP Telemetry Server (Port 9001)
    telemetry_->connectToDrone(drone_ip_, 9001);

    // Connect Video Player to live RTSP/MJPEG stream
    video_widget_->startStream("http://" + drone_ip_ + ":8080/live.mjpg");
}

void MainWindow::onTelemetryReceived(const nidar::net::TelemetryPayload& telem) {
    map_canvas_->updateDronePose(telem.pose_x, telem.pose_y, telem.pose_yaw_deg);

    // Format 30-minute mission countdown
    uint32_t mins = telem.mission_timer_sec / 60;
    uint32_t secs = telem.mission_timer_sec % 60;
    timer_label_->setText(QString("MISSION TIMER: %1:%2 / 30:00")
                          .arg(mins, 2, 10, QChar('0'))
                          .arg(secs, 2, 10, QChar('0')));

    if (mins >= 27) {
        timer_label_->setStyleSheet("color: #e53e3e; font-weight: bold; font-family: 'Consolas'; font-size: 14px;");
    }

    battery_label_->setText(QString("BATT: %1V").arg(telem.battery_volts, 0, 'f', 1));

    static const char* states[] = {
        "BOOT_AND_INIT", "TAKEOFF_AND_ENTER", "EXPLORE_MAZE",
        "TAG_SURVIVOR", "FIND_EXIT_AND_LAND", "EMERGENCY_ABORT", "MISSION_COMPLETED"
    };
    if (telem.flight_state <= 6) {
        state_label_->setText(QString("FLIGHT STATE: %1").arg(states[telem.flight_state]));
    }
}

void MainWindow::onMapReceived(int w, int h, float res, float ox, float oy, const QVector<uint8_t>& cells) {
    map_canvas_->updateMap(w, h, res, ox, oy, cells);
}

void MainWindow::onSurvivorTagged(const QJsonObject& json) {
    QString survivor_id = json["survivor_id"].toString();
    float conf = json["confidence"].toDouble();
    QJsonObject coord = json["survivor_coord"].toObject();
    float sx = coord["x"].toDouble();
    float sy = coord["y"].toDouble();
    QJsonObject room = json["grid_room"].toObject();
    QString room_id = room["room_id"].toString();

    // 1. Instantly drop marker on 2D arena canvas
    map_canvas_->dropSurvivorMarker(sx, sy, survivor_id, room_id, conf);

    // 2. Flash video player target card
    video_widget_->setDetectedSurvivorOverlay(survivor_id, conf);

    // 3. Insert into GCS survivor log table
    int row = survivor_table_->rowCount();
    survivor_table_->insertRow(row);
    survivor_table_->setItem(row, 0, new QTableWidgetItem(survivor_id));
    survivor_table_->setItem(row, 1, new QTableWidgetItem(room_id));
    survivor_table_->setItem(row, 2, new QTableWidgetItem(QString("(%1m, %2m)").arg(sx, 0, 'f', 2).arg(sy, 0, 'f', 2)));
    survivor_table_->setItem(row, 3, new QTableWidgetItem(QString("%1%").arg((int)(conf * 100))));
    survivor_table_->setItem(row, 4, new QTableWidgetItem(QDateTime::currentDateTime().toString("hh:mm:ss")));
    survivor_table_->setItem(row, 5, new QTableWidgetItem("TAGGED & VERIFIED"));

    qDebug() << "[GCS] SURVIVOR TAGGED EVENT:" << survivor_id << "in" << room_id;
}

void MainWindow::onEmergencyAbortClicked() {
    QString target = drone_ip_.isEmpty() ? "192.168.4.1" : drone_ip_;
    abort_->blastEmergencyAbort(target, 9999);

    status_label_->setText("STATUS: EMERGENCY ABORT SENT!");
    status_label_->setStyleSheet("color: #e53e3e; font-weight: bold; font-size: 13px;");

    QMessageBox::critical(this, "EMERGENCY ABORT SENT",
                          "High-Priority Emergency Abort datagrams blasted over UDP port 9999 to " + target + ".\n"
                          "Drone motors commanded to hard halt immediately.");
}
