#pragma once

#include <QObject>
#include <QString>
#include <QTcpSocket>
#include <QByteArray>
#include <QVector>
#include <QJsonObject>
#include <QJsonDocument>
#include "networking/protocol_types.hpp"

class TelemetryClient : public QObject {
    Q_OBJECT
public:
    explicit TelemetryClient(QObject* parent = nullptr);
    ~TelemetryClient();

    void connectToDrone(const QString& host_ip, uint16_t port = 9001);
    void disconnectFromDrone();
    bool isConnected() const;

signals:
    void connected();
    void disconnected();
    void telemetryReceived(const nidar::net::TelemetryPayload& telemetry);
    void mapReceived(int width, int height, float resolution, float origin_x, float origin_y, const QVector<uint8_t>& raw_cells);
    void survivorTagged(const QJsonObject& survivor_json);
    void stateMessageReceived(const QString& message);

private slots:
    void onReadyRead();
    void onSocketConnected();
    void onSocketDisconnected();

private:
    void processBuffer();
    QVector<uint8_t> decompressRLE(const uint8_t* data, size_t len, size_t expected_size);

    QTcpSocket* socket_{nullptr};
    QByteArray receive_buffer_;
};
