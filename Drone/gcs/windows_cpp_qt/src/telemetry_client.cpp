#include "telemetry_client.hpp"
#include <QDebug>

TelemetryClient::TelemetryClient(QObject* parent)
    : QObject(parent) {
    socket_ = new QTcpSocket(this);
    connect(socket_, &QTcpSocket::connected, this, &TelemetryClient::onSocketConnected);
    connect(socket_, &QTcpSocket::disconnected, this, &TelemetryClient::onSocketDisconnected);
    connect(socket_, &QTcpSocket::readyRead, this, &TelemetryClient::onReadyRead);
}

TelemetryClient::~TelemetryClient() {
    disconnectFromDrone();
}

void TelemetryClient::connectToDrone(const QString& host_ip, uint16_t port) {
    if (socket_->state() == QAbstractSocket::ConnectedState) {
        socket_->disconnectFromHost();
    }
    qDebug() << "[Telemetry Client] Connecting to drone at" << host_ip << ":" << port;
    socket_->connectToHost(host_ip, port);
}

void TelemetryClient::disconnectFromDrone() {
    if (socket_->isOpen()) {
        socket_->disconnectFromHost();
    }
}

bool TelemetryClient::isConnected() const {
    return socket_->state() == QAbstractSocket::ConnectedState;
}

void TelemetryClient::onSocketConnected() {
    qDebug() << "[Telemetry Client] Successfully connected to drone telemetry link.";
    emit connected();
}

void TelemetryClient::onSocketDisconnected() {
    qDebug() << "[Telemetry Client] Disconnected from drone telemetry link.";
    emit disconnected();
}

void TelemetryClient::onReadyRead() {
    receive_buffer_.append(socket_->readAll());
    processBuffer();
}

QVector<uint8_t> TelemetryClient::decompressRLE(const uint8_t* data, size_t len, size_t expected_size) {
    QVector<uint8_t> out;
    out.reserve(expected_size);

    for (size_t i = 0; i + 1 < len; i += 2) {
        uint8_t count = data[i];
        uint8_t val = data[i + 1];
        for (uint8_t c = 0; c < count; ++c) {
            out.append(val);
        }
    }
    return out;
}

void TelemetryClient::processBuffer() {
    constexpr size_t HEADER_SIZE = sizeof(nidar::net::PacketHeader);

    while (receive_buffer_.size() >= static_cast<int>(HEADER_SIZE)) {
        const uint8_t* ptr = reinterpret_cast<const uint8_t*>(receive_buffer_.constData());
        const auto* hdr = reinterpret_cast<const nidar::net::PacketHeader*>(ptr);

        // Check magic
        if (hdr->magic != nidar::net::NIDAR_PACKET_MAGIC) {
            // Desynchronized stream: drop 1 byte to re-align
            receive_buffer_.remove(0, 1);
            continue;
        }

        size_t total_pkt_len = HEADER_SIZE + hdr->payload_len;
        if (receive_buffer_.size() < static_cast<int>(total_pkt_len)) {
            // Awaiting full payload
            break;
        }

        const uint8_t* payload_ptr = ptr + HEADER_SIZE;

        switch (hdr->type) {
            case nidar::net::PKT_TELEMETRY: {
                if (hdr->payload_len == sizeof(nidar::net::TelemetryPayload)) {
                    const auto* telem = reinterpret_cast<const nidar::net::TelemetryPayload*>(payload_ptr);
                    emit telemetryReceived(*telem);
                }
                break;
            }

            case nidar::net::PKT_OCCUPANCY_GRID: {
                if (hdr->payload_len > sizeof(nidar::net::GridMapHeader)) {
                    const auto* map_hdr = reinterpret_cast<const nidar::net::GridMapHeader*>(payload_ptr);
                    const uint8_t* rle_data = payload_ptr + sizeof(nidar::net::GridMapHeader);
                    size_t expected_cells = static_cast<size_t>(map_hdr->width_cells) * map_hdr->height_cells;
                    QVector<uint8_t> cells = decompressRLE(rle_data, map_hdr->compressed_len, expected_cells);
                    emit mapReceived(map_hdr->width_cells, map_hdr->height_cells,
                                     map_hdr->resolution_m, map_hdr->origin_x_m, map_hdr->origin_y_m,
                                     cells);
                }
                break;
            }

            case nidar::net::PKT_SURVIVOR_TAG: {
                QByteArray json_bytes(reinterpret_cast<const char*>(payload_ptr), hdr->payload_len);
                QJsonDocument doc = QJsonDocument::fromJson(json_bytes);
                if (doc.isObject()) {
                    emit survivorTagged(doc.object());
                }
                break;
            }

            case nidar::net::PKT_MISSION_STATE: {
                QString msg = QString::fromUtf8(reinterpret_cast<const char*>(payload_ptr), hdr->payload_len);
                emit stateMessageReceived(msg);
                break;
            }

            default:
                break;
        }

        // Consume parsed packet
        receive_buffer_.remove(0, static_cast<int>(total_pkt_len));
    }
}
