#include "abort_controller.hpp"
#include <QThread>
#include <QDebug>

AbortController::AbortController(QObject* parent)
    : QObject(parent) {
    udp_socket_ = new QUdpSocket(this);
}

AbortController::~AbortController() {
    if (udp_socket_->isOpen()) {
        udp_socket_->close();
    }
}

void AbortController::blastEmergencyAbort(const QString& target_ip, uint16_t port) {
    QByteArray payload = "EMERGENCY_ABORT_NIDAR_HALT";
    QHostAddress dest(target_ip);

    qCritical() << "\n========================================================";
    qCritical() << "[GCS CRITICAL] BLASTING HARD EMERGENCY ABORT TO" << target_ip << ":" << port;
    qCritical() << "========================================================\n";

    // Send a burst of 10 consecutive UDP datagrams to eliminate any RF packet loss
    int count = 0;
    for (int i = 0; i < 10; ++i) {
        qint64 sent = udp_socket_->writeDatagram(payload, dest, port);
        if (sent > 0) count++;
        QThread::msleep(3); // 3ms separation
    }

    emit abortBlasted(count);
}
