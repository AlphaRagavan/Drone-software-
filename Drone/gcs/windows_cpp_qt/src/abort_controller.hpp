#pragma once

#include <QObject>
#include <QString>
#include <QUdpSocket>
#include <QHostAddress>

class AbortController : public QObject {
    Q_OBJECT
public:
    explicit AbortController(QObject* parent = nullptr);
    ~AbortController();

    void blastEmergencyAbort(const QString& target_ip, uint16_t port = 9999);

signals:
    void abortBlasted(int packets_sent);

private:
    QUdpSocket* udp_socket_{nullptr};
};
