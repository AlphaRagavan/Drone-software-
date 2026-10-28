#pragma once

#include <QObject>
#include <QString>
#include <QHostAddress>
#include <QUdpSocket>
#include <QTimer>

class BonjourResolver : public QObject {
    Q_OBJECT
public:
    explicit BonjourResolver(const QString& target_host = "nidar-drone.local", QObject* parent = nullptr);
    ~BonjourResolver();

    void startDiscovery();
    void stopDiscovery();
    QString getResolvedIp() const { return resolved_ip_; }

signals:
    void hostResolved(const QString& ip_address);
    void resolutionFailed(const QString& error_message);

private slots:
    void onResolveTimer();
    void readMulticastResponse();

private:
    void sendMdnsQuery();

    QString target_host_;
    QString resolved_ip_;
    QUdpSocket* udp_socket_{nullptr};
    QTimer* timer_{nullptr};
    int attempts_{0};
};
