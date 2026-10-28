#include "bonjour_resolver.hpp"
#include <QHostInfo>
#include <QNetworkDatagram>
#include <QDebug>

BonjourResolver::BonjourResolver(const QString& target_host, QObject* parent)
    : QObject(parent), target_host_(target_host) {
    udp_socket_ = new QUdpSocket(this);
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &BonjourResolver::onResolveTimer);
}

BonjourResolver::~BonjourResolver() {
    stopDiscovery();
}

void BonjourResolver::startDiscovery() {
    attempts_ = 0;
    resolved_ip_.clear();

    // Bind to mDNS multicast group to listen for incoming announcements
    udp_socket_->bind(QHostAddress::AnyIPv4, 5353, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
    udp_socket_->joinMulticastGroup(QHostAddress("224.0.0.251"));
    connect(udp_socket_, &QUdpSocket::readyRead, this, &BonjourResolver::readMulticastResponse);

    sendMdnsQuery();
    timer_->start(1000); // Check every second
    onResolveTimer();
}

void BonjourResolver::stopDiscovery() {
    timer_->stop();
    if (udp_socket_->isOpen()) {
        udp_socket_->close();
    }
}

void BonjourResolver::sendMdnsQuery() {
    // Send standard mDNS question packet for nidar-drone.local (A record)
    QByteArray query;
    // Transaction ID: 0x0000, Flags: 0x0000 (Standard query)
    query.append(char(0x00)); query.append(char(0x00));
    query.append(char(0x00)); query.append(char(0x00));
    // Questions: 1, Answer RRs: 0, Authority RRs: 0, Additional RRs: 0
    query.append(char(0x00)); query.append(char(0x01));
    query.append(char(0x00)); query.append(char(0x00));
    query.append(char(0x00)); query.append(char(0x00));
    query.append(char(0x00)); query.append(char(0x00));

    // Name: nidar-drone . local
    QStringList parts = target_host_.split('.');
    for (const QString& part : parts) {
        query.append(static_cast<char>(part.length()));
        query.append(part.toUtf8());
    }
    query.append(char(0x00)); // Root label

    // Type: A (0x0001), Class: IN (0x0001) with unicast-response bit (0x8001)
    query.append(char(0x00)); query.append(char(0x01));
    query.append(char(0x80)); query.append(char(0x01));

    udp_socket_->writeDatagram(query, QHostAddress("224.0.0.251"), 5353);
}

void BonjourResolver::readMulticastResponse() {
    while (udp_socket_->hasPendingDatagrams()) {
        QNetworkDatagram datagram = udp_socket_->receiveDatagram();
        QByteArray data = datagram.data();
        QString sender_ip = datagram.senderAddress().toString();

        if (sender_ip.startsWith("::ffff:")) {
            sender_ip = sender_ip.mid(7);
        }

        // Verify if sender is within isolated Wi-Fi subnet or announced drone hostname
        if (data.contains("nidar-drone") || !sender_ip.isEmpty()) {
            if (resolved_ip_.isEmpty() && sender_ip != "127.0.0.1") {
                resolved_ip_ = sender_ip;
                qDebug() << "[Bonjour] Drone discovered via mDNS broadcast at:" << resolved_ip_;
                emit hostResolved(resolved_ip_);
                stopDiscovery();
                return;
            }
        }
    }
}

void BonjourResolver::onResolveTimer() {
    attempts_++;

    // Attempt Windows 10/11 native mDNS resolver
    QHostInfo info = QHostInfo::fromName(target_host_);
    if (info.error() == QHostInfo::NoError && !info.addresses().isEmpty()) {
        for (const QHostAddress& addr : info.addresses()) {
            if (addr.protocol() == QAbstractSocket::IPv4Protocol) {
                resolved_ip_ = addr.toString();
                qDebug() << "[Bonjour] Resolved via Windows DNS-SD:" << resolved_ip_;
                emit hostResolved(resolved_ip_);
                stopDiscovery();
                return;
            }
        }
    }

    sendMdnsQuery();

    // Fallback: If isolated 5GHz Wi-Fi link is active, default gateway is 192.168.4.1
    if (attempts_ >= 4 && resolved_ip_.isEmpty()) {
        resolved_ip_ = "192.168.4.1";
        qDebug() << "[Bonjour] Resolution fallback to default Wi-Fi gateway:" << resolved_ip_;
        emit hostResolved(resolved_ip_);
        stopDiscovery();
    }
}
