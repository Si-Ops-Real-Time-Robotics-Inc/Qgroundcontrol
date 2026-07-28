/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "UdpRtcmReceiver.h"
#include "QGCLoggingCategory.h"

#include <QtNetwork/QUdpSocket>

QGC_LOGGING_CATEGORY(UdpRtcmReceiverLog, "qgc.rtk.udprtcmreceiver")

UdpRtcmReceiver::UdpRtcmReceiver(quint16 port, QObject *parent)
    : RTCMNetworkSource(parent)
    , _port(port)
{
    // qCDebug(UdpRtcmReceiverLog) << Q_FUNC_INFO << this;
}

UdpRtcmReceiver::~UdpRtcmReceiver()
{
    UdpRtcmReceiver::stop();

    // qCDebug(UdpRtcmReceiverLog) << Q_FUNC_INFO << this;
}

void UdpRtcmReceiver::start()
{
    if (_socket) {
        qCWarning(UdpRtcmReceiverLog) << "start() called while already active";
        return;
    }

    _totalBytes = 0;
    _socket = new QUdpSocket(this);
    (void) connect(_socket, &QUdpSocket::readyRead, this, &UdpRtcmReceiver::_onReadyRead);

    if (!_socket->bind(QHostAddress::AnyIPv4, _port, QUdpSocket::ShareAddress)) {
        const QString msg = tr("UDP: failed to bind port %1: %2").arg(_port).arg(_socket->errorString());
        qCWarning(UdpRtcmReceiverLog) << msg;
        emit errorOccurred(msg);
        stop();
        return;
    }

    qCDebug(UdpRtcmReceiverLog) << "Listening for RTCM on UDP port" << _port;
    emit connectedChanged(true);
}

void UdpRtcmReceiver::stop()
{
    if (!_socket) {
        return;
    }

    _socket->disconnect(this);
    _socket->close();
    _socket->deleteLater();
    _socket = nullptr;
}

void UdpRtcmReceiver::_onReadyRead()
{
    while (_socket && _socket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(static_cast<int>(_socket->pendingDatagramSize()));
        const qint64 read = _socket->readDatagram(datagram.data(), datagram.size());
        if (read <= 0) {
            continue;
        }
        datagram.truncate(static_cast<int>(read));
        _totalBytes += static_cast<quint64>(read);
        emit rtcmData(datagram);
        emit bytesReceived(_totalBytes);
    }
}
