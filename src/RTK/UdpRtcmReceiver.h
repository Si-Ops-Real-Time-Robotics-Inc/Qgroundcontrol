/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include <QtCore/QLoggingCategory>

#include "RTCMNetworkSource.h"

Q_DECLARE_LOGGING_CATEGORY(UdpRtcmReceiverLog)

class QUdpSocket;

/// Receives raw RTCM3 datagrams on a local UDP port.
class UdpRtcmReceiver : public RTCMNetworkSource
{
    Q_OBJECT

public:
    explicit UdpRtcmReceiver(quint16 port, QObject *parent = nullptr);
    ~UdpRtcmReceiver() override;

public slots:
    void start() override;
    void stop() override;

private slots:
    void _onReadyRead();

private:
    const quint16 _port;
    QUdpSocket *_socket = nullptr;
    quint64 _totalBytes = 0;
};
