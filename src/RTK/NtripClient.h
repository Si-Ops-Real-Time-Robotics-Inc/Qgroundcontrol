/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include <QtCore/QByteArray>
#include <QtCore/QLoggingCategory>
#include <QtCore/QString>
#include <QtNetwork/QAbstractSocket>

#include "RTCMNetworkSource.h"

Q_DECLARE_LOGGING_CATEGORY(NtripClientLog)

class QTcpSocket;

/// NTRIP v1/v2 client that streams raw RTCM3 from a caster. With useNtripHandshake
/// disabled it also serves the plain "raw RTCM over TCP" case (no HTTP handshake).
class NtripClient : public RTCMNetworkSource
{
    Q_OBJECT

public:
    struct Config {
        QString host;
        quint16 port = 2101;
        QString mountpoint;
        QString username;
        QString password;
        bool    useNtripHandshake = true;   ///< false => raw TCP source (skip handshake)
    };

    explicit NtripClient(const Config &config, QObject *parent = nullptr);
    ~NtripClient() override;

public slots:
    void start() override;
    void stop() override;
    /// Send an NMEA GGA sentence upstream (VRS / network RTK). No-op if not streaming.
    void sendGGA(const QByteArray &gga);

private slots:
    void _onSocketConnected();
    void _onSocketReadyRead();
    void _onSocketDisconnected();
    void _onSocketErrorOccurred(QAbstractSocket::SocketError error);

private:
    void _sendRequest();
    void _processHeader();
    void _emitRtcm(const QByteArray &data);

    enum class State {
        Idle,
        WaitingForHeader,
        Streaming
    };

    const Config _config;
    QTcpSocket *_socket = nullptr;
    QByteArray _headerBuffer;
    State _state = State::Idle;
    quint64 _totalBytes = 0;

    static constexpr int kMaxHeaderBytes = 8192;
    static constexpr int kConnectTimeoutMs = 5000;
};
