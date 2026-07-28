/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "NtripClient.h"
#include "QGCLoggingCategory.h"

#include <QtNetwork/QTcpSocket>

QGC_LOGGING_CATEGORY(NtripClientLog, "qgc.rtk.ntripclient")

NtripClient::NtripClient(const Config &config, QObject *parent)
    : RTCMNetworkSource(parent)
    , _config(config)
{
    // qCDebug(NtripClientLog) << Q_FUNC_INFO << this;
}

NtripClient::~NtripClient()
{
    NtripClient::stop();

    // qCDebug(NtripClientLog) << Q_FUNC_INFO << this;
}

void NtripClient::start()
{
    if (_socket) {
        qCWarning(NtripClientLog) << "start() called while already active";
        return;
    }

    _headerBuffer.clear();
    _totalBytes = 0;
    _state = State::Idle;

    _socket = new QTcpSocket(this);
    _socket->setSocketOption(QAbstractSocket::LowDelayOption, 1);
    _socket->setSocketOption(QAbstractSocket::KeepAliveOption, 1);

    (void) connect(_socket, &QTcpSocket::connected, this, &NtripClient::_onSocketConnected);
    (void) connect(_socket, &QTcpSocket::readyRead, this, &NtripClient::_onSocketReadyRead);
    (void) connect(_socket, &QTcpSocket::disconnected, this, &NtripClient::_onSocketDisconnected);
    (void) connect(_socket, &QTcpSocket::errorOccurred, this, &NtripClient::_onSocketErrorOccurred);

    qCDebug(NtripClientLog) << "Connecting to" << _config.host << ":" << _config.port
                            << (_config.useNtripHandshake ? QStringLiteral("(NTRIP)") : QStringLiteral("(raw TCP)"));
    _socket->connectToHost(_config.host, _config.port);
}

void NtripClient::stop()
{
    if (!_socket) {
        return;
    }

    _socket->disconnect(this);
    _socket->abort();
    _socket->deleteLater();
    _socket = nullptr;
    _state = State::Idle;
    _headerBuffer.clear();
}

void NtripClient::_onSocketConnected()
{
    qCDebug(NtripClientLog) << "Socket connected";

    if (_config.useNtripHandshake) {
        _state = State::WaitingForHeader;
        _sendRequest();
    } else {
        // Raw TCP source: no handshake, everything received is RTCM.
        _state = State::Streaming;
        emit connectedChanged(true);
    }
}

void NtripClient::_sendRequest()
{
    QByteArray mount = _config.mountpoint.toUtf8();
    if (!mount.startsWith('/')) {
        mount.prepend('/');
    }

    QByteArray request;
    request += "GET " + mount + " HTTP/1.1\r\n";
    request += "Host: " + _config.host.toUtf8() + ":" + QByteArray::number(_config.port) + "\r\n";
    request += "Ntrip-Version: Ntrip/2.0\r\n";
    request += "User-Agent: NTRIP QGroundControl\r\n";
    if (!_config.username.isEmpty() || !_config.password.isEmpty()) {
        const QByteArray credentials = (_config.username + ":" + _config.password).toUtf8().toBase64();
        request += "Authorization: Basic " + credentials + "\r\n";
    }
    request += "Connection: close\r\n";
    request += "\r\n";

    (void) _socket->write(request);
}

void NtripClient::_onSocketReadyRead()
{
    if (_state == State::Streaming) {
        _emitRtcm(_socket->readAll());
        return;
    }

    if (_state == State::WaitingForHeader) {
        _headerBuffer += _socket->readAll();
        _processHeader();
        return;
    }

    // Idle/unexpected - drain.
    (void) _socket->readAll();
}

void NtripClient::_processHeader()
{
    // Real casters vary, so be liberal in what we accept:
    //  - NTRIP v1 (ICY, raw TCP - NOT HTTP): "ICY 200 OK\r\n" then RTCM immediately (no blank line).
    //  - NTRIP v2 (HTTP): "HTTP/1.1 200 OK\r\n<headers>\r\n\r\n" then RTCM.
    //  - Line endings may be CRLF or bare LF; HTTP header block may end with \r\n\r\n or \n\n.

    // Locate the end of the status line, accepting CRLF or a bare LF.
    int firstLineEnd = -1;
    int eolLen = 0;
    for (int i = 0; i < _headerBuffer.size(); ++i) {
        if (_headerBuffer.at(i) == '\n') {
            const bool crlf = (i > 0) && (_headerBuffer.at(i - 1) == '\r');
            firstLineEnd = crlf ? (i - 1) : i;
            eolLen = crlf ? 2 : 1;
            break;
        }
    }
    if (firstLineEnd < 0) {
        if (_headerBuffer.size() > kMaxHeaderBytes) {
            emit errorOccurred(tr("NTRIP: invalid response from caster"));
            stop();
        }
        return; // wait for the rest of the status line
    }

    const QByteArray firstLine = _headerBuffer.left(firstLineEnd).trimmed();

    if (firstLine.contains("SOURCETABLE")) {
        emit errorOccurred(tr("NTRIP: mountpoint not found (caster returned source table)"));
        stop();
        return;
    }

    if (!firstLine.contains("200")) {
        emit errorOccurred(tr("NTRIP: caster rejected request: %1").arg(QString::fromLatin1(firstLine)));
        stop();
        return;
    }

    // Where does the RTCM payload begin?
    int headerEnd = -1;
    const bool isHttp = firstLine.startsWith("HTTP");
    if (!isHttp) {
        // ICY / non-HTTP 200 (NTRIP v1): RTCM follows the status line directly.
        headerEnd = firstLineEnd + eolLen;
    } else {
        // HTTP (NTRIP v2): payload follows the end-of-headers blank line (CRLFCRLF or LFLF).
        int blank = _headerBuffer.indexOf("\r\n\r\n");
        int blankLen = 4;
        const int lflf = _headerBuffer.indexOf("\n\n");
        if ((blank < 0) || ((lflf >= 0) && (lflf < blank))) {
            blank = lflf;
            blankLen = 2;
        }
        if (blank < 0) {
            if (_headerBuffer.size() > kMaxHeaderBytes) {
                emit errorOccurred(tr("NTRIP: caster header too large"));
                stop();
            }
            return; // wait for the full header block
        }
        headerEnd = blank + blankLen;
    }

    const QByteArray remainder = _headerBuffer.mid(headerEnd);
    _headerBuffer.clear();
    _state = State::Streaming;
    emit connectedChanged(true);
    qCDebug(NtripClientLog) << "NTRIP stream established";

    if (!remainder.isEmpty()) {
        _emitRtcm(remainder);
    }
}

void NtripClient::_emitRtcm(const QByteArray &data)
{
    if (data.isEmpty()) {
        return;
    }
    _totalBytes += static_cast<quint64>(data.size());
    emit rtcmData(data);
    emit bytesReceived(_totalBytes);
}

void NtripClient::sendGGA(const QByteArray &gga)
{
    if (!_socket || (_state != State::Streaming) || gga.isEmpty()) {
        return;
    }
    (void) _socket->write(gga);
    qCDebug(NtripClientLog) << "Sent GGA:" << gga.trimmed();
}

void NtripClient::_onSocketDisconnected()
{
    qCDebug(NtripClientLog) << "Socket disconnected";
    _state = State::Idle;
    emit connectedChanged(false);
}

void NtripClient::_onSocketErrorOccurred(QAbstractSocket::SocketError error)
{
    Q_UNUSED(error);
    if (_socket) {
        const QString msg = _socket->errorString();
        qCWarning(NtripClientLog) << "Socket error:" << msg;
        emit errorOccurred(msg);
    }
}
