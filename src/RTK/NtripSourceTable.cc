/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "NtripSourceTable.h"
#include "QGCLoggingCategory.h"

#include <QtCore/QTimer>
#include <QtNetwork/QTcpSocket>

QGC_LOGGING_CATEGORY(NtripSourceTableLog, "qgc.rtk.ntripsourcetable")

NtripSourceTable::NtripSourceTable(const QString &host, quint16 port, const QString &username, const QString &password, QObject *parent)
    : QObject(parent)
    , _host(host)
    , _port(port)
    , _username(username)
    , _password(password)
{
}

NtripSourceTable::~NtripSourceTable()
{
    if (_socket) {
        _socket->abort();
    }
}

void NtripSourceTable::start()
{
    _socket = new QTcpSocket(this);
    (void) connect(_socket, &QTcpSocket::connected, this, &NtripSourceTable::_onConnected);
    (void) connect(_socket, &QTcpSocket::readyRead, this, &NtripSourceTable::_onReadyRead);
    (void) connect(_socket, &QTcpSocket::disconnected, this, &NtripSourceTable::_onDisconnected);
    (void) connect(_socket, &QTcpSocket::errorOccurred, this, &NtripSourceTable::_onErrorOccurred);

    _timeoutTimer = new QTimer(this);
    _timeoutTimer->setSingleShot(true);
    (void) connect(_timeoutTimer, &QTimer::timeout, this, &NtripSourceTable::_onTimeout);
    _timeoutTimer->start(kTimeoutMs);

    qCDebug(NtripSourceTableLog) << "Fetching source table from" << _host << ":" << _port;
    _socket->connectToHost(_host, _port);
}

void NtripSourceTable::_onConnected()
{
    QByteArray request;
    request += "GET / HTTP/1.0\r\n";
    request += "User-Agent: NTRIP QGroundControl\r\n";
    if (!_username.isEmpty() || !_password.isEmpty()) {
        const QByteArray credentials = (_username + ":" + _password).toUtf8().toBase64();
        request += "Authorization: Basic " + credentials + "\r\n";
    }
    request += "Connection: close\r\n";
    request += "\r\n";
    (void) _socket->write(request);
}

void NtripSourceTable::_onReadyRead()
{
    _buffer += _socket->readAll();

    if (_buffer.size() > kMaxBytes) {
        _parseAndFinish();
        return;
    }

    // The full table ends with ENDSOURCETABLE; finish as soon as we see it.
    if (_buffer.contains("ENDSOURCETABLE")) {
        _parseAndFinish();
    }
}

void NtripSourceTable::_onDisconnected()
{
    // Caster closes the connection (HTTP/1.0) once the table is sent.
    _parseAndFinish();
}

void NtripSourceTable::_onErrorOccurred(QAbstractSocket::SocketError error)
{
    Q_UNUSED(error);
    if (_done) {
        return;
    }
    // A remote close after a complete table shows up as an error on some stacks; try to parse first.
    if (_buffer.contains("ENDSOURCETABLE") || _buffer.contains("STR;")) {
        _parseAndFinish();
        return;
    }
    _done = true;
    const QString msg = _socket ? _socket->errorString() : QStringLiteral("connection error");
    qCWarning(NtripSourceTableLog) << "Source table fetch error:" << msg;
    emit failed(msg);
}

void NtripSourceTable::_onTimeout()
{
    if (_done) {
        return;
    }
    if (!_buffer.isEmpty()) {
        _parseAndFinish();
        return;
    }
    _done = true;
    emit failed(tr("Timed out fetching mountpoint list"));
}

void NtripSourceTable::_parseAndFinish()
{
    if (_done) {
        return;
    }
    _done = true;
    if (_timeoutTimer) {
        _timeoutTimer->stop();
    }

    const QByteArray firstLine = _buffer.left(qMax(0, _buffer.indexOf("\r\n")));
    if (firstLine.contains("401")) {
        emit failed(tr("Authentication failed (401)"));
        return;
    }

    QStringList mountpoints;
    const QList<QByteArray> lines = _buffer.split('\n');
    for (const QByteArray &raw : lines) {
        const QByteArray line = raw.trimmed();
        if (!line.startsWith("STR;")) {
            continue;
        }
        // STR;<mountpoint>;<identifier>;<format>;...
        const QList<QByteArray> fields = line.split(';');
        if (fields.size() >= 2) {
            const QString mount = QString::fromUtf8(fields.at(1)).trimmed();
            if (!mount.isEmpty() && !mountpoints.contains(mount)) {
                mountpoints.append(mount);
            }
        }
    }

    qCDebug(NtripSourceTableLog) << "Parsed" << mountpoints.size() << "mountpoints";
    if (mountpoints.isEmpty()) {
        emit failed(tr("No mountpoints found in caster response"));
    } else {
        mountpoints.sort(Qt::CaseInsensitive);
        emit finished(mountpoints);
    }
}
