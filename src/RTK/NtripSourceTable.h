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
#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtNetwork/QAbstractSocket>

Q_DECLARE_LOGGING_CATEGORY(NtripSourceTableLog)

class QTcpSocket;
class QTimer;

/// One-shot fetch of an NTRIP caster's source table (GET / -> SOURCETABLE), parsing
/// the STR entries into a list of mountpoint names.
class NtripSourceTable : public QObject
{
    Q_OBJECT

public:
    NtripSourceTable(const QString &host, quint16 port, const QString &username, const QString &password, QObject *parent = nullptr);
    ~NtripSourceTable() override;

    void start();

signals:
    void finished(const QStringList &mountpoints);
    void failed(const QString &errorString);

private slots:
    void _onConnected();
    void _onReadyRead();
    void _onDisconnected();
    void _onErrorOccurred(QAbstractSocket::SocketError error);
    void _onTimeout();

private:
    void _parseAndFinish();

    const QString _host;
    const quint16 _port;
    const QString _username;
    const QString _password;

    QTcpSocket *_socket = nullptr;
    QTimer *_timeoutTimer = nullptr;
    QByteArray _buffer;
    bool _done = false;

    static constexpr int kTimeoutMs = 10000;
    static constexpr int kMaxBytes = 1 << 20; // 1 MB cap on a source table
};
