/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include <QtCore/QFile>
#include <QtCore/QLoggingCategory>
#include <QtCore/QObject>
#include <QtCore/QString>

Q_DECLARE_LOGGING_CATEGORY(RTCMFileLoggerLog)

/// Passive tap that appends the raw RTCM3 stream to a file for later replay/analysis.
class RTCMFileLogger : public QObject
{
    Q_OBJECT

public:
    /// @param path Destination file. If empty, a timestamped file is created in the app log directory.
    explicit RTCMFileLogger(const QString &path, QObject *parent = nullptr);
    ~RTCMFileLogger() override;

    bool isOpen() const { return _file.isOpen(); }
    QString filePath() const { return _file.fileName(); }
    quint64 bytesWritten() const { return _bytesWritten; }

public slots:
    void logData(const QByteArray &data);

signals:
    void bytesWrittenChanged(quint64 totalBytes);

private:
    static QString _resolvePath(const QString &path);

    QFile _file;
    quint64 _bytesWritten = 0;
};
