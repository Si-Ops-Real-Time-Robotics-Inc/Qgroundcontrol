/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "RTCMFileLogger.h"
#include "AppSettings.h"
#include "QGCLoggingCategory.h"
#include "SettingsManager.h"

#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QStandardPaths>

QGC_LOGGING_CATEGORY(RTCMFileLoggerLog, "qgc.rtk.rtcmfilelogger")

RTCMFileLogger::RTCMFileLogger(const QString &path, QObject *parent)
    : QObject(parent)
    , _file(_resolvePath(path))
{
    if (!_file.open(QIODevice::WriteOnly | QIODevice::Append)) {
        qCWarning(RTCMFileLoggerLog) << "Failed to open RTCM log file" << _file.fileName() << ":" << _file.errorString();
    } else {
        qCDebug(RTCMFileLoggerLog) << "Logging RTCM to" << _file.fileName();
    }
}

RTCMFileLogger::~RTCMFileLogger()
{
    if (_file.isOpen()) {
        _file.close();
    }
}

QString RTCMFileLogger::_resolvePath(const QString &path)
{
    if (!path.isEmpty()) {
        return path;
    }

    // Default to QGC's user-visible save path (a browsable/shared folder on Android too),
    // falling back to app-data / temp if it isn't configured.
    QString dirPath;
    if (AppSettings *const appSettings = SettingsManager::instance()->appSettings()) {
        dirPath = appSettings->savePath()->rawValue().toString();
    }
    if (dirPath.isEmpty() || !QDir(dirPath).exists()) {
        dirPath = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    }
    if (dirPath.isEmpty()) {
        dirPath = QDir::tempPath();
    }

    QDir dir(QDir(dirPath).filePath(QStringLiteral("RTCMLogs")));
    if (!dir.exists()) {
        (void) dir.mkpath(QStringLiteral("."));
    }

    const QString timestamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd_hh-mm-ss"));
    return dir.filePath(QStringLiteral("rtcm_%1.rtcm3").arg(timestamp));
}

void RTCMFileLogger::logData(const QByteArray &data)
{
    if (!_file.isOpen() || data.isEmpty()) {
        return;
    }
    (void) _file.write(data);
    // Flush every write so an app crash loses at most the last chunk (RTCM rate is low).
    (void) _file.flush();
    _bytesWritten += static_cast<quint64>(data.size());
    emit bytesWrittenChanged(_bytesWritten);
}
