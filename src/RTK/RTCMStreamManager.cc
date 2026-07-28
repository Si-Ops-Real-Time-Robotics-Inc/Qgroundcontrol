/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "RTCMStreamManager.h"
#include "NetworkRTCMFactGroup.h"
#include "NtripClient.h"
#include "NtripSourceTable.h"
#include "QGCLoggingCategory.h"
#include "RTCMFileLogger.h"
#include "RTCMMavlink.h"
#include "RtcmBaseParser.h"
#include "RTKSettings.h"
#include "SettingsManager.h"
#include "UdpRtcmReceiver.h"
#include "MultiVehicleManager.h"
#include "Vehicle.h"

#include <QtCore/qapplicationstatic.h>
#include <QtCore/QDateTime>
#include <QtCore/QFileInfo>
#include <QtCore/QThread>
#include <QtPositioning/QGeoCoordinate>

#include <cmath>

QGC_LOGGING_CATEGORY(RTCMStreamManagerLog, "qgc.rtk.rtcmstreammanager")

Q_APPLICATION_STATIC(RTCMStreamManager, _rtcmStreamManager);

namespace {
    enum RtcmSourceType {
        SourceNone   = 0,
        SourceSerial = 1,
        SourceNtrip  = 2,
        SourceTcp    = 3,
        SourceUdp    = 4
    };
}

RTCMStreamManager::RTCMStreamManager(QObject *parent)
    : QObject(parent)
    , _factGroup(new NetworkRTCMFactGroup(this))
{
    _ggaTimer.setInterval(kGGAIntervalMs);
    (void) connect(&_ggaTimer, &QTimer::timeout, this, &RTCMStreamManager::_sendGGATick);

    RTKSettings *const rtkSettings = SettingsManager::instance()->rtkSettings();
    (void) connect(rtkSettings->rtcmSourceType(), &Fact::rawValueChanged, this, &RTCMStreamManager::_onSourceTypeSettingChanged);
    (void) connect(rtkSettings->logRtcmToFile(), &Fact::rawValueChanged, this, &RTCMStreamManager::_onLogSettingChanged);
}

RTCMStreamManager::~RTCMStreamManager()
{
    stopStream();
}

RTCMStreamManager *RTCMStreamManager::instance()
{
    return _rtcmStreamManager();
}

FactGroup *RTCMStreamManager::rtcmFactGroup()
{
    return _factGroup;
}

void RTCMStreamManager::startStream()
{
    stopStream();

    RTKSettings *const rtkSettings = SettingsManager::instance()->rtkSettings();
    const int sourceType = rtkSettings->rtcmSourceType()->rawValue().toInt();

    QString sourceLabel;
    QString mountLabel;

    switch (sourceType) {
    case SourceNtrip: {
        NtripClient::Config config;
        config.host = rtkSettings->ntripHost()->rawValue().toString();
        config.port = static_cast<quint16>(rtkSettings->ntripPort()->rawValue().toUInt());
        config.mountpoint = rtkSettings->ntripMountpoint()->rawValue().toString();
        config.username = rtkSettings->ntripUsername()->rawValue().toString();
        config.password = rtkSettings->ntripPassword()->rawValue().toString();
        config.useNtripHandshake = true;
        if (config.host.isEmpty()) {
            qCWarning(RTCMStreamManagerLog) << "NTRIP host is empty";
            return;
        }
        _worker = new NtripClient(config);
        sourceLabel = QStringLiteral("NTRIP");
        mountLabel = QStringLiteral("%1:%2/%3").arg(config.host).arg(config.port).arg(config.mountpoint);
        break;
    }
    case SourceTcp: {
        NtripClient::Config config;
        config.host = rtkSettings->tcpHost()->rawValue().toString();
        config.port = static_cast<quint16>(rtkSettings->tcpPort()->rawValue().toUInt());
        config.useNtripHandshake = false;
        if (config.host.isEmpty()) {
            qCWarning(RTCMStreamManagerLog) << "TCP host is empty";
            return;
        }
        _worker = new NtripClient(config);
        sourceLabel = QStringLiteral("TCP");
        mountLabel = QStringLiteral("%1:%2").arg(config.host).arg(config.port);
        break;
    }
    case SourceUdp: {
        const quint16 port = static_cast<quint16>(rtkSettings->udpPort()->rawValue().toUInt());
        _worker = new UdpRtcmReceiver(port);
        sourceLabel = QStringLiteral("UDP");
        mountLabel = QStringLiteral(":%1").arg(port);
        break;
    }
    case SourceNone:
    case SourceSerial:
    default:
        // Serial is handled by the existing GPSRtk autoconnect path; None does nothing.
        qCDebug(RTCMStreamManagerLog) << "No network RTCM source selected (type" << sourceType << ")";
        return;
    }

    // Worker runs on its own thread (mirrors GPSRtk / TCPLink).
    _workerThread = new QThread(this);
    _workerThread->setObjectName(QStringLiteral("RTCM_%1").arg(sourceLabel));
    _worker->moveToThread(_workerThread);
    (void) connect(_workerThread, &QThread::started, _worker, &RTCMNetworkSource::start);
    (void) connect(_workerThread, &QThread::finished, _worker, &QObject::deleteLater);

    // RTCMMavlink lives on the main thread; the cross-thread signal marshals a copy.
    _rtcmMavlink = new RTCMMavlink(this);
    (void) connect(_worker, &RTCMNetworkSource::rtcmData, _rtcmMavlink, &RTCMMavlink::RTCMDataUpdate, Qt::QueuedConnection);

    if (rtkSettings->logRtcmToFile()->rawValue().toBool()) {
        _startFileLogging();
    }

    // Passive tap to decode the base station position (RTCM 1005/1006) for map display.
    _baseParser = new RtcmBaseParser(this);
    (void) connect(_worker, &RTCMNetworkSource::rtcmData, _baseParser, &RtcmBaseParser::addData, Qt::QueuedConnection);
    (void) connect(_baseParser, &RtcmBaseParser::basePositionUpdate, this, &RTCMStreamManager::_onBasePositionUpdate, Qt::QueuedConnection);

    (void) connect(_worker, &RTCMNetworkSource::connectedChanged, this, &RTCMStreamManager::_onSourceConnectedChanged, Qt::QueuedConnection);
    (void) connect(_worker, &RTCMNetworkSource::errorOccurred, this, &RTCMStreamManager::_onSourceError, Qt::QueuedConnection);
    (void) connect(_worker, &RTCMNetworkSource::bytesReceived, this, &RTCMStreamManager::_onSourceBytesReceived, Qt::QueuedConnection);

    _lastByteCount = 0;
    _rateTimer.start();

    _factGroup->sourceType()->setRawValue(sourceLabel);
    _factGroup->mountpoint()->setRawValue(mountLabel);
    _factGroup->bytesPerSecond()->setRawValue(0);
    _factGroup->connected()->setRawValue(false);

    _workerThread->start();

    if ((sourceType == SourceNtrip) && rtkSettings->ntripSendGGA()->rawValue().toBool()) {
        _ggaTimer.start();
    }

    qCDebug(RTCMStreamManagerLog) << "Started RTCM stream:" << sourceLabel << mountLabel;
    emit activeChanged();
}

void RTCMStreamManager::stopStream()
{
    _ggaTimer.stop();
    _lastGGACoordinate = QGeoCoordinate();

    if (_workerThread) {
        if (_worker) {
            (void) QMetaObject::invokeMethod(_worker, "stop", Qt::QueuedConnection);
        }
        _workerThread->quit();
        if (!_workerThread->wait(kThreadDisconnectTimeout)) {
            qCWarning(RTCMStreamManagerLog) << "Timed out waiting for RTCM worker thread to exit";
        }
        // _worker is deleted via the QThread::finished -> deleteLater connection.
        _worker = nullptr;
        _workerThread->deleteLater();
        _workerThread = nullptr;
    }

    if (_rtcmMavlink) {
        _rtcmMavlink->deleteLater();
        _rtcmMavlink = nullptr;
    }

    _stopFileLogging();

    if (_baseParser) {
        _baseParser->deleteLater();
        _baseParser = nullptr;
    }

    if (_factGroup) {
        _factGroup->connected()->setRawValue(false);
        _factGroup->sourceType()->setRawValue(QStringLiteral("None"));
        _factGroup->mountpoint()->setRawValue(QString());
        _factGroup->bytesPerSecond()->setRawValue(0);
        _factGroup->baseValid()->setRawValue(false);
    }

    emit activeChanged();
}

void RTCMStreamManager::_onSourceConnectedChanged(bool connected)
{
    qCDebug(RTCMStreamManagerLog) << "Source connected:" << connected;
    if (_factGroup) {
        _factGroup->connected()->setRawValue(connected);
    }
}

void RTCMStreamManager::_onSourceError(const QString &errorString)
{
    qCWarning(RTCMStreamManagerLog) << "Source error:" << errorString;
    if (_factGroup) {
        _factGroup->connected()->setRawValue(false);
    }
}

void RTCMStreamManager::_onSourceBytesReceived(quint64 totalBytes)
{
    if (!_rateTimer.isValid() || !_factGroup) {
        return;
    }

    const qint64 elapsed = _rateTimer.elapsed();
    if (elapsed >= 1000) {
        const double rate = (static_cast<double>(totalBytes - _lastByteCount) * 1000.0) / static_cast<double>(elapsed);
        _factGroup->bytesPerSecond()->setRawValue(rate);
        _lastByteCount = totalBytes;
        (void) _rateTimer.restart();
    }
}

void RTCMStreamManager::fetchMountpoints()
{
    if (_fetchingMountpoints) {
        return;
    }

    RTKSettings *const rtkSettings = SettingsManager::instance()->rtkSettings();
    const QString host = rtkSettings->ntripHost()->rawValue().toString();
    const quint16 port = static_cast<quint16>(rtkSettings->ntripPort()->rawValue().toUInt());
    const QString user = rtkSettings->ntripUsername()->rawValue().toString();
    const QString pass = rtkSettings->ntripPassword()->rawValue().toString();

    if (host.isEmpty()) {
        _mountpointsError = tr("Enter the caster host first");
        emit mountpointsErrorChanged();
        return;
    }

    if (_sourceTable) {
        _sourceTable->deleteLater();
        _sourceTable = nullptr;
    }

    _mountpointsError.clear();
    emit mountpointsErrorChanged();
    _fetchingMountpoints = true;
    emit fetchingMountpointsChanged();

    _sourceTable = new NtripSourceTable(host, port, user, pass, this);
    (void) connect(_sourceTable, &NtripSourceTable::finished, this, [this](const QStringList &list) {
        _mountpoints = list;
        emit mountpointsChanged();
        _fetchingMountpoints = false;
        emit fetchingMountpointsChanged();
        _sourceTable->deleteLater();
        _sourceTable = nullptr;
    });
    (void) connect(_sourceTable, &NtripSourceTable::failed, this, [this](const QString &error) {
        _mountpointsError = error;
        emit mountpointsErrorChanged();
        _fetchingMountpoints = false;
        emit fetchingMountpointsChanged();
        _sourceTable->deleteLater();
        _sourceTable = nullptr;
    });
    _sourceTable->start();
}

void RTCMStreamManager::_onBasePositionUpdate(double latitude, double longitude, double altitude, int stationId)
{
    if (!_factGroup) {
        return;
    }
    _factGroup->baseLatitude()->setRawValue(latitude);
    _factGroup->baseLongitude()->setRawValue(longitude);
    _factGroup->baseAltitude()->setRawValue(altitude);
    _factGroup->baseStationId()->setRawValue(stationId);
    _factGroup->baseValid()->setRawValue(true);
}

void RTCMStreamManager::_onLogSettingChanged()
{
    // Allow the user to start/stop file logging live while a stream is active.
    if (!active() || !_worker) {
        return;
    }

    const bool enable = SettingsManager::instance()->rtkSettings()->logRtcmToFile()->rawValue().toBool();
    if (enable) {
        _startFileLogging();
    } else {
        _stopFileLogging();
    }
}

void RTCMStreamManager::_startFileLogging()
{
    if (_fileLogger || !_worker) {
        return;
    }

    RTKSettings *const rtkSettings = SettingsManager::instance()->rtkSettings();
    _fileLogger = new RTCMFileLogger(rtkSettings->rtcmLogPath()->rawValue().toString(), this);
    (void) connect(_worker, &RTCMNetworkSource::rtcmData, _fileLogger, &RTCMFileLogger::logData, Qt::QueuedConnection);
    (void) connect(_fileLogger, &RTCMFileLogger::bytesWrittenChanged, this, &RTCMStreamManager::_onLogBytesWritten, Qt::QueuedConnection);

    if (_factGroup) {
        _factGroup->logFileName()->setRawValue(QFileInfo(_fileLogger->filePath()).fileName());
        _factGroup->logBytes()->setRawValue(0);
    }
    qCDebug(RTCMStreamManagerLog) << "Started RTCM file logging ->" << _fileLogger->filePath();
}

void RTCMStreamManager::_stopFileLogging()
{
    if (_fileLogger) {
        _fileLogger->deleteLater();
        _fileLogger = nullptr;
        qCDebug(RTCMStreamManagerLog) << "Stopped RTCM file logging";
    }
    if (_factGroup) {
        _factGroup->logFileName()->setRawValue(QString());
        _factGroup->logBytes()->setRawValue(0);
    }
}

void RTCMStreamManager::_onLogBytesWritten(quint64 totalBytes)
{
    if (_factGroup) {
        _factGroup->logBytes()->setRawValue(static_cast<double>(totalBytes));
    }
}

void RTCMStreamManager::_onSourceTypeSettingChanged()
{
    // If the user switches source type while a stream is active, tear it down so the
    // running stream never lags behind the settings. Reconnection is explicit.
    if (active()) {
        qCDebug(RTCMStreamManagerLog) << "Source type changed while active - stopping stream";
        stopStream();
    }
}

void RTCMStreamManager::_sendGGATick()
{
    if (!_worker) {
        return;
    }

    // Refresh the cached position from the active vehicle while it is connected.
    Vehicle *const vehicle = MultiVehicleManager::instance()->activeVehicle();
    if (vehicle) {
        const QGeoCoordinate coordinate = vehicle->coordinate();
        if (coordinate.isValid()) {
            _lastGGACoordinate = coordinate;
        }
    }

    // Keep sending the last known position even if the vehicle link drops mid-flight,
    // so the caster keeps the (VRS) stream alive and logging continues.
    if (!_lastGGACoordinate.isValid()) {
        return;
    }

    const QByteArray gga = _buildGGA(_lastGGACoordinate);
    (void) QMetaObject::invokeMethod(_worker, "sendGGA", Qt::QueuedConnection, Q_ARG(QByteArray, gga));
}

QByteArray RTCMStreamManager::_buildGGA(const QGeoCoordinate &coordinate)
{
    const double lat = coordinate.latitude();
    const double lon = coordinate.longitude();
    const double altitude = std::isnan(coordinate.altitude()) ? 0.0 : coordinate.altitude();

    const QChar latHemisphere = (lat < 0.0) ? QChar('S') : QChar('N');
    const QChar lonHemisphere = (lon < 0.0) ? QChar('W') : QChar('E');

    const double absLat = std::abs(lat);
    const double absLon = std::abs(lon);

    const int latDeg = static_cast<int>(absLat);
    const double latMin = (absLat - latDeg) * 60.0;
    const int lonDeg = static_cast<int>(absLon);
    const double lonMin = (absLon - lonDeg) * 60.0;

    const QString utc = QDateTime::currentDateTimeUtc().toString(QStringLiteral("hhmmss.zzz"));

    // $GPGGA,time,lat,N,lon,E,fixQuality,numSats,hdop,alt,M,geoid,M,,*cs
    QString sentence = QStringLiteral("GPGGA,%1,%2%3,%4,%5%6,%7,%8,%9,%10,%11,M,0.0,M,,")
        .arg(utc)
        .arg(latDeg, 2, 10, QChar('0'))
        .arg(latMin, 9, 'f', 6, QChar('0'))
        .arg(latHemisphere)
        .arg(lonDeg, 3, 10, QChar('0'))
        .arg(lonMin, 9, 'f', 6, QChar('0'))
        .arg(lonHemisphere)
        .arg(1)      // fix quality: GPS fix
        .arg(10)     // satellites in use (nominal)
        .arg(0.9, 0, 'f', 1)
        .arg(altitude, 0, 'f', 2);

    quint8 checksum = 0;
    const QByteArray body = sentence.toLatin1();
    for (const char c : body) {
        checksum ^= static_cast<quint8>(c);
    }

    return QStringLiteral("$%1*%2\r\n")
        .arg(sentence)
        .arg(checksum, 2, 16, QChar('0')).toUpper().toLatin1();
}
