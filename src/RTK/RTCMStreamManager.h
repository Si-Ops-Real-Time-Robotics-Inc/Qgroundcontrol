/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include <QtCore/QElapsedTimer>
#include <QtCore/QLoggingCategory>
#include <QtCore/QObject>
#include <QtCore/QStringList>
#include <QtCore/QTimer>
#include <QtPositioning/QGeoCoordinate>

Q_DECLARE_LOGGING_CATEGORY(RTCMStreamManagerLog)

class FactGroup;
class NetworkRTCMFactGroup;
class RTCMFileLogger;
class RTCMMavlink;
class RTCMNetworkSource;
class RtcmBaseParser;
class NtripSourceTable;
class QThread;

Q_MOC_INCLUDE("FactGroup.h")

/// App singleton that drives a network RTCM correction source (NTRIP / TCP / UDP),
/// forwards the RTCM to all vehicles via RTCMMavlink, and optionally logs it to a file.
/// Mirrors the GPSRtk serial pipeline (worker QThread + RTCMMavlink) but transport-agnostic.
class RTCMStreamManager : public QObject
{
    Q_OBJECT
    Q_PROPERTY(FactGroup *rtcmFactGroup READ rtcmFactGroup CONSTANT)
    Q_PROPERTY(bool active READ active NOTIFY activeChanged)
    Q_PROPERTY(QStringList mountpoints READ mountpoints NOTIFY mountpointsChanged)
    Q_PROPERTY(bool fetchingMountpoints READ fetchingMountpoints NOTIFY fetchingMountpointsChanged)
    Q_PROPERTY(QString mountpointsError READ mountpointsError NOTIFY mountpointsErrorChanged)

public:
    explicit RTCMStreamManager(QObject *parent = nullptr);
    ~RTCMStreamManager() override;

    static RTCMStreamManager *instance();

    FactGroup *rtcmFactGroup();
    bool active() const { return (_workerThread != nullptr); }
    QStringList mountpoints() const { return _mountpoints; }
    bool fetchingMountpoints() const { return _fetchingMountpoints; }
    QString mountpointsError() const { return _mountpointsError; }

    /// Start streaming using the source configured in RTKSettings.
    Q_INVOKABLE void startStream();
    /// Stop the active stream and release all resources.
    Q_INVOKABLE void stopStream();
    /// Query the configured NTRIP caster's source table to populate mountpoints().
    Q_INVOKABLE void fetchMountpoints();

signals:
    void activeChanged();
    void mountpointsChanged();
    void fetchingMountpointsChanged();
    void mountpointsErrorChanged();

private slots:
    void _onSourceConnectedChanged(bool connected);
    void _onSourceError(const QString &errorString);
    void _onSourceBytesReceived(quint64 totalBytes);
    void _onSourceTypeSettingChanged();
    void _onLogSettingChanged();
    void _onLogBytesWritten(quint64 totalBytes);
    void _onBasePositionUpdate(double latitude, double longitude, double altitude, int stationId);
    void _sendGGATick();

private:
    void _startFileLogging();
    void _stopFileLogging();
    static QByteArray _buildGGA(const QGeoCoordinate &coordinate);

    RTCMNetworkSource *_worker = nullptr;
    QThread *_workerThread = nullptr;
    RTCMMavlink *_rtcmMavlink = nullptr;
    RTCMFileLogger *_fileLogger = nullptr;
    RtcmBaseParser *_baseParser = nullptr;
    NetworkRTCMFactGroup *_factGroup = nullptr;
    NtripSourceTable *_sourceTable = nullptr;

    QStringList _mountpoints;
    bool _fetchingMountpoints = false;
    QString _mountpointsError;

    QTimer _ggaTimer;
    QGeoCoordinate _lastGGACoordinate;  ///< last known drone position, kept so GGA keeps flowing if the vehicle drops
    QElapsedTimer _rateTimer;
    quint64 _lastByteCount = 0;

    static constexpr uint32_t kThreadDisconnectTimeout = 2000;
    static constexpr int kGGAIntervalMs = 10000;
};
