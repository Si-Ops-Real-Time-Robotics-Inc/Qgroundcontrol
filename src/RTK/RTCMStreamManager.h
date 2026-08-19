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

#include <functional>

Q_DECLARE_LOGGING_CATEGORY(RTCMStreamManagerLog)

class BluetoothRtcmScanner;
class FactGroup;
class NetworkRTCMFactGroup;
class RTCMFileLogger;
class RTCMMavlink;
class RTCMNetworkSource;
class RtcmStreamParser;
class NtripSourceTable;
class QThread;

Q_MOC_INCLUDE("FactGroup.h")

/// App singleton that drives a network RTCM correction source (NTRIP / TCP / UDP),
/// validates the incoming bytes as RTCM3 (RtcmStreamParser) before forwarding them to all
/// vehicles via RTCMMavlink, and optionally logs the raw stream to a file.
/// Mirrors the GPSRtk serial pipeline (worker QThread + RTCMMavlink) but transport-agnostic.
class RTCMStreamManager : public QObject
{
    Q_OBJECT
    Q_PROPERTY(FactGroup *rtcmFactGroup READ rtcmFactGroup CONSTANT)
    Q_PROPERTY(bool active READ active NOTIFY activeChanged)
    Q_PROPERTY(QStringList mountpoints READ mountpoints NOTIFY mountpointsChanged)
    Q_PROPERTY(bool fetchingMountpoints READ fetchingMountpoints NOTIFY fetchingMountpointsChanged)
    Q_PROPERTY(QString mountpointsError READ mountpointsError NOTIFY mountpointsErrorChanged)
    Q_PROPERTY(bool bluetoothAvailable READ bluetoothAvailable NOTIFY bluetoothAvailableChanged)
    Q_PROPERTY(QStringList bluetoothDevices READ bluetoothDevices NOTIFY bluetoothDevicesChanged)
    Q_PROPERTY(bool scanningBluetooth READ scanningBluetooth NOTIFY scanningBluetoothChanged)
    Q_PROPERTY(QString bluetoothError READ bluetoothError NOTIFY bluetoothErrorChanged)

public:
    explicit RTCMStreamManager(QObject *parent = nullptr);
    ~RTCMStreamManager() override;

    static RTCMStreamManager *instance();

    FactGroup *rtcmFactGroup();
    bool active() const { return (_worker != nullptr); }
    QStringList mountpoints() const { return _mountpoints; }
    bool fetchingMountpoints() const { return _fetchingMountpoints; }
    QString mountpointsError() const { return _mountpointsError; }
    bool bluetoothAvailable() const { return _bluetoothAvailable; }
    QStringList bluetoothDevices() const { return _bluetoothDevices; }
    bool scanningBluetooth() const { return _scanningBluetooth; }
    QString bluetoothError() const { return _bluetoothError; }

    /// Start streaming using the source configured in RTKSettings.
    Q_INVOKABLE void startStream();
    /// Stop the active stream and release all resources.
    Q_INVOKABLE void stopStream();
    /// Query the configured NTRIP caster's source table to populate mountpoints().
    Q_INVOKABLE void fetchMountpoints();
    /// Discover nearby/paired Bluetooth devices to populate bluetoothDevices().
    Q_INVOKABLE void scanBluetoothDevices();
    /// Cancel an in-progress Bluetooth scan.
    Q_INVOKABLE void stopBluetoothScan();
    /// Store the device at the given bluetoothDevices() index as the RTCM source.
    Q_INVOKABLE void selectBluetoothDevice(int index);

signals:
    void activeChanged();
    void mountpointsChanged();
    void fetchingMountpointsChanged();
    void mountpointsErrorChanged();
    void bluetoothDevicesChanged();
    void scanningBluetoothChanged();
    void bluetoothErrorChanged();
    void bluetoothAvailableChanged();

private slots:
    void _onSourceConnectedChanged(bool connected);
    void _onSourceError(const QString &errorString);
    void _onSourceBytesReceived(quint64 totalBytes);
    void _updateValidationFacts();
    void _onSourceTypeSettingChanged();
    void _onLogSettingChanged();
    void _onLogBytesWritten(quint64 totalBytes);
    void _onBasePositionUpdate(double latitude, double longitude, double altitude, int stationId);
    void _sendGGATick();

private:
    void _startFileLogging();
    void _stopFileLogging();
    /// Runs action() once Bluetooth access is granted. Returns false if the permission is
    /// still pending (action runs later) or was denied (action never runs).
    bool _withBluetoothPermission(const std::function<void()> &action);
    void _setBluetoothError(const QString &errorString);
    /// Re-probe the local adapter. Only meaningful once Bluetooth access has been granted: on Android
    /// the Qt adapter query returns "nothing" until then.
    void _refreshBluetoothAvailable();
    static QByteArray _buildGGA(const QGeoCoordinate &coordinate);

    RTCMNetworkSource *_worker = nullptr;
    QThread *_workerThread = nullptr;
    RTCMMavlink *_rtcmMavlink = nullptr;
    RTCMFileLogger *_fileLogger = nullptr;
    RtcmStreamParser *_streamParser = nullptr;
    NetworkRTCMFactGroup *_factGroup = nullptr;
    NtripSourceTable *_sourceTable = nullptr;

    QStringList _mountpoints;
    bool _fetchingMountpoints = false;
    QString _mountpointsError;

    BluetoothRtcmScanner *_bluetoothScanner = nullptr;
    QStringList _bluetoothDevices;
    bool _scanningBluetooth = false;
    QString _bluetoothError;
#ifdef QGC_ENABLE_BLUETOOTH
    /// Starts optimistic so the UI does not claim "no adapter" before the user has had a chance to
    /// grant Bluetooth access (Android reports no adapter until then). Corrected by the scan/connect path.
    bool _bluetoothAvailable = true;
#else
    bool _bluetoothAvailable = false;
#endif

    QTimer _ggaTimer;
    QGeoCoordinate _lastGGACoordinate;  ///< last known drone position, kept so GGA keeps flowing if the vehicle drops
    QElapsedTimer _rateTimer;
    quint64 _lastValidByteCount = 0;  ///< validated RTCM bytes at the last rate tick

    static constexpr uint32_t kThreadDisconnectTimeout = 2000;
    static constexpr int kGGAIntervalMs = 10000;
};
