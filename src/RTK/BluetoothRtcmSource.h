/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include <QtBluetooth/QBluetoothDeviceDiscoveryAgent>
#include <QtBluetooth/QBluetoothDeviceInfo>
#include <QtBluetooth/QBluetoothServiceDiscoveryAgent>
#include <QtBluetooth/QBluetoothServiceInfo>
#include <QtBluetooth/QBluetoothSocket>
#include <QtCore/QLoggingCategory>
#include <QtCore/QStringList>

#include "RTCMNetworkSource.h"

Q_DECLARE_LOGGING_CATEGORY(BluetoothRtcmSourceLog)

/*===========================================================================*/

/// Discovers classic Bluetooth (RFCOMM / Serial Port Profile) devices so the user can pick
/// an RTK receiver or a base station radio from the RTK settings page.
/// Lives on the main thread; the discovery agent is asynchronous.
class BluetoothRtcmScanner : public QObject
{
    Q_OBJECT

public:
    explicit BluetoothRtcmScanner(QObject *parent = nullptr);
    ~BluetoothRtcmScanner() override;

    void start();
    void stop();
    bool scanning() const;

    /// Display strings ("name (address)") for the devices found so far.
    QStringList deviceLabels() const { return _labels; }
    QString nameAt(int index) const;
    QString addressAt(int index) const;

signals:
    void devicesChanged();
    void scanningChanged();
    void errorOccurred(const QString &errorString);

private slots:
    void _deviceDiscovered(const QBluetoothDeviceInfo &info);
    void _onErrorOccurred(QBluetoothDeviceDiscoveryAgent::Error error);

private:
    QBluetoothDeviceDiscoveryAgent *_agent = nullptr;
    QStringList _labels;
    QStringList _names;
    QStringList _addresses;     ///< BT address, or the device UUID on iOS
};

/*===========================================================================*/

/// Streams raw RTCM3 from a paired Bluetooth device over RFCOMM (Serial Port Profile).
/// Typical sources are an RTK receiver or a base station radio exposing a serial bridge.
///
/// Except on Android, the SPP service is discovered here rather than by letting
/// QBluetoothSocket::connectToService(address, uuid) run its own discovery: that path crashes Qt
/// for devices publishing more than one RFCOMM record (its serviceDiscovered() slot clears
/// d->discoveryAgent after the first record, then dereferences the null pointer on the second).
/// Receivers exposing COM0/COM1 hit this every time.
class BluetoothRtcmSource : public RTCMNetworkSource
{
    Q_OBJECT

public:
    struct Config {
        QString deviceName;
        QString deviceAddress;  ///< BT address, or the device UUID on iOS
    };

    explicit BluetoothRtcmSource(const Config &config, QObject *parent = nullptr);
    ~BluetoothRtcmSource() override;

public slots:
    void start() override;
    void stop() override;

private slots:
    void _onSocketConnected();
    void _onSocketDisconnected();
    void _onSocketReadyRead();
    void _onSocketErrorOccurred(QBluetoothSocket::SocketError socketError);
    void _serviceDiscovered(const QBluetoothServiceInfo &info);
    void _onServiceDiscoveryFinished();
    void _onServiceErrorOccurred(QBluetoothServiceDiscoveryAgent::Error error);

private:
    const Config _config;
    QBluetoothSocket *_socket = nullptr;
    QBluetoothServiceDiscoveryAgent *_serviceDiscoveryAgent = nullptr;
    QBluetoothServiceInfo _bestService;     ///< lowest RFCOMM channel seen, connected once discovery ends
    quint64 _totalBytes = 0;
};
