/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "BluetoothRtcmSource.h"
#include "QGCLoggingCategory.h"

#include <QtBluetooth/QBluetoothAddress>
#include <QtBluetooth/QBluetoothServiceInfo>
#include <QtBluetooth/QBluetoothUuid>

QGC_LOGGING_CATEGORY(BluetoothRtcmSourceLog, "qgc.rtk.bluetoothrtcmsource")

/*===========================================================================*/

BluetoothRtcmScanner::BluetoothRtcmScanner(QObject *parent)
    : QObject(parent)
    , _agent(new QBluetoothDeviceDiscoveryAgent(this))
{
    (void) connect(_agent, &QBluetoothDeviceDiscoveryAgent::deviceDiscovered, this, &BluetoothRtcmScanner::_deviceDiscovered);
    (void) connect(_agent, &QBluetoothDeviceDiscoveryAgent::canceled, this, &BluetoothRtcmScanner::scanningChanged);
    (void) connect(_agent, &QBluetoothDeviceDiscoveryAgent::finished, this, &BluetoothRtcmScanner::scanningChanged);
    (void) connect(_agent, &QBluetoothDeviceDiscoveryAgent::errorOccurred, this, &BluetoothRtcmScanner::_onErrorOccurred);
}

BluetoothRtcmScanner::~BluetoothRtcmScanner()
{
    BluetoothRtcmScanner::stop();
}

void BluetoothRtcmScanner::start()
{
    _labels.clear();
    _names.clear();
    _addresses.clear();
    emit devicesChanged();

    _agent->start();
    emit scanningChanged();
}

void BluetoothRtcmScanner::stop()
{
    if (scanning()) {
        _agent->stop();
    }
}

bool BluetoothRtcmScanner::scanning() const
{
    return _agent->isActive();
}

QString BluetoothRtcmScanner::nameAt(int index) const
{
    return ((index >= 0) && (index < _names.count())) ? _names.at(index) : QString();
}

QString BluetoothRtcmScanner::addressAt(int index) const
{
    return ((index >= 0) && (index < _addresses.count())) ? _addresses.at(index) : QString();
}

void BluetoothRtcmScanner::_deviceDiscovered(const QBluetoothDeviceInfo &info)
{
    if (info.name().isEmpty() || !info.isValid()) {
        return;
    }

#ifdef Q_OS_IOS
    const QString address = info.deviceUuid().toString();
#else
    const QString address = info.address().toString();
#endif
    if (address.isEmpty() || _addresses.contains(address)) {
        return;
    }

    _names.append(info.name());
    _addresses.append(address);
    _labels.append(QStringLiteral("%1 (%2)").arg(info.name(), address));
    emit devicesChanged();
}

void BluetoothRtcmScanner::_onErrorOccurred(QBluetoothDeviceDiscoveryAgent::Error error)
{
    const QString errorString = _agent->errorString();
    qCWarning(BluetoothRtcmSourceLog) << "Discovery error:" << error << errorString;
    emit errorOccurred(errorString);
    emit scanningChanged();
}

/*===========================================================================*/

BluetoothRtcmSource::BluetoothRtcmSource(const Config &config, QObject *parent)
    : RTCMNetworkSource(parent)
    , _config(config)
{
    // qCDebug(BluetoothRtcmSourceLog) << Q_FUNC_INFO << this;
}

BluetoothRtcmSource::~BluetoothRtcmSource()
{
    BluetoothRtcmSource::stop();

    // qCDebug(BluetoothRtcmSourceLog) << Q_FUNC_INFO << this;
}

void BluetoothRtcmSource::start()
{
    if (_socket) {
        qCWarning(BluetoothRtcmSourceLog) << "start() called while already active";
        return;
    }

    _totalBytes = 0;
    _bestService = QBluetoothServiceInfo();

    _socket = new QBluetoothSocket(QBluetoothServiceInfo::RfcommProtocol, this);
    (void) connect(_socket, &QBluetoothSocket::connected, this, &BluetoothRtcmSource::_onSocketConnected);
    (void) connect(_socket, &QBluetoothSocket::disconnected, this, &BluetoothRtcmSource::_onSocketDisconnected);
    (void) connect(_socket, &QBluetoothSocket::readyRead, this, &BluetoothRtcmSource::_onSocketReadyRead);
    (void) connect(_socket, &QBluetoothSocket::errorOccurred, this, &BluetoothRtcmSource::_onSocketErrorOccurred);

    qCDebug(BluetoothRtcmSourceLog) << "Connecting to" << _config.deviceName << _config.deviceAddress;

#ifdef Q_OS_ANDROID
    // Android connects straight to the SPP UUID - no service discovery involved, so the Qt
    // multiple-records crash described in the header cannot happen there.
    static constexpr QBluetoothUuid uuid = QBluetoothUuid(QBluetoothUuid::ServiceClassUuid::SerialPort);
    _socket->connectToService(QBluetoothAddress(_config.deviceAddress), uuid);
#else
    _serviceDiscoveryAgent = new QBluetoothServiceDiscoveryAgent(this);
#ifndef Q_OS_IOS
    // Restrict the query to the selected device. iOS has no address, so it filters by device UUID
    // in _serviceDiscovered() instead.
    if (!_serviceDiscoveryAgent->setRemoteAddress(QBluetoothAddress(_config.deviceAddress))) {
        const QString msg = tr("Bluetooth: invalid device address %1").arg(_config.deviceAddress);
        qCWarning(BluetoothRtcmSourceLog) << msg;
        emit errorOccurred(msg);
        stop();
        return;
    }
#endif
    _serviceDiscoveryAgent->setUuidFilter(QBluetoothUuid(QBluetoothUuid::ServiceClassUuid::SerialPort));

    (void) connect(_serviceDiscoveryAgent, &QBluetoothServiceDiscoveryAgent::serviceDiscovered, this, &BluetoothRtcmSource::_serviceDiscovered);
    (void) connect(_serviceDiscoveryAgent, &QBluetoothServiceDiscoveryAgent::finished, this, &BluetoothRtcmSource::_onServiceDiscoveryFinished);
    (void) connect(_serviceDiscoveryAgent, &QBluetoothServiceDiscoveryAgent::canceled, this, &BluetoothRtcmSource::_onServiceDiscoveryFinished);
    (void) connect(_serviceDiscoveryAgent, &QBluetoothServiceDiscoveryAgent::errorOccurred, this, &BluetoothRtcmSource::_onServiceErrorOccurred);

    _serviceDiscoveryAgent->start(QBluetoothServiceDiscoveryAgent::FullDiscovery);
#endif
}

void BluetoothRtcmSource::stop()
{
    if (_serviceDiscoveryAgent) {
        _serviceDiscoveryAgent->disconnect(this);
        if (_serviceDiscoveryAgent->isActive()) {
            _serviceDiscoveryAgent->stop();
        }
        _serviceDiscoveryAgent->deleteLater();
        _serviceDiscoveryAgent = nullptr;
    }

    if (!_socket) {
        return;
    }

    _socket->disconnect(this);
    _socket->abort();
    _socket->deleteLater();
    _socket = nullptr;
}

void BluetoothRtcmSource::_onSocketConnected()
{
    qCDebug(BluetoothRtcmSourceLog) << "Connected to" << _config.deviceName;
    emit connectedChanged(true);
}

void BluetoothRtcmSource::_onSocketDisconnected()
{
    qCDebug(BluetoothRtcmSourceLog) << "Disconnected from" << _config.deviceName;
    emit connectedChanged(false);
}

void BluetoothRtcmSource::_onSocketReadyRead()
{
    if (!_socket) {
        return;
    }

    const QByteArray data = _socket->readAll();
    if (data.isEmpty()) {
        return;
    }

    _totalBytes += static_cast<quint64>(data.size());
    emit rtcmData(data);
    emit bytesReceived(_totalBytes);
}

void BluetoothRtcmSource::_onSocketErrorOccurred(QBluetoothSocket::SocketError socketError)
{
    if (!_socket) {
        return;
    }

    // Safety net for a stray second connect attempt (Qt reports OperationError for it while the
    // first one is still in flight). The original attempt carries on, so it is not a stream failure.
    if ((socketError == QBluetoothSocket::SocketError::OperationError) &&
        (_socket->state() != QBluetoothSocket::SocketState::UnconnectedState)) {
        qCDebug(BluetoothRtcmSourceLog) << "Ignoring spurious operation error, socket state" << _socket->state();
        return;
    }

    const QString errorString = _socket->errorString();
    qCWarning(BluetoothRtcmSourceLog) << "Socket error:" << socketError << errorString;
    emit errorOccurred(tr("Bluetooth: %1").arg(errorString));
}

void BluetoothRtcmSource::_serviceDiscovered(const QBluetoothServiceInfo &info)
{
    if (!_socket || !_serviceDiscoveryAgent || !info.isValid()) {
        return;
    }

#ifdef Q_OS_IOS
    if (info.device().deviceUuid().toString() != _config.deviceAddress) {
        return;
    }
#endif

    if ((info.serverChannel() <= 0) && (info.protocolServiceMultiplexer() <= 0)) {
        return;
    }

    // Collect rather than connect: receivers expose several serial ports (COM0/COM1) and the
    // discovery order varies between runs, so always settle on the lowest RFCOMM channel to make
    // the choice reproducible. The connect itself happens once, in _onServiceDiscoveryFinished().
    qCDebug(BluetoothRtcmSourceLog) << "Found service" << info.serviceName() << "channel" << info.serverChannel();
    if (!_bestService.isValid() || (info.serverChannel() < _bestService.serverChannel())) {
        _bestService = info;
    }
}

void BluetoothRtcmSource::_onServiceDiscoveryFinished()
{
    if (!_socket || (_socket->state() != QBluetoothSocket::SocketState::UnconnectedState)) {
        return;
    }

    if (!_bestService.isValid()) {
        emit errorOccurred(tr("Bluetooth: no serial port service found on %1").arg(_config.deviceName));
        return;
    }

    // Stop listening before connecting: a late result must never trigger a second connect.
    if (_serviceDiscoveryAgent) {
        _serviceDiscoveryAgent->disconnect(this);
        _serviceDiscoveryAgent->stop();
    }

    qCDebug(BluetoothRtcmSourceLog) << "Connecting to service" << _bestService.serviceName()
                                    << "channel" << _bestService.serverChannel();
    _socket->connectToService(_bestService);
}

void BluetoothRtcmSource::_onServiceErrorOccurred(QBluetoothServiceDiscoveryAgent::Error error)
{
    const QString errorString = _serviceDiscoveryAgent ? _serviceDiscoveryAgent->errorString() : QString();
    qCWarning(BluetoothRtcmSourceLog) << "Service discovery error:" << error << errorString;
    emit errorOccurred(tr("Bluetooth: %1").arg(errorString));
}
