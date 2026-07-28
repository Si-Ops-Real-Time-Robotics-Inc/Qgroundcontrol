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
#include <QtCore/QObject>
#include <QtCore/QString>

/// Abstract worker interface for a network RTCM correction source (NTRIP / TCP / UDP).
/// Concrete sources live on their own QThread and are driven by RTCMStreamManager.
/// They only have to emit raw RTCM3 bytes via rtcmData(); RTCMMavlink handles the
/// MAVLink packing and forwarding to the vehicle.
class RTCMNetworkSource : public QObject
{
    Q_OBJECT

public:
    explicit RTCMNetworkSource(QObject *parent = nullptr) : QObject(parent) {}
    ~RTCMNetworkSource() override = default;

public slots:
    /// Open the source (called once the worker thread has started).
    virtual void start() = 0;
    /// Close the source and release resources.
    virtual void stop() = 0;

signals:
    /// Raw RTCM3 bytes received from the source.
    void rtcmData(const QByteArray &data);
    /// The source became connected (true) or disconnected (false).
    void connectedChanged(bool connected);
    /// A non-recoverable error occurred (bad credentials, mountpoint, socket error, ...).
    void errorOccurred(const QString &errorString);
    /// Total number of RTCM bytes received since start(), for bandwidth display.
    void bytesReceived(quint64 totalBytes);
};
