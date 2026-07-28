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

Q_DECLARE_LOGGING_CATEGORY(RtcmBaseParserLog)

/// Streaming RTCM3 framer that extracts the base-station antenna position from
/// message types 1005 / 1006 (stationary reference station ARP). It is a passive
/// tap on the RTCM byte stream - it never modifies or blocks the data forwarded
/// to the vehicle.
class RtcmBaseParser : public QObject
{
    Q_OBJECT

public:
    explicit RtcmBaseParser(QObject *parent = nullptr);
    ~RtcmBaseParser() override;

public slots:
    /// Feed raw RTCM3 bytes (arbitrary chunk boundaries).
    void addData(const QByteArray &data);
    /// Reset framing state (called on (re)connect).
    void reset();

signals:
    /// A 1005/1006 message was decoded into a WGS84 base position.
    void basePositionUpdate(double latitude, double longitude, double altitude, int stationId);

private:
    void _parseFrames();
    static void _ecefToGeodetic(double x, double y, double z, double &latDeg, double &lonDeg, double &alt);

    QByteArray _buffer;

    static constexpr int kMaxBufferBytes = 16384;
};
