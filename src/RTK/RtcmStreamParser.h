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

Q_DECLARE_LOGGING_CATEGORY(RtcmStreamParserLog)

/// Streaming RTCM3 framer and validator. It sits between a correction source
/// (NTRIP / TCP / UDP / Bluetooth) and RTCMMavlink, and is the gate that keeps
/// non-RTCM bytes off the MAVLink link: only frames with a valid RTCM3 preamble,
/// length and CRC-24Q are forwarded via rtcmFrames(). Anything else - HTML error
/// pages from a caster, NMEA from a misconfigured receiver, line noise, partial
/// frames - is counted in discardedBytes() and dropped.
///
/// It also decodes the base-station antenna position from message types
/// 1005 / 1006 (stationary reference station ARP) for map display.
class RtcmStreamParser : public QObject
{
    Q_OBJECT

public:
    explicit RtcmStreamParser(QObject *parent = nullptr);
    ~RtcmStreamParser() override;

    /// Number of complete, CRC-valid RTCM3 frames seen since the last reset().
    quint64 validFrames() const { return _validFrames; }
    /// Number of bytes contained in those frames.
    quint64 validBytes() const { return _validBytes; }
    /// Number of bytes dropped because they were not part of a valid RTCM3 frame.
    quint64 discardedBytes() const { return _discardedBytes; }

public slots:
    /// Feed raw bytes from the source (arbitrary chunk boundaries).
    void addData(const QByteArray &data);
    /// Drop framing state and statistics (called on (re)connect).
    void reset();

signals:
    /// One or more complete, CRC-validated RTCM3 frames, ready to forward to the vehicle.
    /// Batched up to kMaxBatchBytes so RTCMMavlink can pack them into as few
    /// GPS_RTCM_DATA sequences as possible.
    void rtcmFrames(const QByteArray &frames);
    /// A 1005/1006 message was decoded into a WGS84 base position.
    void basePositionUpdate(double latitude, double longitude, double altitude, int stationId);

private:
    void _parseFrames();
    void _handleFrame(const QByteArray &frame);
    void _appendToBatch(const QByteArray &frame);
    void _flushBatch();
    void _discard(qsizetype bytes);
    static void _ecefToGeodetic(double x, double y, double z, double &latDeg, double &lonDeg, double &alt);

    QByteArray _buffer;     ///< unparsed input
    QByteArray _batch;      ///< validated frames waiting to be emitted

    quint64 _validFrames = 0;
    quint64 _validBytes = 0;
    quint64 _discardedBytes = 0;
    quint64 _lastWarnedDiscardedBytes = 0;

    /// A GPS_RTCM_DATA sequence carries at most 4 fragments of 180 bytes (the fragment id
    /// field is 2 bits wide), so batching beyond that would only force an extra sequence.
    static constexpr qsizetype kMaxBatchBytes = 4 * 180;
    /// Largest possible RTCM3 frame: 3 byte header + 1023 byte payload + 3 byte CRC.
    static constexpr int kMaxFrameBytes = 3 + 1023 + 3;
    /// Enough for several max-size frames; anything beyond this is not a framed RTCM stream.
    static constexpr int kMaxBufferBytes = 16384;
    /// Warn at most once per this many discarded bytes, so a garbage stream cannot flood the log.
    static constexpr quint64 kDiscardWarnInterval = 4096;
};
