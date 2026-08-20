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

Q_DECLARE_LOGGING_CATEGORY(RtcmNavDecoderLog)

/// One broadcast ephemeris set. Keplerian for GPS/Galileo/BeiDou/QZSS; GLONASS instead carries a
/// position/velocity/acceleration state vector, so the two halves of this struct are exclusive.
struct RtcmEphemeris
{
    char system = 0;            ///< RINEX system letter: G R E C J
    int prn = 0;

    // --- Keplerian (GPS / Galileo / BeiDou / QZSS) ---
    int week = 0;               ///< constellation week number, already un-rolled where needed
    int iode = 0;
    int iodc = 0;
    int health = 0;
    int uraIndex = 0;           ///< URA / SISA / URAI index, not metres
    int codesOnL2 = 0;          ///< GPS/QZSS codes on L2, or Galileo data sources
    double toc = 0.0;           ///< clock reference, seconds into the week
    double toe = 0.0;           ///< ephemeris reference, seconds into the week
    double af0 = 0.0;
    double af1 = 0.0;
    double af2 = 0.0;
    double crs = 0.0;
    double crc = 0.0;
    double cuc = 0.0;
    double cus = 0.0;
    double cic = 0.0;
    double cis = 0.0;
    double deltaN = 0.0;        ///< rad/s
    double m0 = 0.0;            ///< rad
    double eccentricity = 0.0;
    double sqrtA = 0.0;         ///< sqrt(m)
    double omega0 = 0.0;        ///< rad
    double i0 = 0.0;            ///< rad
    double omega = 0.0;         ///< rad, argument of perigee
    double omegaDot = 0.0;      ///< rad/s
    double idot = 0.0;          ///< rad/s
    double tgd = 0.0;           ///< GPS/QZSS TGD, Galileo BGD E5a/E1, BeiDou TGD1
    double tgd2 = 0.0;          ///< Galileo BGD E5b/E1, BeiDou TGD2
    double fitInterval = 0.0;
    bool galileoInav = false;   ///< set for 1046, clear for 1045

    // --- GLONASS ---
    int frequencyChannel = 0;   ///< k, -7..+6
    int ageOfData = 0;
    int dayNumber = 0;          ///< Nt, day within the four-year interval
    int fourYearInterval = 0;   ///< N4
    int frameTime = 0;          ///< tk
    int intervalIndex = 0;      ///< tb, 15 minute units of the Moscow day
    double positionX = 0.0;     ///< metres, PZ-90
    double positionY = 0.0;
    double positionZ = 0.0;
    double velocityX = 0.0;     ///< m/s
    double velocityY = 0.0;
    double velocityZ = 0.0;
    double accelerationX = 0.0; ///< m/s^2
    double accelerationY = 0.0;
    double accelerationZ = 0.0;
    double tauN = 0.0;          ///< clock bias, seconds
    double gammaN = 0.0;        ///< relative frequency offset

    /// Identity of this ephemeris set: the same satellite re-broadcasting the same issue of data
    /// must not produce a second RINEX record.
    QByteArray key() const;
};

Q_DECLARE_METATYPE(RtcmEphemeris)

/// Decodes RTCM3 broadcast ephemeris messages - 1019 (GPS), 1020 (GLONASS), 1042 (BeiDou),
/// 1044 (QZSS), 1045 (Galileo F/NAV) and 1046 (Galileo I/NAV) - into RINEX-ready records.
/// Feed it the validated frames from RtcmStreamParser.
class RtcmNavDecoder : public QObject
{
    Q_OBJECT

public:
    explicit RtcmNavDecoder(QObject *parent = nullptr);
    ~RtcmNavDecoder() override;

    quint64 decodedCount() const { return _decodedCount; }

public slots:
    /// Feed one or more complete, already CRC-validated RTCM3 frames.
    void addFrames(const QByteArray &frames);
    void reset();

signals:
    void ephemerisReady(const RtcmEphemeris &ephemeris);

private:
    void _decodeFrame(const quint8 *payload, int payloadLen);
    void _decodeGps(const quint8 *payload, int payloadLen);
    void _decodeQzss(const quint8 *payload, int payloadLen);
    void _decodeBeidou(const quint8 *payload, int payloadLen);
    void _decodeGalileo(const quint8 *payload, int payloadLen, bool inav);
    void _decodeGlonass(const quint8 *payload, int payloadLen);
    void _emit(const RtcmEphemeris &ephemeris);

    quint64 _decodedCount = 0;
};
