/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "RtcmNavDecoder.h"
#include "QGCLoggingCategory.h"
#include "RtcmBitReader.h"

#include <cmath>

QGC_LOGGING_CATEGORY(RtcmNavDecoderLog, "qgc.rtk.rtcmnavdecoder")

namespace {

/// Semicircles to radians, using the value fixed by the GPS interface specification rather than
/// M_PI - the broadcast parameters are defined against it.
constexpr double kSemicircleToRad = 3.1415926535898;

constexpr double p2(int exponent)
{
    return (exponent < 0) ? (1.0 / static_cast<double>(1ULL << -exponent))
                          : static_cast<double>(1ULL << exponent);
}

// Scale factors used by more than one constellation.
const double kP2_5  = p2(-5);
const double kP2_19 = p2(-19);
const double kP2_29 = p2(-29);
const double kP2_31 = p2(-31);
const double kP2_33 = p2(-33);
const double kP2_43 = p2(-43);
const double kP2_55 = p2(-55);

} // namespace

QByteArray RtcmEphemeris::key() const
{
    // A satellite rebroadcasts the same ephemeris for hours; the reference time plus the issue of
    // data identifies one distinct set. GLONASS has no IODE, so its interval index stands in.
    if (system == 'R') {
        return QByteArrayLiteral("R") + QByteArray::number(prn) + '|'
               + QByteArray::number(dayNumber) + '|' + QByteArray::number(intervalIndex);
    }
    return QByteArray(1, system) + QByteArray::number(prn) + '|'
           + QByteArray::number(week) + '|' + QByteArray::number(static_cast<qint64>(toe)) + '|'
           + QByteArray::number(iode) + (galileoInav ? "|I" : "");
}

RtcmNavDecoder::RtcmNavDecoder(QObject *parent)
    : QObject(parent)
{
}

RtcmNavDecoder::~RtcmNavDecoder()
{
}

void RtcmNavDecoder::reset()
{
    _decodedCount = 0;
}

void RtcmNavDecoder::addFrames(const QByteArray &frames)
{
    // The parser only hands over complete, CRC-valid frames, so walking by length is safe.
    const quint8 *data = reinterpret_cast<const quint8 *>(frames.constData());
    qsizetype offset = 0;
    while ((offset + 6) <= frames.size()) {
        const int payloadLen = (static_cast<int>(data[offset + 1] & 0x03) << 8) | static_cast<int>(data[offset + 2]);
        const qsizetype frameLen = 3 + payloadLen + 3;
        if ((offset + frameLen) > frames.size()) {
            break;
        }
        _decodeFrame(data + offset + 3, payloadLen);
        offset += frameLen;
    }
}

void RtcmNavDecoder::_decodeFrame(const quint8 *payload, int payloadLen)
{
    if (payloadLen < 2) {
        return;
    }

    const int messageNumber = (static_cast<int>(payload[0]) << 4) | (static_cast<int>(payload[1]) >> 4);
    switch (messageNumber) {
    case 1019: _decodeGps(payload, payloadLen); break;
    case 1020: _decodeGlonass(payload, payloadLen); break;
    case 1042: _decodeBeidou(payload, payloadLen); break;
    case 1044: _decodeQzss(payload, payloadLen); break;
    case 1045: _decodeGalileo(payload, payloadLen, false); break;
    case 1046: _decodeGalileo(payload, payloadLen, true); break;
    default: break;
    }
}

void RtcmNavDecoder::_decodeGps(const quint8 *payload, int payloadLen)
{
    // 488 bits.
    if ((payloadLen * 8) < 488) {
        return;
    }

    RtcmBitReader r(payload, payloadLen);
    RtcmEphemeris e;
    e.system = 'G';
    r.skip(12);
    e.prn = static_cast<int>(r.u(6));
    e.week = static_cast<int>(r.u(10));
    e.uraIndex = static_cast<int>(r.u(4));
    e.codesOnL2 = static_cast<int>(r.u(2));
    e.idot = static_cast<double>(r.s(14)) * kP2_43 * kSemicircleToRad;
    e.iode = static_cast<int>(r.u(8));
    e.toc = static_cast<double>(r.u(16)) * 16.0;
    e.af2 = static_cast<double>(r.s(8)) * kP2_55;
    e.af1 = static_cast<double>(r.s(16)) * kP2_43;
    e.af0 = static_cast<double>(r.s(22)) * kP2_31;
    e.iodc = static_cast<int>(r.u(10));
    e.crs = static_cast<double>(r.s(16)) * kP2_5;
    e.deltaN = static_cast<double>(r.s(16)) * kP2_43 * kSemicircleToRad;
    e.m0 = static_cast<double>(r.s(32)) * kP2_31 * kSemicircleToRad;
    e.cuc = static_cast<double>(r.s(16)) * kP2_29;
    e.eccentricity = static_cast<double>(r.u(32)) * kP2_33;
    e.cus = static_cast<double>(r.s(16)) * kP2_29;
    e.sqrtA = static_cast<double>(r.u(32)) * kP2_19;
    e.toe = static_cast<double>(r.u(16)) * 16.0;
    e.cic = static_cast<double>(r.s(16)) * kP2_29;
    e.omega0 = static_cast<double>(r.s(32)) * kP2_31 * kSemicircleToRad;
    e.cis = static_cast<double>(r.s(16)) * kP2_29;
    e.i0 = static_cast<double>(r.s(32)) * kP2_31 * kSemicircleToRad;
    e.crc = static_cast<double>(r.s(16)) * kP2_5;
    e.omega = static_cast<double>(r.s(32)) * kP2_31 * kSemicircleToRad;
    e.omegaDot = static_cast<double>(r.s(24)) * kP2_43 * kSemicircleToRad;
    e.tgd = static_cast<double>(r.s(8)) * kP2_31;
    e.health = static_cast<int>(r.u(6));
    r.skip(1);                                          // L2 P data flag
    e.fitInterval = (r.u(1) != 0) ? 0.0 : 4.0;          // 0 = 4 hour fit, 1 = longer

    if (!r.overrun()) {
        _emit(e);
    }
}

void RtcmNavDecoder::_decodeQzss(const quint8 *payload, int payloadLen)
{
    // 485 bits. Same parameters as GPS but in a different field order.
    if ((payloadLen * 8) < 485) {
        return;
    }

    RtcmBitReader r(payload, payloadLen);
    RtcmEphemeris e;
    e.system = 'J';
    r.skip(12);
    e.prn = static_cast<int>(r.u(4)) + 192;             // QZSS PRN = id + 192
    e.toc = static_cast<double>(r.u(16)) * 16.0;
    e.af2 = static_cast<double>(r.s(8)) * kP2_55;
    e.af1 = static_cast<double>(r.s(16)) * kP2_43;
    e.af0 = static_cast<double>(r.s(22)) * kP2_31;
    e.iode = static_cast<int>(r.u(8));
    e.crs = static_cast<double>(r.s(16)) * kP2_5;
    e.deltaN = static_cast<double>(r.s(16)) * kP2_43 * kSemicircleToRad;
    e.m0 = static_cast<double>(r.s(32)) * kP2_31 * kSemicircleToRad;
    e.cuc = static_cast<double>(r.s(16)) * kP2_29;
    e.eccentricity = static_cast<double>(r.u(32)) * kP2_33;
    e.cus = static_cast<double>(r.s(16)) * kP2_29;
    e.sqrtA = static_cast<double>(r.u(32)) * kP2_19;
    e.toe = static_cast<double>(r.u(16)) * 16.0;
    e.cic = static_cast<double>(r.s(16)) * kP2_29;
    e.omega0 = static_cast<double>(r.s(32)) * kP2_31 * kSemicircleToRad;
    e.cis = static_cast<double>(r.s(16)) * kP2_29;
    e.i0 = static_cast<double>(r.s(32)) * kP2_31 * kSemicircleToRad;
    e.crc = static_cast<double>(r.s(16)) * kP2_5;
    e.omega = static_cast<double>(r.s(32)) * kP2_31 * kSemicircleToRad;
    e.omegaDot = static_cast<double>(r.s(24)) * kP2_43 * kSemicircleToRad;
    e.idot = static_cast<double>(r.s(14)) * kP2_43 * kSemicircleToRad;
    e.codesOnL2 = static_cast<int>(r.u(2));
    e.week = static_cast<int>(r.u(10));
    e.uraIndex = static_cast<int>(r.u(4));
    e.health = static_cast<int>(r.u(6));
    e.tgd = static_cast<double>(r.s(8)) * kP2_31;
    e.iodc = static_cast<int>(r.u(10));
    e.fitInterval = (r.u(1) != 0) ? 0.0 : 2.0;

    if (!r.overrun()) {
        _emit(e);
    }
}

void RtcmNavDecoder::_decodeBeidou(const quint8 *payload, int payloadLen)
{
    // 511 bits. BeiDou uses wider correction terms and 8 s time resolution.
    if ((payloadLen * 8) < 511) {
        return;
    }

    RtcmBitReader r(payload, payloadLen);
    RtcmEphemeris e;
    e.system = 'C';
    r.skip(12);
    e.prn = static_cast<int>(r.u(6));
    e.week = static_cast<int>(r.u(13));
    e.uraIndex = static_cast<int>(r.u(4));
    e.idot = static_cast<double>(r.s(14)) * kP2_43 * kSemicircleToRad;
    e.iode = static_cast<int>(r.u(5));                  // AODE
    e.toc = static_cast<double>(r.u(17)) * 8.0;
    e.af2 = static_cast<double>(r.s(11)) * p2(-66);
    e.af1 = static_cast<double>(r.s(22)) * p2(-50);
    e.af0 = static_cast<double>(r.s(24)) * kP2_33;
    e.iodc = static_cast<int>(r.u(5));                  // AODC
    e.crs = static_cast<double>(r.s(18)) * p2(-6);
    e.deltaN = static_cast<double>(r.s(16)) * kP2_43 * kSemicircleToRad;
    e.m0 = static_cast<double>(r.s(32)) * kP2_31 * kSemicircleToRad;
    e.cuc = static_cast<double>(r.s(18)) * kP2_31;
    e.eccentricity = static_cast<double>(r.u(32)) * kP2_33;
    e.cus = static_cast<double>(r.s(18)) * kP2_31;
    e.sqrtA = static_cast<double>(r.u(32)) * kP2_19;
    e.toe = static_cast<double>(r.u(17)) * 8.0;
    e.cic = static_cast<double>(r.s(18)) * kP2_31;
    e.omega0 = static_cast<double>(r.s(32)) * kP2_31 * kSemicircleToRad;
    e.cis = static_cast<double>(r.s(18)) * kP2_31;
    e.i0 = static_cast<double>(r.s(32)) * kP2_31 * kSemicircleToRad;
    e.crc = static_cast<double>(r.s(18)) * p2(-6);
    e.omega = static_cast<double>(r.s(32)) * kP2_31 * kSemicircleToRad;
    e.omegaDot = static_cast<double>(r.s(24)) * kP2_43 * kSemicircleToRad;
    e.tgd = static_cast<double>(r.s(10)) * 1e-10;       // TGD1, 0.1 ns
    e.tgd2 = static_cast<double>(r.s(10)) * 1e-10;      // TGD2
    e.health = static_cast<int>(r.u(1));

    if (!r.overrun()) {
        _emit(e);
    }
}

void RtcmNavDecoder::_decodeGalileo(const quint8 *payload, int payloadLen, bool inav)
{
    // 1045 F/NAV is 496 bits, 1046 I/NAV is 503; they share everything up to the broadcast group
    // delays, where I/NAV adds a second one and a wider health word.
    const int required = inav ? 503 : 496;
    if ((payloadLen * 8) < required) {
        return;
    }

    RtcmBitReader r(payload, payloadLen);
    RtcmEphemeris e;
    e.system = 'E';
    e.galileoInav = inav;
    r.skip(12);
    e.prn = static_cast<int>(r.u(6));
    e.week = static_cast<int>(r.u(12));
    e.iode = static_cast<int>(r.u(10));                 // IODnav
    e.uraIndex = static_cast<int>(r.u(8));              // SISA
    e.idot = static_cast<double>(r.s(14)) * kP2_43 * kSemicircleToRad;
    e.toc = static_cast<double>(r.u(14)) * 60.0;
    e.af2 = static_cast<double>(r.s(6)) * p2(-59);
    e.af1 = static_cast<double>(r.s(21)) * p2(-46);
    e.af0 = static_cast<double>(r.s(31)) * p2(-34);
    e.crs = static_cast<double>(r.s(16)) * kP2_5;
    e.deltaN = static_cast<double>(r.s(16)) * kP2_43 * kSemicircleToRad;
    e.m0 = static_cast<double>(r.s(32)) * kP2_31 * kSemicircleToRad;
    e.cuc = static_cast<double>(r.s(16)) * kP2_29;
    e.eccentricity = static_cast<double>(r.u(32)) * kP2_33;
    e.cus = static_cast<double>(r.s(16)) * kP2_29;
    e.sqrtA = static_cast<double>(r.u(32)) * kP2_19;
    e.toe = static_cast<double>(r.u(14)) * 60.0;
    e.cic = static_cast<double>(r.s(16)) * kP2_29;
    e.omega0 = static_cast<double>(r.s(32)) * kP2_31 * kSemicircleToRad;
    e.cis = static_cast<double>(r.s(16)) * kP2_29;
    e.i0 = static_cast<double>(r.s(32)) * kP2_31 * kSemicircleToRad;
    e.crc = static_cast<double>(r.s(16)) * kP2_5;
    e.omega = static_cast<double>(r.s(32)) * kP2_31 * kSemicircleToRad;
    e.omegaDot = static_cast<double>(r.s(24)) * kP2_43 * kSemicircleToRad;
    e.tgd = static_cast<double>(r.s(10)) * p2(-32);     // BGD E5a/E1
    if (inav) {
        e.tgd2 = static_cast<double>(r.s(10)) * p2(-32);// BGD E5b/E1
        const int e5bHealth = static_cast<int>(r.u(2));
        const int e5bValid = static_cast<int>(r.u(1));
        const int e1bHealth = static_cast<int>(r.u(2));
        const int e1bValid = static_cast<int>(r.u(1));
        e.health = e5bHealth | (e5bValid << 2) | (e1bHealth << 3) | (e1bValid << 5);
    } else {
        const int e5aHealth = static_cast<int>(r.u(2));
        const int e5aValid = static_cast<int>(r.u(1));
        e.health = e5aHealth | (e5aValid << 2);
    }

    // Galileo System Time is aligned to GPS time but its week count started 1024 weeks later.
    e.week += 1024;

    if (!r.overrun()) {
        _emit(e);
    }
}

void RtcmNavDecoder::_decodeGlonass(const quint8 *payload, int payloadLen)
{
    // 360 bits. Every numeric field here is sign-magnitude, not two's complement.
    if ((payloadLen * 8) < 360) {
        return;
    }

    RtcmBitReader r(payload, payloadLen);
    RtcmEphemeris e;
    e.system = 'R';
    r.skip(12);
    e.prn = static_cast<int>(r.u(6));                   // slot number
    e.frequencyChannel = static_cast<int>(r.u(5)) - 7;
    r.skip(1);                                          // almanac health
    r.skip(1);                                          // almanac health availability
    r.skip(2);                                          // P1
    e.frameTime = static_cast<int>(r.u(12));            // tk
    r.skip(1);                                          // Bn MSB
    r.skip(1);                                          // P2
    e.intervalIndex = static_cast<int>(r.u(7));         // tb, 15 minute units

    // Position/velocity/acceleration are interleaved per axis: velocity, position, acceleration.
    e.velocityX = static_cast<double>(r.signMagnitude(24)) * p2(-20) * 1000.0;
    e.positionX = static_cast<double>(r.signMagnitude(27)) * p2(-11) * 1000.0;
    e.accelerationX = static_cast<double>(r.signMagnitude(5)) * p2(-30) * 1000.0;
    e.velocityY = static_cast<double>(r.signMagnitude(24)) * p2(-20) * 1000.0;
    e.positionY = static_cast<double>(r.signMagnitude(27)) * p2(-11) * 1000.0;
    e.accelerationY = static_cast<double>(r.signMagnitude(5)) * p2(-30) * 1000.0;
    e.velocityZ = static_cast<double>(r.signMagnitude(24)) * p2(-20) * 1000.0;
    e.positionZ = static_cast<double>(r.signMagnitude(27)) * p2(-11) * 1000.0;
    e.accelerationZ = static_cast<double>(r.signMagnitude(5)) * p2(-30) * 1000.0;

    r.skip(1);                                          // P3
    e.gammaN = static_cast<double>(r.signMagnitude(11)) * p2(-40);
    r.skip(2);                                          // P
    r.skip(1);                                          // ln (third string)
    e.tauN = static_cast<double>(r.signMagnitude(22)) * p2(-30);
    r.skip(5);                                          // delta tau n
    e.ageOfData = static_cast<int>(r.u(5));             // En
    r.skip(1);                                          // P4
    r.skip(4);                                          // Ft
    e.dayNumber = static_cast<int>(r.u(11));            // Nt
    r.skip(2);                                          // M
    r.skip(1);                                          // additional data availability
    r.skip(11);                                         // NA
    r.skip(32);                                         // tau c
    e.fourYearInterval = static_cast<int>(r.u(5));      // N4

    if (!r.overrun()) {
        _emit(e);
    }
}

void RtcmNavDecoder::_emit(const RtcmEphemeris &ephemeris)
{
    ++_decodedCount;
    emit ephemerisReady(ephemeris);
}
