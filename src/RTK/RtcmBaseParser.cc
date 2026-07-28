/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "RtcmBaseParser.h"
#include "QGCLoggingCategory.h"

#include <QtCore/QtMath>

#include <cmath>

QGC_LOGGING_CATEGORY(RtcmBaseParserLog, "qgc.rtk.rtcmbaseparser")

namespace {

constexpr quint8 kRtcmPreamble = 0xD3;

// CRC-24Q (Qualcomm) as used by RTCM3. Polynomial 0x1864CFB, init 0.
quint32 crc24q(const quint8 *data, int len)
{
    quint32 crc = 0;
    for (int i = 0; i < len; ++i) {
        crc ^= static_cast<quint32>(data[i]) << 16;
        for (int b = 0; b < 8; ++b) {
            crc <<= 1;
            if (crc & 0x1000000U) {
                crc ^= 0x1864CFBU;
            }
        }
    }
    return crc & 0xFFFFFFU;
}

// Read numBits (MSB-first) starting at bit offset startBit from buffer.
quint64 getBits(const quint8 *buffer, int startBit, int numBits)
{
    quint64 value = 0;
    for (int i = 0; i < numBits; ++i) {
        const int bit = startBit + i;
        const quint8 byte = buffer[bit / 8];
        const int shift = 7 - (bit % 8);
        value = (value << 1) | ((byte >> shift) & 0x01U);
    }
    return value;
}

// Read a signed two's-complement field.
qint64 getBitsSigned(const quint8 *buffer, int startBit, int numBits)
{
    const quint64 raw = getBits(buffer, startBit, numBits);
    const quint64 signBit = 1ULL << (numBits - 1);
    if (raw & signBit) {
        return static_cast<qint64>(raw) - static_cast<qint64>(1ULL << numBits);
    }
    return static_cast<qint64>(raw);
}

} // namespace

RtcmBaseParser::RtcmBaseParser(QObject *parent)
    : QObject(parent)
{
}

RtcmBaseParser::~RtcmBaseParser()
{
}

void RtcmBaseParser::reset()
{
    _buffer.clear();
}

void RtcmBaseParser::addData(const QByteArray &data)
{
    _buffer += data;
    _parseFrames();

    // Safety valve against unbounded growth on a garbage stream.
    if (_buffer.size() > kMaxBufferBytes) {
        _buffer = _buffer.right(kMaxBufferBytes / 2);
    }
}

void RtcmBaseParser::_parseFrames()
{
    while (true) {
        // Find frame preamble.
        const int start = _buffer.indexOf(static_cast<char>(kRtcmPreamble));
        if (start < 0) {
            _buffer.clear();
            return;
        }
        if (start > 0) {
            _buffer.remove(0, start);
        }
        if (_buffer.size() < 3) {
            return; // need header
        }

        const quint8 *buf = reinterpret_cast<const quint8 *>(_buffer.constData());
        const int payloadLen = (static_cast<int>(buf[1] & 0x03) << 8) | static_cast<int>(buf[2]);
        const int frameLen = 3 + payloadLen + 3; // header + payload + CRC24

        if (_buffer.size() < frameLen) {
            return; // wait for the rest of the frame
        }

        const quint32 computed = crc24q(buf, 3 + payloadLen);
        const quint32 received = (static_cast<quint32>(buf[3 + payloadLen]) << 16) |
                                 (static_cast<quint32>(buf[3 + payloadLen + 1]) << 8) |
                                 static_cast<quint32>(buf[3 + payloadLen + 2]);

        if (computed != received) {
            // False preamble or corruption - skip one byte and resync.
            _buffer.remove(0, 1);
            continue;
        }

        const quint8 *payload = buf + 3;
        const int messageNumber = static_cast<int>(getBits(payload, 0, 12));

        if ((messageNumber == 1005) || (messageNumber == 1006)) {
            const int stationId = static_cast<int>(getBits(payload, 12, 12));
            // ECEF X/Y/Z are 38-bit signed, resolution 0.0001 m.
            const qint64 xRaw = getBitsSigned(payload, 34, 38);
            const qint64 yRaw = getBitsSigned(payload, 74, 38);
            const qint64 zRaw = getBitsSigned(payload, 114, 38);

            const double x = static_cast<double>(xRaw) * 0.0001;
            const double y = static_cast<double>(yRaw) * 0.0001;
            const double z = static_cast<double>(zRaw) * 0.0001;

            double lat = 0.0;
            double lon = 0.0;
            double alt = 0.0;
            _ecefToGeodetic(x, y, z, lat, lon, alt);

            qCDebug(RtcmBaseParserLog) << "Base station" << messageNumber << "id" << stationId
                                       << "lat" << lat << "lon" << lon << "alt" << alt;
            emit basePositionUpdate(lat, lon, alt, stationId);
        }

        _buffer.remove(0, frameLen);
    }
}

void RtcmBaseParser::_ecefToGeodetic(double x, double y, double z, double &latDeg, double &lonDeg, double &alt)
{
    // WGS84
    static constexpr double a = 6378137.0;
    static constexpr double f = 1.0 / 298.257223563;
    const double b = a * (1.0 - f);
    const double e2 = (a * a - b * b) / (a * a);
    const double ep2 = (a * a - b * b) / (b * b);

    const double p = std::sqrt((x * x) + (y * y));
    lonDeg = qRadiansToDegrees(std::atan2(y, x));

    if (p < 1e-6) {
        // Polar case.
        latDeg = (z >= 0.0) ? 90.0 : -90.0;
        alt = std::abs(z) - b;
        return;
    }

    // Bowring's method.
    const double theta = std::atan2(z * a, p * b);
    const double sinTheta = std::sin(theta);
    const double cosTheta = std::cos(theta);
    const double lat = std::atan2(z + ep2 * b * sinTheta * sinTheta * sinTheta,
                                  p - e2 * a * cosTheta * cosTheta * cosTheta);
    const double sinLat = std::sin(lat);
    const double N = a / std::sqrt(1.0 - e2 * sinLat * sinLat);

    latDeg = qRadiansToDegrees(lat);
    alt = (p / std::cos(lat)) - N;
}
