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
#include <QtCore/QString>
#include <QtCore/QtGlobal>

/// MSB-first bit reader over an RTCM3 payload. Reads past the end return 0 and latch overrun(),
/// so a truncated message fails loudly instead of decoding garbage.
class RtcmBitReader
{
public:
    RtcmBitReader(const quint8 *data, int lengthBytes)
        : _data(data)
        , _bits(lengthBytes * 8)
    {
    }

    bool overrun() const { return _overrun; }
    int position() const { return _pos; }
    int remaining() const { return _bits - _pos; }

    quint64 u(int n)
    {
        if ((n <= 0) || ((_pos + n) > _bits)) {
            _overrun = (n > 0);
            return 0;
        }
        quint64 value = 0;
        for (int i = 0; i < n; ++i) {
            const int bit = _pos + i;
            value = (value << 1) | ((_data[bit / 8] >> (7 - (bit % 8))) & 0x01U);
        }
        _pos += n;
        return value;
    }

    /// Two's-complement signed field.
    qint64 s(int n)
    {
        const quint64 raw = u(n);
        if ((n > 0) && (raw & (1ULL << (n - 1)))) {
            return static_cast<qint64>(raw) - static_cast<qint64>(1ULL << n);
        }
        return static_cast<qint64>(raw);
    }

    /// Sign-magnitude field: GLONASS encodes its ephemeris this way, not in two's complement.
    /// Decoding one as the other yields plausible magnitudes with wrong signs.
    qint64 signMagnitude(int n)
    {
        const quint64 raw = u(n);
        if (n <= 1) {
            return static_cast<qint64>(raw);
        }
        const quint64 magnitude = raw & ((1ULL << (n - 1)) - 1);
        return (raw & (1ULL << (n - 1))) ? -static_cast<qint64>(magnitude)
                                         : static_cast<qint64>(magnitude);
    }

    void skip(int n) { (void) u(n); }

    /// Length-prefixed ASCII descriptor: an 8 bit byte count followed by that many characters.
    QString descriptor()
    {
        const int length = static_cast<int>(u(8));
        QByteArray text;
        text.reserve(length);
        for (int i = 0; i < length; ++i) {
            text.append(static_cast<char>(u(8)));
        }
        return QString::fromLatin1(text).trimmed();
    }

private:
    const quint8 *_data;
    int _bits;
    int _pos = 0;
    bool _overrun = false;
};
