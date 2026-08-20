/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "RtcmObsDecoder.h"
#include "QGCLoggingCategory.h"

#include <algorithm>
#include <climits>
#include <cmath>

QGC_LOGGING_CATEGORY(RtcmObsDecoderLog, "qgc.rtk.rtcmobsdecoder")

namespace {

constexpr double kSpeedOfLight = 299792458.0;
/// Light travels this far in one millisecond - MSM ranges are expressed in milliseconds.
constexpr double kRangePerMs = kSpeedOfLight / 1000.0;

// Carrier frequencies (Hz).
constexpr double kFreqL1 = 1575.42e6;   // GPS/QZSS L1, Galileo E1, BeiDou B1C
constexpr double kFreqL2 = 1227.60e6;   // GPS/QZSS L2
constexpr double kFreqL5 = 1176.45e6;   // GPS/QZSS L5, Galileo E5a, BeiDou B2a
constexpr double kFreqE5b = 1207.140e6; // Galileo E5b, BeiDou B2I
constexpr double kFreqE5ab = 1191.795e6;// Galileo E5 AltBOC
constexpr double kFreqE6 = 1278.75e6;   // Galileo E6
constexpr double kFreqB1I = 1561.098e6; // BeiDou B1I
constexpr double kFreqB3I = 1268.52e6;  // BeiDou B3I
constexpr double kFreqG3 = 1202.025e6;  // GLONASS L3 (CDMA)

/// Bit reader over an RTCM payload, MSB first.
class BitReader
{
public:
    BitReader(const quint8 *data, int lenBytes) : _data(data), _bits(lenBytes * 8) {}

    bool overrun() const { return _overrun; }
    int position() const { return _pos; }
    int remaining() const { return _bits - _pos; }

    quint64 u(int n)
    {
        if ((_pos + n) > _bits) {
            _overrun = true;
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

    qint64 s(int n)
    {
        const quint64 raw = u(n);
        if (raw & (1ULL << (n - 1))) {
            return static_cast<qint64>(raw) - static_cast<qint64>(1ULL << n);
        }
        return static_cast<qint64>(raw);
    }

    void skip(int n) { (void) u(n); }

private:
    const quint8 *_data;
    int _bits;
    int _pos = 0;
    bool _overrun = false;
};

// RTCM signal id (1-based) -> RINEX 3 observation code suffix. Empty = reserved/unknown.
const char *const kSigGps[32] = {
    "",   "1C", "1P", "1W", "",   "",   "",   "2C", "2P", "2W", "",   "",
    "",   "",   "2S", "2L", "2X", "",   "",   "",   "",   "5I", "5Q", "5X",
    "",   "",   "",   "",   "",   "1S", "1L", "1X"
};
const char *const kSigGlo[32] = {
    "",   "1C", "1P", "",   "",   "",   "",   "2C", "2P", "",   "3I", "3Q",
    "3X", "",   "",   "",   "",   "",   "",   "",   "",   "",   "",   "",
    "",   "",   "",   "",   "",   "",   "",   ""
};
const char *const kSigGal[32] = {
    "",   "1C", "1A", "1B", "1X", "1Z", "",   "6C", "6A", "6B", "6X", "6Z",
    "",   "7I", "7Q", "7X", "",   "8I", "8Q", "8X", "",   "5I", "5Q", "5X",
    "",   "",   "",   "",   "",   "",   "",   ""
};
const char *const kSigQzs[32] = {
    "",   "1C", "",   "",   "",   "",   "",   "",   "6S", "6L", "6X", "",
    "",   "",   "2S", "2L", "2X", "",   "",   "",   "",   "5I", "5Q", "5X",
    "",   "",   "",   "",   "",   "1S", "1L", "1X"
};
const char *const kSigSbs[32] = {
    "",   "1C", "",   "",   "",   "",   "",   "",   "",   "",   "",   "",
    "",   "",   "",   "",   "",   "",   "",   "",   "",   "5I", "5Q", "5X",
    "",   "",   "",   "",   "",   "",   "",   ""
};
const char *const kSigBds[32] = {
    "",   "2I", "2Q", "2X", "",   "",   "",   "6I", "6Q", "6X", "",   "",
    "",   "7I", "7Q", "7X", "",   "",   "",   "",   "",   "5D", "5P", "5X",
    "",   "",   "",   "",   "",   "",   "",   ""
};

struct MsmSystem {
    char letter;
    const char *const *signalTable;
    int prnOffset;
};

/// Maps the MSM message-number decade to its constellation. Returns false for non-MSM messages.
bool msmSystem(int messageNumber, MsmSystem &out, int &msmType)
{
    const int base = (messageNumber / 10) * 10;
    msmType = messageNumber - base;
    if ((msmType < 1) || (msmType > 7)) {
        return false;
    }
    switch (base) {
    case 1070: out = { 'G', kSigGps, 0 };   return true;
    case 1080: out = { 'R', kSigGlo, 0 };   return true;
    case 1090: out = { 'E', kSigGal, 0 };   return true;
    case 1100: out = { 'S', kSigSbs, 119 }; return true;  // SBAS PRN = slot + 119
    case 1110: out = { 'J', kSigQzs, 192 }; return true;  // QZSS PRN = slot + 192
    case 1120: out = { 'C', kSigBds, 0 };   return true;
    default:   return false;
    }
}

/// Carrier frequency for a RINEX code suffix, or 0 if it cannot be resolved. GLONASS FDMA needs
/// the satellite's frequency channel number; pass INT_MIN when it is unknown.
double carrierFrequency(char system, const QString &code, int glonassChannel)
{
    if (code.isEmpty()) {
        return 0.0;
    }
    const QChar band = code.at(0);

    switch (system) {
    case 'G':
    case 'J':
        if (band == '1') return kFreqL1;
        if (band == '2') return kFreqL2;
        if (band == '5') return kFreqL5;
        return 0.0;
    case 'E':
        if (band == '1') return kFreqL1;
        if (band == '5') return kFreqL5;
        if (band == '7') return kFreqE5b;
        if (band == '8') return kFreqE5ab;
        if (band == '6') return kFreqE6;
        return 0.0;
    case 'C':
        if (band == '2') return kFreqB1I;
        if (band == '1') return kFreqL1;
        if (band == '5') return kFreqL5;
        if (band == '7') return kFreqE5b;
        if (band == '6') return kFreqB3I;
        return 0.0;
    case 'S':
        if (band == '1') return kFreqL1;
        if (band == '5') return kFreqL5;
        return 0.0;
    case 'R':
        if (band == '3') return kFreqG3;
        if (glonassChannel == INT_MIN) return 0.0;
        if (band == '1') return 1602.0e6 + (glonassChannel * 0.5625e6);
        if (band == '2') return 1246.0e6 + (glonassChannel * 0.4375e6);
        return 0.0;
    default:
        return 0.0;
    }
}

/// Reads a length-prefixed descriptor string (RTCM stores them as a byte count then ASCII).
QString readDescriptor(BitReader &reader)
{
    const int length = static_cast<int>(reader.u(8));
    QByteArray text;
    text.reserve(length);
    for (int i = 0; i < length; ++i) {
        text.append(static_cast<char>(reader.u(8)));
    }
    return QString::fromLatin1(text).trimmed();
}

} // namespace

RtcmObsDecoder::RtcmObsDecoder(QObject *parent)
    : QObject(parent)
{
}

RtcmObsDecoder::~RtcmObsDecoder()
{
}

void RtcmObsDecoder::reset()
{
    _station = RtcmStationInfo();
    _glonassChannels.clear();
    _epoch = RtcmObsEpoch();
    _epochOpen = false;
    _decodedEpochs = 0;
    _unsupportedMsm = 0;
}

void RtcmObsDecoder::flush()
{
    _emitEpoch();
}

void RtcmObsDecoder::addFrames(const QByteArray &frames)
{
    // The parser only ever hands us complete, CRC-valid frames, so walking them by length is safe.
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

void RtcmObsDecoder::_decodeFrame(const quint8 *payload, int payloadLen)
{
    if (payloadLen < 2) {
        return;
    }

    const int messageNumber = (static_cast<int>(payload[0]) << 4) | (static_cast<int>(payload[1]) >> 4);

    switch (messageNumber) {
    case 1005:
    case 1006:
        _decodeStationArp(payload, payloadLen, messageNumber);
        return;
    case 1007:
    case 1008:
    case 1033:
        _decodeAntennaDescriptor(payload, payloadLen, messageNumber);
        return;
    default:
        break;
    }

    MsmSystem system;
    int msmType = 0;
    if (msmSystem(messageNumber, system, msmType)) {
        _decodeMsm(payload, payloadLen, messageNumber);
    }
}

void RtcmObsDecoder::_decodeStationArp(const quint8 *payload, int payloadLen, int messageNumber)
{
    // 1005 is 19 payload bytes, 1006 adds a 16 bit antenna height.
    if (payloadLen < 19) {
        return;
    }

    BitReader reader(payload, payloadLen);
    reader.skip(12);                                    // message number
    _station.stationId = static_cast<int>(reader.u(12));
    reader.skip(6);                                     // ITRF year + indicator bits
    reader.skip(4);                                     // GPS/GLONASS/Galileo indicators + ref station flag
    const qint64 x = reader.s(38);
    reader.skip(2);                                     // single receiver oscillator + reserved
    const qint64 y = reader.s(38);
    reader.skip(2);                                     // quarter cycle indicator
    const qint64 z = reader.s(38);

    if (reader.overrun()) {
        return;
    }

    _station.ecefX = static_cast<double>(x) * 0.0001;
    _station.ecefY = static_cast<double>(y) * 0.0001;
    _station.ecefZ = static_cast<double>(z) * 0.0001;
    _station.havePosition = true;

    if ((messageNumber == 1006) && (payloadLen >= 21)) {
        _station.antennaHeight = static_cast<double>(reader.u(16)) * 0.0001;
    }

    emit stationInfoChanged(_station);
}

void RtcmObsDecoder::_decodeAntennaDescriptor(const quint8 *payload, int payloadLen, int messageNumber)
{
    BitReader reader(payload, payloadLen);
    reader.skip(12);                                    // message number
    _station.stationId = static_cast<int>(reader.u(12));

    const QString antenna = readDescriptor(reader);
    if (!antenna.isEmpty()) {
        _station.antennaDescriptor = antenna;
    }

    if ((messageNumber == 1008) || (messageNumber == 1033)) {
        reader.skip(8);                                 // antenna setup id
        const QString serial = readDescriptor(reader);
        if (!serial.isEmpty()) {
            _station.antennaSerial = serial;
        }
    }

    if (messageNumber == 1033) {
        const QString receiver = readDescriptor(reader);
        const QString firmware = readDescriptor(reader);
        const QString receiverSerial = readDescriptor(reader);
        if (!receiver.isEmpty()) {
            _station.receiverType = receiver;
        }
        if (!firmware.isEmpty()) {
            _station.receiverVersion = firmware;
        }
        if (!receiverSerial.isEmpty()) {
            _station.receiverSerial = receiverSerial;
        }
    }

    if (!reader.overrun()) {
        emit stationInfoChanged(_station);
    }
}

void RtcmObsDecoder::_decodeMsm(const quint8 *payload, int payloadLen, int messageNumber)
{
    MsmSystem system;
    int msmType = 0;
    if (!msmSystem(messageNumber, system, msmType)) {
        return;
    }

    // MSM1-3 carry no rough range integer and cannot produce a usable pseudorange here.
    if (msmType < 4) {
        if ((_unsupportedMsm++ % 100) == 0) {
            qCWarning(RtcmObsDecoderLog) << "Ignoring MSM" << msmType << "message" << messageNumber
                                         << "- only MSM4/5/6/7 carry full observables";
        }
        return;
    }

    BitReader reader(payload, payloadLen);
    reader.skip(12);                                    // message number
    reader.skip(12);                                    // reference station id

    qint64 epochMs = 0;
    if (system.letter == 'R') {
        // GLONASS sends day-of-week + time-of-day in Moscow time rather than a week-based TOW.
        const qint64 day = static_cast<qint64>(reader.u(3));
        const qint64 todMs = static_cast<qint64>(reader.u(27));
        epochMs = (day * 86400000) + todMs;
    } else {
        epochMs = static_cast<qint64>(reader.u(30));
    }

    const bool multipleMessage = (reader.u(1) != 0);
    reader.skip(3);                                     // IODS
    reader.skip(7);                                     // reserved
    reader.skip(2);                                     // clock steering indicator
    reader.skip(2);                                     // external clock indicator
    reader.skip(1);                                     // divergence-free smoothing indicator
    reader.skip(3);                                     // smoothing interval

    const quint64 satMask = reader.u(64);
    const quint64 sigMask = reader.u(32);
    if (reader.overrun()) {
        return;
    }

    QList<int> satSlots;
    for (int i = 0; i < 64; ++i) {
        if ((satMask >> (63 - i)) & 0x01ULL) {
            satSlots.append(i + 1);
        }
    }
    QList<int> signalIds;
    for (int i = 0; i < 32; ++i) {
        if ((sigMask >> (31 - i)) & 0x01ULL) {
            signalIds.append(i + 1);
        }
    }

    const int satCount = satSlots.size();
    const int sigCount = signalIds.size();
    if ((satCount == 0) || (sigCount == 0) || ((satCount * sigCount) > 64)) {
        return; // the cell mask is capped at 64 bits by the standard
    }

    const int cellBits = satCount * sigCount;
    QList<bool> cells;
    cells.reserve(cellBits);
    const quint64 cellMask = reader.u(cellBits);
    for (int i = 0; i < cellBits; ++i) {
        cells.append(((cellMask >> (cellBits - 1 - i)) & 0x01ULL) != 0);
    }

    const bool hasExtendedSatInfo = ((msmType == 5) || (msmType == 7));
    const bool hasRate = hasExtendedSatInfo;
    const bool extendedSignals = ((msmType == 6) || (msmType == 7));

    // --- satellite blocks ---
    QList<int> roughMs;
    QList<int> extInfo;
    QList<int> roughModMs;
    QList<qint64> roughRate;
    roughMs.reserve(satCount);
    for (int i = 0; i < satCount; ++i) {
        roughMs.append(static_cast<int>(reader.u(8)));
    }
    if (hasExtendedSatInfo) {
        extInfo.reserve(satCount);
        for (int i = 0; i < satCount; ++i) {
            extInfo.append(static_cast<int>(reader.u(4)));
        }
    }
    roughModMs.reserve(satCount);
    for (int i = 0; i < satCount; ++i) {
        roughModMs.append(static_cast<int>(reader.u(10)));
    }
    if (hasRate) {
        roughRate.reserve(satCount);
        for (int i = 0; i < satCount; ++i) {
            roughRate.append(reader.s(14));
        }
    }

    // --- signal blocks ---
    const int cellCount = static_cast<int>(std::count(cells.cbegin(), cells.cend(), true));
    const int fineRangeBits = extendedSignals ? 20 : 15;
    const int finePhaseBits = extendedSignals ? 24 : 22;
    const int lockBits = extendedSignals ? 10 : 4;
    const int cnrBits = extendedSignals ? 10 : 6;
    const double fineRangeScale = extendedSignals ? std::pow(2.0, -29) : std::pow(2.0, -24);
    const double finePhaseScale = extendedSignals ? std::pow(2.0, -31) : std::pow(2.0, -29);
    const double cnrScale = extendedSignals ? std::pow(2.0, -4) : 1.0;
    const qint64 fineRangeInvalid = extendedSignals ? -(1LL << 19) : -(1LL << 14);
    const qint64 finePhaseInvalid = extendedSignals ? -(1LL << 23) : -(1LL << 21);

    QList<qint64> fineRange;
    QList<qint64> finePhase;
    QList<int> halfCycle;
    QList<int> cnr;
    QList<qint64> fineRate;
    fineRange.reserve(cellCount);
    for (int i = 0; i < cellCount; ++i) {
        fineRange.append(reader.s(fineRangeBits));
    }
    finePhase.reserve(cellCount);
    for (int i = 0; i < cellCount; ++i) {
        finePhase.append(reader.s(finePhaseBits));
    }
    for (int i = 0; i < cellCount; ++i) {
        reader.skip(lockBits);                          // lock time indicator
    }
    halfCycle.reserve(cellCount);
    for (int i = 0; i < cellCount; ++i) {
        halfCycle.append(static_cast<int>(reader.u(1)));
    }
    cnr.reserve(cellCount);
    for (int i = 0; i < cellCount; ++i) {
        cnr.append(static_cast<int>(reader.u(cnrBits)));
    }
    if (hasRate) {
        fineRate.reserve(cellCount);
        for (int i = 0; i < cellCount; ++i) {
            fineRate.append(reader.s(15));
        }
    }

    if (reader.overrun()) {
        qCDebug(RtcmObsDecoderLog) << "Truncated MSM message" << messageNumber;
        return;
    }

    // --- assemble ---
    // Every constellation reports the same instant; normalise to GPS time of week so the epochs
    // line up. GLONASS is Moscow-time day/ms and carries no week, so it joins the open epoch.
    qint64 gpsTowMs = -1;
    if (system.letter == 'C') {
        gpsTowMs = (epochMs + kBeidouGpsOffsetMs) % kMsPerWeek;
    } else if (system.letter != 'R') {
        gpsTowMs = epochMs;
    }

    if (gpsTowMs >= 0) {
        if (_epochOpen && (_epoch.gpsTowMs != gpsTowMs)) {
            _emitEpoch();
        }
        if (!_epochOpen) {
            _startEpoch(gpsTowMs);
        }
    } else if (!_epochOpen) {
        return; // GLONASS arrived before any GPS-timed message; nothing to attach it to
    }

    int cellIndex = 0;
    for (int si = 0; si < satCount; ++si) {
        int glonassChannel = INT_MIN;
        if (system.letter == 'R') {
            if (hasExtendedSatInfo && (extInfo.value(si) <= 13)) {
                glonassChannel = extInfo.at(si) - 7;
                _glonassChannels.insert(satSlots.at(si), glonassChannel);
            } else {
                glonassChannel = _glonassChannels.value(satSlots.at(si), INT_MIN);
            }
        }

        RtcmSatObservations satObs;
        satObs.system = system.letter;
        satObs.prn = satSlots.at(si) + system.prnOffset;

        for (int gi = 0; gi < sigCount; ++gi) {
            if (!cells.at((si * sigCount) + gi)) {
                continue;
            }
            const int cell = cellIndex++;

            const QString code = QString::fromLatin1(system.signalTable[signalIds.at(gi) - 1]);
            if (code.isEmpty()) {
                continue;   // reserved signal id - no RINEX equivalent
            }
            if (roughMs.at(si) == 255) {
                continue;   // invalid rough range: nothing usable for this satellite
            }

            const double roughRangeM =
                (static_cast<double>(roughMs.at(si)) + (static_cast<double>(roughModMs.at(si)) * std::pow(2.0, -10)))
                * kRangePerMs;
            const double frequency = carrierFrequency(system.letter, code, glonassChannel);

            RtcmObservation obs;
            if (fineRange.at(cell) != fineRangeInvalid) {
                obs.pseudorange = roughRangeM + (static_cast<double>(fineRange.at(cell)) * fineRangeScale * kRangePerMs);
                obs.havePseudorange = true;
            }
            if ((finePhase.at(cell) != finePhaseInvalid) && (frequency > 0.0)) {
                const double phaseRangeM =
                    roughRangeM + (static_cast<double>(finePhase.at(cell)) * finePhaseScale * kRangePerMs);
                obs.carrierPhase = phaseRangeM * frequency / kSpeedOfLight;
                obs.haveCarrierPhase = true;
            }
            if (hasRate && (frequency > 0.0) && (roughRate.value(si) != -8192)) {
                double rate = static_cast<double>(roughRate.at(si));
                if (fineRate.value(cell) != -16384) {
                    rate += static_cast<double>(fineRate.at(cell)) * 0.0001;
                }
                // RINEX Doppler is positive for a closing range; the MSM rate is range rate.
                obs.doppler = -rate * frequency / kSpeedOfLight;
                obs.haveDoppler = true;
            }
            if (cnr.at(cell) != 0) {
                obs.snr = static_cast<double>(cnr.at(cell)) * cnrScale;
                obs.haveSnr = true;
            }
            obs.lli = halfCycle.at(cell) ? 1 : 0;

            if (obs.havePseudorange || obs.haveCarrierPhase) {
                satObs.observations.insert(code, obs);
            }
        }

        if (!satObs.observations.isEmpty()) {
            _epoch.satellites.append(satObs);
        }
    }

    // DF393 clear means this was the last message of the epoch.
    if (!multipleMessage) {
        _emitEpoch();
    }
}

void RtcmObsDecoder::_startEpoch(qint64 gpsTowMs)
{
    _epoch = RtcmObsEpoch();
    _epoch.gpsTowMs = gpsTowMs;
    _epochOpen = true;
}

void RtcmObsDecoder::_emitEpoch()
{
    if (!_epochOpen) {
        return;
    }

    _epochOpen = false;
    if (_epoch.satellites.isEmpty()) {
        return;
    }

    ++_decodedEpochs;
    emit epochReady(_epoch);
    _epoch = RtcmObsEpoch();
}
