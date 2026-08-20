/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "RtcmRinexTest.h"
#include "RinexObsWriter.h"
#include "RinexNavWriter.h"
#include "RtcmNavDecoder.h"
#include "RtcmObsDecoder.h"
#include "RTCMFileLogger.h"
#include "RtcmStreamParser.h"

#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QTemporaryDir>
#include <QtTest/QSignalSpy>
#include <QtTest/QTest>

#include <cmath>

namespace {

constexpr double kSpeedOfLight = 299792458.0;
constexpr double kRangePerMs = kSpeedOfLight / 1000.0;
constexpr double kFreqL1 = 1575.42e6;
constexpr double kFreqL2 = 1227.60e6;

quint32 crc24q(const QByteArray &data)
{
    quint32 crc = 0;
    for (const char c : data) {
        crc ^= static_cast<quint32>(static_cast<quint8>(c)) << 16;
        for (int i = 0; i < 8; ++i) {
            crc <<= 1;
            if (crc & 0x1000000U) {
                crc ^= 0x1864CFBU;
            }
        }
    }
    return crc & 0xFFFFFFU;
}

/// Builds an RTCM payload bit by bit, MSB first.
class BitWriter
{
public:
    void u(int bits, quint64 value)
    {
        for (int i = bits - 1; i >= 0; --i) {
            _put((value >> i) & 0x01ULL);
        }
    }

    void s(int bits, qint64 value)
    {
        u(bits, static_cast<quint64>(value) & ((bits >= 64) ? ~0ULL : ((1ULL << bits) - 1)));
    }

    /// Pads to a byte boundary and returns the payload.
    QByteArray payload() const { return _bytes; }

private:
    void _put(quint64 bit)
    {
        if (_bitCount % 8 == 0) {
            _bytes.append('\0');
        }
        if (bit) {
            const int index = _bitCount / 8;
            _bytes[index] = static_cast<char>(static_cast<quint8>(_bytes.at(index)) | (0x80U >> (_bitCount % 8)));
        }
        ++_bitCount;
    }

    QByteArray _bytes;
    int _bitCount = 0;
};

/// Wraps a payload in the RTCM3 preamble/length/CRC framing.
QByteArray frame(const QByteArray &payload)
{
    QByteArray out;
    out.append(static_cast<char>(0xD3));
    out.append(static_cast<char>((payload.size() >> 8) & 0x03));
    out.append(static_cast<char>(payload.size() & 0xFF));
    out.append(payload);
    const quint32 crc = crc24q(out);
    out.append(static_cast<char>((crc >> 16) & 0xFF));
    out.append(static_cast<char>((crc >> 8) & 0xFF));
    out.append(static_cast<char>(crc & 0xFF));
    return out;
}

struct SatelliteFields {
    int slot = 0;
    int roughMs = 0;
    int roughMod = 0;
    int extInfo = 0;
    int roughRate = 0;
};

struct SignalFields {
    int finePseudorange = 0;
    int finePhase = 0;
    int fineRate = 0;
    int cnr = 0;
};

/// Encodes an MSM5 message. Every satellite carries every signal (a full cell mask).
QByteArray buildMsm5(int messageNumber,
                     quint32 epochMs,
                     bool multipleMessage,
                     const QList<SatelliteFields> &satellites,
                     const QList<int> &signalIds,
                     const QList<SignalFields> &cells)
{
    BitWriter w;
    w.u(12, static_cast<quint64>(messageNumber));
    w.u(12, 1234);                       // reference station id
    w.u(30, epochMs);
    w.u(1, multipleMessage ? 1 : 0);
    w.u(3, 0);                           // IODS
    w.u(7, 0);                           // reserved
    w.u(2, 0);                           // clock steering
    w.u(2, 0);                           // external clock
    w.u(1, 0);                           // divergence-free smoothing
    w.u(3, 0);                           // smoothing interval

    quint64 satMask = 0;
    for (const SatelliteFields &sat : satellites) {
        satMask |= (1ULL << (64 - sat.slot));
    }
    w.u(64, satMask);

    quint32 sigMask = 0;
    for (const int id : signalIds) {
        sigMask |= (1U << (32 - id));
    }
    w.u(32, sigMask);

    // Full cell mask: every listed satellite carries every listed signal.
    const int cellBits = satellites.size() * signalIds.size();
    for (int i = 0; i < cellBits; ++i) {
        w.u(1, 1);
    }

    for (const SatelliteFields &sat : satellites) {
        w.u(8, static_cast<quint64>(sat.roughMs));
    }
    for (const SatelliteFields &sat : satellites) {
        w.u(4, static_cast<quint64>(sat.extInfo));
    }
    for (const SatelliteFields &sat : satellites) {
        w.u(10, static_cast<quint64>(sat.roughMod));
    }
    for (const SatelliteFields &sat : satellites) {
        w.s(14, sat.roughRate);
    }

    for (const SignalFields &cell : cells) {
        w.s(15, cell.finePseudorange);
    }
    for (const SignalFields &cell : cells) {
        w.s(22, cell.finePhase);
    }
    for (int i = 0; i < cells.size(); ++i) {
        w.u(4, 15);                      // lock time indicator
    }
    for (int i = 0; i < cells.size(); ++i) {
        w.u(1, 0);                       // half-cycle ambiguity
    }
    for (const SignalFields &cell : cells) {
        w.u(6, static_cast<quint64>(cell.cnr));
    }
    for (const SignalFields &cell : cells) {
        w.s(15, cell.fineRate);
    }

    return frame(w.payload());
}

/// Reference reconstruction of what the decoder should produce, straight from the RTCM scaling.
double expectedPseudorange(const SatelliteFields &sat, const SignalFields &cell)
{
    const double rough = (sat.roughMs + (sat.roughMod * std::pow(2.0, -10))) * kRangePerMs;
    return rough + (cell.finePseudorange * std::pow(2.0, -24) * kRangePerMs);
}

double expectedCarrierPhase(const SatelliteFields &sat, const SignalFields &cell, double frequency)
{
    const double rough = (sat.roughMs + (sat.roughMod * std::pow(2.0, -10))) * kRangePerMs;
    const double phaseM = rough + (cell.finePhase * std::pow(2.0, -29) * kRangePerMs);
    return phaseM * frequency / kSpeedOfLight;
}

double expectedDoppler(const SatelliteFields &sat, const SignalFields &cell, double frequency)
{
    const double rate = sat.roughRate + (cell.fineRate * 0.0001);
    return -rate * frequency / kSpeedOfLight;
}

const RtcmSatObservations *findSat(const RtcmObsEpoch &epoch, char system, int prn)
{
    for (const RtcmSatObservations &sat : epoch.satellites) {
        if ((sat.system == system) && (sat.prn == prn)) {
            return &sat;
        }
    }
    return nullptr;
}

} // namespace

void RtcmRinexTest::_testParserRejectsNonRtcm()
{
    RtcmStreamParser parser;
    QSignalSpy spy(&parser, &RtcmStreamParser::rtcmFrames);

    const QByteArray garbage =
        QByteArrayLiteral("HTTP/1.1 401 Unauthorized\r\n\r\n<html>Unauthorized</html>\r\n")
        + QByteArrayLiteral("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n");

    parser.addData(garbage);

    QCOMPARE(spy.count(), 0);
    QCOMPARE(parser.validFrames(), 0ULL);
    // Everything either got discarded or is still buffered waiting for more input.
    QVERIFY(parser.discardedBytes() > 0);
    QVERIFY(parser.discardedBytes() <= static_cast<quint64>(garbage.size()));
}

void RtcmRinexTest::_testParserReassemblesSplitFrames()
{
    const QList<SatelliteFields> sats = { { 5, 80, 512, 0, -100 } };
    const QList<SignalFields> cells = { { 1000, 2000, 50, 45 } };
    const QByteArray msg = buildMsm5(1075, 100000, false, sats, { 2 }, cells);

    RtcmStreamParser parser;
    QSignalSpy spy(&parser, &RtcmStreamParser::rtcmFrames);

    // Feed one byte at a time; the framer must still produce exactly the original frame.
    for (const char byte : msg) {
        parser.addData(QByteArray(1, byte));
    }

    QCOMPARE(parser.validFrames(), 1ULL);
    QCOMPARE(parser.discardedBytes(), 0ULL);

    QByteArray emitted;
    for (const QList<QVariant> &args : spy) {
        emitted += args.at(0).toByteArray();
    }
    QCOMPARE(emitted, msg);
}

void RtcmRinexTest::_testMsm5RoundTrip()
{
    // Two GPS satellites, L1 C/A (signal 2) and L2 W (signal 10).
    const QList<SatelliteFields> sats = {
        { 5,  80, 512, 0, -1200 },
        { 12, 75, 100, 0,   900 },
    };
    const QList<int> signalIds = { 2, 10 };
    // A real receiver measures one range, so the fine pseudorange (2^-24 ms) and fine phase
    // (2^-29 ms) describe the same distance: phase = 32 x pseudorange. Keeping that relation here
    // is what makes the code-minus-phase check below meaningful.
    const QList<SignalFields> cells = {
        { 1000,  1000 * 32,  1500, 49 },   // sat 5,  L1
        { -500,  -500 * 32,  -800, 46 },   // sat 5,  L2
        { 3000,  3000 * 32,  2500, 43 },   // sat 12, L1
        { -2000, -2000 * 32, -300, 44 },   // sat 12, L2
    };

    RtcmObsDecoder decoder;
    QSignalSpy spy(&decoder, &RtcmObsDecoder::epochReady);
    decoder.addFrames(buildMsm5(1075, 100000, false, sats, signalIds, cells));

    QCOMPARE(spy.count(), 1);
    const RtcmObsEpoch epoch = spy.at(0).at(0).value<RtcmObsEpoch>();
    QCOMPARE(epoch.gpsTowMs, 100000);
    QCOMPARE(epoch.satellites.size(), 2);

    struct Expect { int slot; int cellIndex; const char *code; double frequency; };
    const QList<Expect> expectations = {
        { 5,  0, "1C", kFreqL1 },
        { 5,  1, "2W", kFreqL2 },
        { 12, 2, "1C", kFreqL1 },
        { 12, 3, "2W", kFreqL2 },
    };

    for (const Expect &e : expectations) {
        const RtcmSatObservations *sat = findSat(epoch, 'G', e.slot);
        QVERIFY2(sat, qPrintable(QStringLiteral("missing G%1").arg(e.slot)));
        QVERIFY2(sat->observations.contains(QString::fromLatin1(e.code)),
                 qPrintable(QStringLiteral("G%1 missing %2").arg(e.slot).arg(e.code)));

        const RtcmObservation obs = sat->observations.value(QString::fromLatin1(e.code));
        const SatelliteFields &satFields = (e.slot == 5) ? sats.at(0) : sats.at(1);
        const SignalFields &cell = cells.at(e.cellIndex);

        QVERIFY(obs.havePseudorange);
        QVERIFY(qAbs(obs.pseudorange - expectedPseudorange(satFields, cell)) < 1e-6);
        QVERIFY(obs.haveCarrierPhase);
        QVERIFY(qAbs(obs.carrierPhase - expectedCarrierPhase(satFields, cell, e.frequency)) < 1e-3);
        QVERIFY(obs.haveDoppler);
        QVERIFY(qAbs(obs.doppler - expectedDoppler(satFields, cell, e.frequency)) < 1e-6);
        QVERIFY(obs.haveSnr);
        QCOMPARE(obs.snr, static_cast<double>(cell.cnr));

        // Code and phase describe the same range, so converting the phase back to metres must
        // land on the pseudorange. A wrong wavelength or phase scale shows up here immediately.
        const double phaseMetres = obs.carrierPhase * kSpeedOfLight / e.frequency;
        QVERIFY(qAbs(obs.pseudorange - phaseMetres) < 0.001);
    }
}

void RtcmRinexTest::_testGlonassFrequencyChannel()
{
    // Extended satellite info 7 means frequency channel 0, so L1 is exactly 1602.000 MHz.
    const QList<SatelliteFields> sats = { { 3, 70, 256, 7, 0 } };
    const QList<SignalFields> cells = { { 500, 900, 0, 48 } };

    RtcmObsDecoder decoder;
    QSignalSpy spy(&decoder, &RtcmObsDecoder::epochReady);

    // GLONASS carries no week-based time, so it needs an open epoch from a GPS-timed message.
    decoder.addFrames(buildMsm5(1075, 200000, true, { { 5, 80, 512, 0, 0 } }, { 2 }, { { 0, 0, 0, 40 } }));
    decoder.addFrames(buildMsm5(1085, 300000, false, sats, { 2 }, cells));

    QCOMPARE(spy.count(), 1);
    const RtcmObsEpoch epoch = spy.at(0).at(0).value<RtcmObsEpoch>();

    const RtcmSatObservations *sat = findSat(epoch, 'R', 3);
    QVERIFY(sat);
    const RtcmObservation obs = sat->observations.value(QStringLiteral("1C"));
    QVERIFY(obs.haveCarrierPhase);
    QVERIFY(qAbs(obs.carrierPhase - expectedCarrierPhase(sats.at(0), cells.at(0), 1602.0e6)) < 1e-3);

    QCOMPARE(decoder.glonassChannels().value(3), 0);
}

void RtcmRinexTest::_testEpochAssembly()
{
    RtcmObsDecoder decoder;
    QSignalSpy spy(&decoder, &RtcmObsDecoder::epochReady);

    const QList<SignalFields> cells = { { 100, 200, 0, 45 } };

    // GPS and Galileo for the same instant, then BeiDou whose time of week runs 14 s behind GPS.
    decoder.addFrames(buildMsm5(1075, 100000, true, { { 5, 80, 512, 0, 0 } }, { 2 }, cells));
    decoder.addFrames(buildMsm5(1095, 100000, true, { { 7, 82, 256, 0, 0 } }, { 2 }, cells));
    decoder.addFrames(buildMsm5(1125, 100000 - 14000, false, { { 9, 84, 128, 0, 0 } }, { 2 }, cells));

    QCOMPARE(spy.count(), 1);
    const RtcmObsEpoch epoch = spy.at(0).at(0).value<RtcmObsEpoch>();
    QCOMPARE(epoch.gpsTowMs, 100000);
    QCOMPARE(epoch.satellites.size(), 3);
    QVERIFY(findSat(epoch, 'G', 5));
    QVERIFY(findSat(epoch, 'E', 7));
    QVERIFY(findSat(epoch, 'C', 9));

    // With the multiple-message bit still set the epoch stays open waiting for more
    // constellations, and is only closed when a message for the next instant arrives.
    decoder.addFrames(buildMsm5(1075, 101000, true, { { 5, 80, 512, 0, 0 } }, { 2 }, cells));
    QCOMPARE(spy.count(), 1);

    decoder.addFrames(buildMsm5(1075, 102000, true, { { 5, 80, 512, 0, 0 } }, { 2 }, cells));
    QCOMPARE(spy.count(), 2);
    QCOMPARE(spy.at(1).at(0).value<RtcmObsEpoch>().gpsTowMs, 101000);

    // flush() closes whatever is still open when the stream stops.
    decoder.flush();
    QCOMPARE(spy.count(), 3);
    QCOMPARE(spy.at(2).at(0).value<RtcmObsEpoch>().gpsTowMs, 102000);
}

void RtcmRinexTest::_testRinexOutput()
{
    QTemporaryDir tempDir;
    QVERIFY(tempDir.isValid());
    const QString path = tempDir.filePath(QStringLiteral("test.obs"));

    RtcmStreamParser parser;
    RtcmObsDecoder decoder;
    RinexObsWriter writer(path);
    QVERIFY(writer.isOpen());

    QObject::connect(&parser, &RtcmStreamParser::rtcmFrames, &decoder, &RtcmObsDecoder::addFrames);
    QObject::connect(&decoder, &RtcmObsDecoder::epochReady, &writer, &RinexObsWriter::writeEpoch);
    QObject::connect(&decoder, &RtcmObsDecoder::stationInfoChanged, &writer, [&writer, &decoder]() {
        writer.setStationInfo(decoder.stationInfo());
        writer.setGlonassChannels(decoder.glonassChannels());
    });

    const QList<SatelliteFields> sats = { { 5, 80, 512, 0, -1200 }, { 12, 75, 100, 0, 900 } };
    const QList<SignalFields> cells = {
        { 1000, 2000, 1500, 49 }, { -500, -1200, -800, 46 },
        { 3000, 4000, 2500, 43 }, { -2000, 5000, -300, 44 },
    };

    // More epochs than the writer buffers before committing to a header.
    const int epochCount = 8;
    for (int i = 0; i < epochCount; ++i) {
        parser.addData(buildMsm5(1075, 100000 + (i * 1000), false, sats, { 2, 10 }, cells));
    }
    writer.finish();

    QCOMPARE(writer.epochsWritten(), static_cast<quint64>(epochCount));

    QFile file(path);
    QVERIFY(file.open(QIODevice::ReadOnly | QIODevice::Text));
    const QStringList lines = QString::fromLatin1(file.readAll()).split(QLatin1Char('\n'));
    file.close();

    // --- header ---
    QVERIFY(lines.first().contains(QStringLiteral("RINEX VERSION / TYPE")));
    QVERIFY(lines.first().startsWith(QStringLiteral("     3.04")));
    QVERIFY(lines.first().mid(20, 16) == QStringLiteral("OBSERVATION DATA"));

    int endOfHeader = -1;
    QString obsTypeLine;
    for (int i = 0; i < lines.size(); ++i) {
        // Header labels live in columns 61-80.
        const QString label = lines.at(i).mid(60, 20).trimmed();
        if (label == QStringLiteral("SYS / # / OBS TYPES")) {
            obsTypeLine = lines.at(i);
        } else if (label == QStringLiteral("END OF HEADER")) {
            endOfHeader = i;
            break;
        }
    }
    QVERIFY2(endOfHeader > 0, "no END OF HEADER record");

    // Two signals -> C/L/D/S for each.
    QCOMPARE(obsTypeLine.at(0), QLatin1Char('G'));
    QCOMPARE(obsTypeLine.mid(1, 5).trimmed().toInt(), 8);
    for (const QString &code : { QStringLiteral("C1C"), QStringLiteral("L1C"), QStringLiteral("D1C"),
                                 QStringLiteral("S1C"), QStringLiteral("C2W"), QStringLiteral("L2W") }) {
        QVERIFY2(obsTypeLine.contains(code), qPrintable(QStringLiteral("missing obs type %1").arg(code)));
    }

    // --- epochs ---
    int epochLines = 0;
    int satLines = 0;
    for (int i = endOfHeader + 1; i < lines.size(); ++i) {
        const QString &line = lines.at(i);
        if (line.isEmpty()) {
            continue;
        }
        if (line.startsWith(QLatin1Char('>'))) {
            ++epochLines;
            QCOMPARE(line.mid(32, 3).trimmed().toInt(), sats.size());   // satellite count
        } else {
            ++satLines;
            QCOMPARE(line.at(0), QLatin1Char('G'));
            // C1C occupies columns 4-17 and must match the encoded pseudorange.
            const double pseudorange = line.mid(3, 14).trimmed().toDouble();
            const int slot = line.mid(1, 2).toInt();
            const SatelliteFields &satFields = (slot == 5) ? sats.at(0) : sats.at(1);
            const SignalFields &cell = (slot == 5) ? cells.at(0) : cells.at(2);
            QVERIFY(qAbs(pseudorange - expectedPseudorange(satFields, cell)) < 0.001);
        }
    }
    QCOMPARE(epochLines, epochCount);
    QCOMPARE(satLines, epochCount * sats.size());
}


namespace {

constexpr double kSemicircleToRad = 3.1415926535898;

/// Encodes a 1019 GPS ephemeris (488 bits) from raw field values.
QByteArray buildGpsEphemeris(int prn, int week, int iode, int toeRaw, qint64 m0Raw,
                             quint32 eccRaw, quint32 sqrtARaw, qint64 i0Raw, qint64 af0Raw)
{
    BitWriter w;
    w.u(12, 1019);
    w.u(6, prn);
    w.u(10, week);
    w.u(4, 0);              // URA
    w.u(2, 0);              // codes on L2
    w.s(14, 0);             // IDOT
    w.u(8, iode);
    w.u(16, toeRaw);        // toc
    w.s(8, 0);              // af2
    w.s(16, 0);             // af1
    w.s(22, af0Raw);
    w.u(10, iode);          // IODC
    w.s(16, 0);             // Crs
    w.s(16, 0);             // delta n
    w.s(32, m0Raw);
    w.s(16, 0);             // Cuc
    w.u(32, eccRaw);
    w.s(16, 0);             // Cus
    w.u(32, sqrtARaw);
    w.u(16, toeRaw);
    w.s(16, 0);             // Cic
    w.s(32, 0);             // Omega0
    w.s(16, 0);             // Cis
    w.s(32, i0Raw);
    w.s(16, 0);             // Crc
    w.s(32, 0);             // omega
    w.s(24, 0);             // Omega dot
    w.s(8, 0);              // TGD
    w.u(6, 0);              // health
    w.u(1, 0);              // L2 P flag
    w.u(1, 0);              // fit interval
    return frame(w.payload());
}

/// Encodes a 1020 GLONASS ephemeris (360 bits). Coordinates are sign-magnitude.
QByteArray buildGlonassEphemeris(int slot, int channel, int tb,
                                 qint64 xRaw, qint64 yRaw, qint64 zRaw)
{
    // Sign-magnitude: the top bit is the sign, the rest is the magnitude.
    const auto signMagnitude = [](qint64 value, int bits) -> quint64 {
        const quint64 magnitude = static_cast<quint64>(value < 0 ? -value : value);
        return (value < 0) ? (magnitude | (1ULL << (bits - 1))) : magnitude;
    };

    BitWriter w;
    w.u(12, 1020);
    w.u(6, slot);
    w.u(5, channel + 7);
    w.u(1, 0); w.u(1, 0); w.u(2, 0);        // almanac health, availability, P1
    w.u(12, 0);                             // tk
    w.u(1, 0); w.u(1, 0);                   // Bn, P2
    w.u(7, tb);
    w.u(24, signMagnitude(0, 24));          // vx
    w.u(27, signMagnitude(xRaw, 27));
    w.u(5, signMagnitude(0, 5));            // ax
    w.u(24, signMagnitude(0, 24));          // vy
    w.u(27, signMagnitude(yRaw, 27));
    w.u(5, signMagnitude(0, 5));            // ay
    w.u(24, signMagnitude(0, 24));          // vz
    w.u(27, signMagnitude(zRaw, 27));
    w.u(5, signMagnitude(0, 5));            // az
    w.u(1, 0);                              // P3
    w.u(11, signMagnitude(0, 11));          // gamma
    w.u(2, 0); w.u(1, 0);                   // P, ln
    w.u(22, signMagnitude(0, 22));          // tau n
    w.u(5, signMagnitude(0, 5));            // delta tau n
    w.u(5, 0);                              // En
    w.u(1, 0); w.u(4, 0);                   // P4, Ft
    w.u(11, 1);                             // Nt
    w.u(2, 0); w.u(1, 0);                   // M, availability
    w.u(11, 0);                             // NA
    w.u(32, 0);                             // tau c
    w.u(5, 8);                              // N4
    w.u(22, 0); w.u(1, 0); w.u(7, 0);       // tau GPS, ln, reserved
    return frame(w.payload());
}

} // namespace

void RtcmRinexTest::_testGpsEphemerisRoundTrip()
{
    // Values chosen so the decoded orbit is a real GPS orbit: sqrtA ~ 5153.7 m^0.5, e ~ 0.007,
    // i0 ~ 0.99 rad. A wrong scale factor or bit offset moves these far out of range.
    const quint32 sqrtARaw = 2701131947U;       // 5153.7 / 2^-19
    const quint32 eccRaw = 60129542U;           // ~0.007 / 2^-33
    const qint64 i0Raw = 680390000LL;           // ~0.317 semicircles
    const qint64 m0Raw = 123456789LL;
    const qint64 af0Raw = -12345LL;
    const int toeRaw = 5400;                    // 5400 * 16 = 86400 s

    RtcmNavDecoder decoder;
    QSignalSpy spy(&decoder, &RtcmNavDecoder::ephemerisReady);
    decoder.addFrames(buildGpsEphemeris(7, 380, 105, toeRaw, m0Raw, eccRaw, sqrtARaw, i0Raw, af0Raw));

    QCOMPARE(spy.count(), 1);
    const RtcmEphemeris e = spy.at(0).at(0).value<RtcmEphemeris>();

    QCOMPARE(e.system, 'G');
    QCOMPARE(e.prn, 7);
    QCOMPARE(e.week, 380);
    QCOMPARE(e.iode, 105);
    QCOMPARE(e.toe, 86400.0);
    QCOMPARE(e.toc, 86400.0);

    QVERIFY(qAbs(e.sqrtA - (sqrtARaw * std::pow(2.0, -19))) < 1e-6);
    QVERIFY(qAbs(e.eccentricity - (eccRaw * std::pow(2.0, -33))) < 1e-12);
    QVERIFY(qAbs(e.i0 - (i0Raw * std::pow(2.0, -31) * kSemicircleToRad)) < 1e-12);
    QVERIFY(qAbs(e.m0 - (m0Raw * std::pow(2.0, -31) * kSemicircleToRad)) < 1e-12);
    QVERIFY(qAbs(e.af0 - (af0Raw * std::pow(2.0, -31))) < 1e-15);

    // The decoded orbit must actually be a GPS orbit.
    const double semiMajorAxisKm = (e.sqrtA * e.sqrtA) / 1000.0;
    QVERIFY2(qAbs(semiMajorAxisKm - 26560.0) < 50.0,
             qPrintable(QStringLiteral("semi-major axis %1 km").arg(semiMajorAxisKm)));
    QVERIFY(e.eccentricity < 0.03);
}

void RtcmRinexTest::_testGlonassEphemerisSignMagnitude()
{
    // A negative Y coordinate is the point: reading sign-magnitude as two's complement would
    // return a huge positive number and the orbit radius would be nonsense.
    const qint64 xRaw = 20000000LL;             // * 2^-11 km
    const qint64 yRaw = -15000000LL;
    const qint64 zRaw = 8000000LL;

    RtcmNavDecoder decoder;
    QSignalSpy spy(&decoder, &RtcmNavDecoder::ephemerisReady);
    decoder.addFrames(buildGlonassEphemeris(9, -4, 40, xRaw, yRaw, zRaw));

    QCOMPARE(spy.count(), 1);
    const RtcmEphemeris e = spy.at(0).at(0).value<RtcmEphemeris>();

    QCOMPARE(e.system, 'R');
    QCOMPARE(e.prn, 9);
    QCOMPARE(e.frequencyChannel, -4);
    QCOMPARE(e.intervalIndex, 40);

    const double scale = std::pow(2.0, -11) * 1000.0;
    QVERIFY(qAbs(e.positionX - (xRaw * scale)) < 1e-3);
    QVERIFY(qAbs(e.positionY - (yRaw * scale)) < 1e-3);
    QVERIFY(qAbs(e.positionZ - (zRaw * scale)) < 1e-3);
    QVERIFY2(e.positionY < 0.0, "sign-magnitude decoding lost the negative sign");
}

void RtcmRinexTest::_testNavOutputAndDeduplication()
{
    QTemporaryDir tempDir;
    QVERIFY(tempDir.isValid());
    const QString obsPath = tempDir.filePath(QStringLiteral("test.obs"));
    const QString navPath = RinexNavWriter::pathForObsFile(obsPath);
    QVERIFY(navPath.endsWith(QStringLiteral("test.nav")));

    RtcmStreamParser parser;
    RtcmNavDecoder decoder;
    RinexNavWriter writer(navPath);
    QVERIFY(writer.isOpen());

    QObject::connect(&parser, &RtcmStreamParser::rtcmFrames, &decoder, &RtcmNavDecoder::addFrames);
    QObject::connect(&decoder, &RtcmNavDecoder::ephemerisReady, &writer, &RinexNavWriter::writeEphemeris);

    const QByteArray gps = buildGpsEphemeris(7, 380, 105, 5400, 123456789LL, 60129542U, 2701131947U,
                                             680390000LL, -12345LL);
    const QByteArray glo = buildGlonassEphemeris(9, -4, 40, 20000000LL, -15000000LL, 8000000LL);

    // Feed each one several times: satellites rebroadcast continuously.
    for (int i = 0; i < 5; ++i) {
        parser.addData(gps);
        parser.addData(glo);
    }
    // A different issue of data for the same satellite is a genuinely new record.
    parser.addData(buildGpsEphemeris(7, 380, 106, 5580, 123456789LL, 60129542U, 2701131947U,
                                     680390000LL, -12345LL));
    writer.finish();

    QCOMPARE(decoder.decodedCount(), 11ULL);    // every message decoded
    QCOMPARE(writer.recordsWritten(), 3ULL);    // but only three distinct records written

    QFile file(navPath);
    QVERIFY(file.open(QIODevice::ReadOnly | QIODevice::Text));
    const QStringList lines = QString::fromLatin1(file.readAll()).split(QLatin1Char('\n'));
    file.close();

    QVERIFY(lines.first().contains(QStringLiteral("RINEX VERSION / TYPE")));
    QVERIFY(lines.first().contains(QStringLiteral("N: GNSS NAV DATA")));

    int endOfHeader = -1;
    for (int i = 0; i < lines.size(); ++i) {
        if (lines.at(i).mid(60, 20).trimmed() == QStringLiteral("END OF HEADER")) {
            endOfHeader = i;
            break;
        }
    }
    QVERIFY2(endOfHeader > 0, "no END OF HEADER record");

    // GPS records are 8 lines, GLONASS 4: two GPS plus one GLONASS is 20 lines.
    int gpsRecords = 0;
    int gloRecords = 0;
    int bodyLines = 0;
    for (int i = endOfHeader + 1; i < lines.size(); ++i) {
        const QString &line = lines.at(i);
        if (line.isEmpty()) {
            continue;
        }
        ++bodyLines;
        if (line.startsWith(QStringLiteral("G0"))) {
            ++gpsRecords;
        } else if (line.startsWith(QStringLiteral("R0"))) {
            ++gloRecords;
        }
    }
    QCOMPARE(gpsRecords, 2);
    QCOMPARE(gloRecords, 1);
    QCOMPARE(bodyLines, (2 * 8) + (1 * 4));

    // Every numeric field must be the Fortran D exponent form RINEX readers expect.
    for (int i = endOfHeader + 1; i < lines.size(); ++i) {
        const QString &line = lines.at(i);
        if (line.isEmpty()) {
            continue;
        }
        QVERIFY2(line.contains(QLatin1Char('D')),
                 qPrintable(QStringLiteral("no D exponent in: %1").arg(line)));
    }
}


namespace {

/// Encodes a 1006 stationary reference station ARP with antenna height (168 bits).
QByteArray buildStationArp(int stationId, qint64 xRaw, qint64 yRaw, qint64 zRaw, int heightRaw)
{
    BitWriter w;
    w.u(12, 1006);
    w.u(12, stationId);
    w.u(6, 0);              // ITRF realisation year
    w.u(1, 1);              // GPS indicator
    w.u(1, 1);              // GLONASS indicator
    w.u(1, 1);              // Galileo indicator
    w.u(1, 0);              // reference station indicator
    w.s(38, xRaw);
    w.u(1, 0);              // single receiver oscillator
    w.u(1, 0);              // reserved
    w.s(38, yRaw);
    w.u(2, 0);              // quarter cycle indicator
    w.s(38, zRaw);
    w.u(16, heightRaw);
    return frame(w.payload());
}

} // namespace

void RtcmRinexTest::_testBasePositionReachesHeader()
{
    QTemporaryDir tempDir;
    QVERIFY(tempDir.isValid());
    const QString path = tempDir.filePath(QStringLiteral("base.obs"));

    RtcmStreamParser parser;
    RtcmObsDecoder decoder;
    RinexObsWriter writer(path);
    QVERIFY(writer.isOpen());

    QObject::connect(&parser, &RtcmStreamParser::rtcmFrames, &decoder, &RtcmObsDecoder::addFrames);
    QObject::connect(&decoder, &RtcmObsDecoder::epochReady, &writer, &RinexObsWriter::writeEpoch);
    QObject::connect(&decoder, &RtcmObsDecoder::stationInfoChanged, &writer, [&writer, &decoder]() {
        writer.setStationInfo(decoder.stationInfo());
        writer.setGlonassChannels(decoder.glonassChannels());
    });

    const QList<SatelliteFields> sats = { { 5, 80, 512, 0, -1200 } };
    const QList<SignalFields> cells = { { 1000, 1000 * 32, 1500, 49 } };

    // ECEF for a point on the surface; 0.1 mm resolution means raw = metres * 10000.
    const double x = -1615000.1234;
    const double y = 6008000.5678;
    const double z = 1174000.9012;
    const double antennaHeight = 1.5;

    // Ten epochs go by before the base position shows up - more than kHeaderEpochs, so a writer
    // that committed the header on schedule would already have lost it.
    for (int i = 0; i < 10; ++i) {
        parser.addData(buildMsm5(1075, 100000 + (i * 1000), false, sats, { 2 }, cells));
    }
    parser.addData(buildStationArp(2050,
                                   static_cast<qint64>(std::llround(x * 10000.0)),
                                   static_cast<qint64>(std::llround(y * 10000.0)),
                                   static_cast<qint64>(std::llround(z * 10000.0)),
                                   static_cast<int>(std::llround(antennaHeight * 10000.0))));
    for (int i = 10; i < 14; ++i) {
        parser.addData(buildMsm5(1075, 100000 + (i * 1000), false, sats, { 2 }, cells));
    }
    writer.finish();

    // Nothing was dropped while waiting for the position.
    QCOMPARE(writer.epochsWritten(), 14ULL);

    QFile file(path);
    QVERIFY(file.open(QIODevice::ReadOnly | QIODevice::Text));
    const QStringList lines = QString::fromLatin1(file.readAll()).split(QLatin1Char('\n'));
    file.close();

    QString positionLine;
    QString antennaLine;
    QString markerLine;
    for (const QString &line : lines) {
        const QString label = line.mid(60, 20).trimmed();
        if (label == QStringLiteral("APPROX POSITION XYZ")) {
            positionLine = line;
        } else if (label == QStringLiteral("ANTENNA: DELTA H/E/N")) {
            antennaLine = line;
        } else if (label == QStringLiteral("MARKER NAME")) {
            markerLine = line;
        }
    }

    QVERIFY2(!positionLine.isEmpty(), "the observation header has no APPROX POSITION XYZ record");
    // Three F14.4 fields.
    QVERIFY(qAbs(positionLine.mid(0, 14).trimmed().toDouble() - x) < 0.001);
    QVERIFY(qAbs(positionLine.mid(14, 14).trimmed().toDouble() - y) < 0.001);
    QVERIFY(qAbs(positionLine.mid(28, 14).trimmed().toDouble() - z) < 0.001);

    QVERIFY2(!antennaLine.isEmpty(), "no ANTENNA: DELTA H/E/N record");
    QVERIFY(qAbs(antennaLine.mid(0, 14).trimmed().toDouble() - antennaHeight) < 0.001);

    // The station id identifies which base produced the file.
    QVERIFY2(markerLine.contains(QStringLiteral("2050")),
             qPrintable(QStringLiteral("marker name lost the station id: %1").arg(markerLine)));
}


void RtcmRinexTest::_testBothLogFormats()
{
    QTemporaryDir tempDir;
    QVERIFY(tempDir.isValid());
    const QString base = tempDir.filePath(QStringLiteral("session"));

    // Mirrors how RTCMStreamManager wires the "Both" format: one parser feeding a RINEX
    // observation writer, a RINEX navigation writer and the raw RTCM3 logger.
    RtcmStreamParser parser;
    RtcmObsDecoder obsDecoder;
    RtcmNavDecoder navDecoder;
    RinexObsWriter obsWriter(base + QStringLiteral(".obs"));
    RinexNavWriter navWriter(base + QStringLiteral(".nav"));
    RTCMFileLogger rawLogger(base + QStringLiteral(".rtcm3"));

    QVERIFY(obsWriter.isOpen());
    QVERIFY(navWriter.isOpen());

    QObject::connect(&parser, &RtcmStreamParser::rtcmFrames, &obsDecoder, &RtcmObsDecoder::addFrames);
    QObject::connect(&parser, &RtcmStreamParser::rtcmFrames, &navDecoder, &RtcmNavDecoder::addFrames);
    QObject::connect(&parser, &RtcmStreamParser::rtcmFrames, &rawLogger, &RTCMFileLogger::logData);
    QObject::connect(&obsDecoder, &RtcmObsDecoder::epochReady, &obsWriter, &RinexObsWriter::writeEpoch);
    QObject::connect(&navDecoder, &RtcmNavDecoder::ephemerisReady, &navWriter, &RinexNavWriter::writeEphemeris);
    QObject::connect(&obsDecoder, &RtcmObsDecoder::stationInfoChanged, &obsWriter, [&obsWriter, &obsDecoder]() {
        obsWriter.setStationInfo(obsDecoder.stationInfo());
        obsWriter.setGlonassChannels(obsDecoder.glonassChannels());
    });

    const QList<SatelliteFields> sats = { { 5, 80, 512, 0, -1200 } };
    const QList<SignalFields> cells = { { 1000, 1000 * 32, 1500, 49 } };

    // A stream carrying observations, a station position and an ephemeris - what a real base sends.
    QByteArray stream;
    stream += buildStationArp(1386, -18131912854LL, 59971223142LL, 11907854045LL, 766);
    stream += buildGpsEphemeris(7, 380, 105, 5400, 123456789LL, 60129542U, 2701131947U,
                                680390000LL, -12345LL);
    for (int i = 0; i < 8; ++i) {
        stream += buildMsm5(1075, 100000 + (i * 1000), false, sats, { 2 }, cells);
    }
    parser.addData(stream);

    obsWriter.finish();
    navWriter.finish();

    QCOMPARE(obsWriter.epochsWritten(), 8ULL);
    QCOMPARE(navWriter.recordsWritten(), 1ULL);

    // All three files exist, share the base name, and none is empty.
    for (const QString &extension : { QStringLiteral(".obs"), QStringLiteral(".nav"), QStringLiteral(".rtcm3") }) {
        const QString path = base + extension;
        QFileInfo info(path);
        QVERIFY2(info.exists(), qPrintable(QStringLiteral("missing %1").arg(path)));
        QVERIFY2(info.size() > 0, qPrintable(QStringLiteral("empty %1").arg(path)));
        QCOMPARE(info.completeBaseName(), QStringLiteral("session"));
    }

    // The raw log must be the validated stream byte for byte, so any RTCM tool can replay it.
    QFile raw(base + QStringLiteral(".rtcm3"));
    QVERIFY(raw.open(QIODevice::ReadOnly));
    const QByteArray rawBytes = raw.readAll();
    raw.close();
    QCOMPARE(rawBytes, stream);

    // The observation file carries the base position that only the raw stream knew about.
    QFile obs(base + QStringLiteral(".obs"));
    QVERIFY(obs.open(QIODevice::ReadOnly | QIODevice::Text));
    const QString obsText = QString::fromLatin1(obs.readAll());
    obs.close();
    QVERIFY2(obsText.contains(QStringLiteral("APPROX POSITION XYZ")), "no base position in the .obs");
    QVERIFY(obsText.contains(QStringLiteral("RTCM1386")));

    // And the navigation file carries the ephemeris.
    QFile nav(base + QStringLiteral(".nav"));
    QVERIFY(nav.open(QIODevice::ReadOnly | QIODevice::Text));
    const QString navText = QString::fromLatin1(nav.readAll());
    nav.close();
    QVERIFY2(navText.contains(QStringLiteral("N: GNSS NAV DATA")), "not a navigation file");
    QVERIFY2(navText.contains(QStringLiteral("G07")), "no GPS 7 ephemeris record");
}
