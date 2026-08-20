/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "RinexObsWriter.h"
#include "AppSettings.h"
#include "QGCLoggingCategory.h"
#include "SettingsManager.h"

#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QStandardPaths>

#include <algorithm>
#include <cmath>

QGC_LOGGING_CATEGORY(RinexObsWriterLog, "qgc.rtk.rinexobswriter")

namespace {

constexpr char kRinexVersion[] = "3.04";

/// GPS time origin: 1980-01-06 00:00:00 UTC.
QDateTime gpsEpochOrigin()
{
    return QDateTime(QDate(1980, 1, 6), QTime(0, 0, 0), QTimeZone::UTC);
}

/// RINEX orders systems G R E J C S I; anything else follows.
int systemOrder(char system)
{
    static const QByteArray order("GREJCSI");
    const int index = order.indexOf(system);
    return (index < 0) ? order.size() : index;
}

/// RINEX signal strength indicator: 1..9 mapped from dB-Hz in 6 dB steps.
int snrIndicator(double snr)
{
    const int value = static_cast<int>(snr / 6.0);
    return std::clamp(value, 1, 9);
}

/// Observation records list codes as C/L/D/S for each signal, in a stable order.
QList<QString> observationCodesFor(const QList<QString> &signalCodes)
{
    QList<QString> codes;
    codes.reserve(signalCodes.size() * 4);
    for (const QString &signal : signalCodes) {
        codes.append(QStringLiteral("C") + signal);
        codes.append(QStringLiteral("L") + signal);
        codes.append(QStringLiteral("D") + signal);
        codes.append(QStringLiteral("S") + signal);
    }
    return codes;
}

} // namespace

RinexObsWriter::RinexObsWriter(const QString &filePath, QObject *parent)
    : QObject(parent)
    , _filePath(filePath.isEmpty() ? defaultFilePath() : filePath)
{
    (void) _openFile();
}

RinexObsWriter::~RinexObsWriter()
{
    finish();
}

QString RinexObsWriter::defaultFilePath()
{
    // Same resolution as RTCMFileLogger: prefer the user-visible save folder so the file can be
    // pulled off a tablet, then app-data, then temp.
    QString base;
    if (AppSettings *const appSettings = SettingsManager::instance()->appSettings()) {
        base = appSettings->savePath()->rawValue().toString();
    }
    if (base.isEmpty() || !QDir(base).exists()) {
        base = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    }
    if (base.isEmpty()) {
        base = QDir::tempPath();
    }

    const QDir dir(QDir(base).filePath(QStringLiteral("RTCMLogs")));
    if (!dir.exists()) {
        (void) QDir().mkpath(dir.absolutePath());
    }

    const QString stamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd_hh-mm-ss"));
    return dir.filePath(QStringLiteral("rtcm_%1.obs").arg(stamp));
}

bool RinexObsWriter::_openFile()
{
    const QFileInfo info(_filePath);
    if (!info.dir().exists()) {
        (void) QDir().mkpath(info.dir().absolutePath());
    }

    _file.setFileName(_filePath);
    // Truncate rather than append: a RINEX file is a header plus its own epochs, so appending to
    // an existing one would produce a second header mid-file and an unreadable result.
    if (!_file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        qCWarning(RinexObsWriterLog) << "Cannot open RINEX file" << _filePath << _file.errorString();
        return false;
    }

    qCDebug(RinexObsWriterLog) << "Writing RINEX observations to" << _filePath;
    return true;
}

void RinexObsWriter::setStationInfo(const RtcmStationInfo &info)
{
    _station = info;
}

void RinexObsWriter::setGlonassChannels(const QHash<int, int> &channels)
{
    _glonassChannels = channels;
}

void RinexObsWriter::_write(const QString &line)
{
    if (!_file.isOpen()) {
        return;
    }

    const QByteArray bytes = (line + QStringLiteral("\n")).toLatin1();
    const qint64 written = _file.write(bytes);
    if (written > 0) {
        _totalBytes += static_cast<quint64>(written);
    }
}

QString RinexObsWriter::_headerLine(const QString &content, const QString &label)
{
    // RINEX header records are 60 columns of content followed by a 20 column label.
    return content.leftJustified(60, QLatin1Char(' '), true) + label.leftJustified(20, QLatin1Char(' '), true);
}

void RinexObsWriter::_collectObsTypes(const RtcmObsEpoch &epoch)
{
    for (const RtcmSatObservations &sat : epoch.satellites) {
        QList<QString> &signalCodes = _obsTypes[sat.system];
        for (auto it = sat.observations.cbegin(); it != sat.observations.cend(); ++it) {
            if (!signalCodes.contains(it.key())) {
                signalCodes.append(it.key());
            }
        }
    }
}

QDateTime RinexObsWriter::_epochToGpsDateTime(qint64 gpsTowMs) const
{
    // MSM messages carry a time of week but no week number, so the week comes from the system
    // clock. A stream more than half a week out of step with this machine's clock would be
    // resolved into the wrong week - that is inherent to the format, not to this code.
    return gpsEpochOrigin().addMSecs((_gpsWeek * 604800000LL) + gpsTowMs);
}

void RinexObsWriter::_writeHeader(const RtcmObsEpoch &firstEpoch)
{
    // Resolve the GPS week from the wall clock, then nudge it if the time of week says the epoch
    // actually belongs either side of a week rollover.
    const QDateTime nowGps = QDateTime::currentDateTimeUtc().addSecs(kGpsUtcLeapSeconds);
    const qint64 sinceOriginMs = gpsEpochOrigin().msecsTo(nowGps);
    _gpsWeek = sinceOriginMs / 604800000LL;
    const qint64 nowTowMs = sinceOriginMs % 604800000LL;
    if ((firstEpoch.gpsTowMs - nowTowMs) > 302400000LL) {
        --_gpsWeek;
    } else if ((nowTowMs - firstEpoch.gpsTowMs) > 302400000LL) {
        ++_gpsWeek;
    }

    _write(QStringLiteral("%1%2%3%4")
               .arg(QString(kRinexVersion).rightJustified(9))
               .arg(QString(), 11)
               .arg(QStringLiteral("OBSERVATION DATA").leftJustified(20))
               .arg(QStringLiteral("M: MIXED").leftJustified(20))
           + QStringLiteral("RINEX VERSION / TYPE"));

    const QString runDate = QDateTime::currentDateTimeUtc().toString(QStringLiteral("yyyyMMdd hhmmss UTC"));
    _write(_headerLine(QStringLiteral("QGroundControl").leftJustified(20)
                           + QStringLiteral("QGroundControl").leftJustified(20)
                           + runDate,
                       QStringLiteral("PGM / RUN BY / DATE")));

    _write(_headerLine(QStringLiteral("Decoded from an RTCM3 correction stream by QGroundControl."),
                       QStringLiteral("COMMENT")));
    if (_station.stationId >= 0) {
        _write(_headerLine(QStringLiteral("RTCM reference station ID %1").arg(_station.stationId),
                           QStringLiteral("COMMENT")));
    }

    const QString markerName = (_station.stationId >= 0)
                                   ? QStringLiteral("RTCM%1").arg(_station.stationId, 4, 10, QLatin1Char('0'))
                                   : QStringLiteral("UNKNOWN");
    _write(_headerLine(markerName, QStringLiteral("MARKER NAME")));
    _write(_headerLine(QStringLiteral("NON_GEODETIC"), QStringLiteral("MARKER TYPE")));
    _write(_headerLine(QStringLiteral("UNKNOWN").leftJustified(20) + QStringLiteral("UNKNOWN"),
                       QStringLiteral("OBSERVER / AGENCY")));

    _write(_headerLine(_station.receiverSerial.leftJustified(20)
                           + _station.receiverType.leftJustified(20)
                           + _station.receiverVersion.leftJustified(20),
                       QStringLiteral("REC # / TYPE / VERS")));
    _write(_headerLine(_station.antennaSerial.leftJustified(20)
                           + _station.antennaDescriptor.leftJustified(20),
                       QStringLiteral("ANT # / TYPE")));

    if (_station.havePosition) {
        _write(_headerLine(QStringLiteral("%1%2%3")
                               .arg(_station.ecefX, 14, 'f', 4)
                               .arg(_station.ecefY, 14, 'f', 4)
                               .arg(_station.ecefZ, 14, 'f', 4),
                           QStringLiteral("APPROX POSITION XYZ")));
    }
    // The RTCM ARP is already the antenna reference point, so the eccentricity is zero. 1006's
    // antenna height is the height of the ARP above the monument, which RINEX records here.
    _write(_headerLine(QStringLiteral("%1%2%3")
                           .arg(_station.antennaHeight, 14, 'f', 4)
                           .arg(0.0, 14, 'f', 4)
                           .arg(0.0, 14, 'f', 4),
                       QStringLiteral("ANTENNA: DELTA H/E/N")));

    // SYS / # / OBS TYPES - up to 13 codes per line, continuation lines are blank in columns 1-6.
    QList<char> systems = _obsTypes.keys();
    std::sort(systems.begin(), systems.end(), [](char a, char b) {
        return systemOrder(a) < systemOrder(b);
    });
    for (const char system : systems) {
        QList<QString> signalCodes = _obsTypes.value(system);
        std::sort(signalCodes.begin(), signalCodes.end());
        _obsTypes[system] = signalCodes;

        const QList<QString> codes = observationCodesFor(signalCodes);
        for (int start = 0; start < codes.size(); start += 13) {
            QString content;
            if (start == 0) {
                content = QStringLiteral("%1  %2").arg(QChar::fromLatin1(system)).arg(codes.size(), 3);
            } else {
                content = QStringLiteral("      ");
            }
            for (int i = start; (i < (start + 13)) && (i < codes.size()); ++i) {
                content += QStringLiteral(" ") + codes.at(i).leftJustified(3);
            }
            _write(_headerLine(content, QStringLiteral("SYS / # / OBS TYPES")));
        }
    }

    if (_interval > 0.0) {
        _write(_headerLine(QStringLiteral("%1").arg(_interval, 10, 'f', 3), QStringLiteral("INTERVAL")));
    }

    const QDateTime first = _epochToGpsDateTime(firstEpoch.gpsTowMs);
    const double seconds = first.time().second() + (first.time().msec() / 1000.0);
    _write(_headerLine(QStringLiteral("%1%2%3%4%5%6%7%8")
                           .arg(first.date().year(), 6)
                           .arg(first.date().month(), 6)
                           .arg(first.date().day(), 6)
                           .arg(first.time().hour(), 6)
                           .arg(first.time().minute(), 6)
                           .arg(seconds, 13, 'f', 7)
                           .arg(QString(), 5)
                           .arg(QStringLiteral("GPS")),
                       QStringLiteral("TIME OF FIRST OBS")));

    if (!_glonassChannels.isEmpty()) {
        QList<int> slotNumbers = _glonassChannels.keys();
        std::sort(slotNumbers.begin(), slotNumbers.end());
        for (int start = 0; start < slotNumbers.size(); start += 8) {
            QString content;
            content = (start == 0) ? QStringLiteral("%1").arg(slotNumbers.size(), 3) : QStringLiteral("   ");
            for (int i = start; (i < (start + 8)) && (i < slotNumbers.size()); ++i) {
                content += QStringLiteral(" R%1 %2")
                               .arg(slotNumbers.at(i), 2, 10, QLatin1Char('0'))
                               .arg(_glonassChannels.value(slotNumbers.at(i)), 2);
            }
            _write(_headerLine(content, QStringLiteral("GLONASS SLOT / FRQ #")));
        }
    }
    // Code-phase biases are not decoded from 1230, so declare them absent rather than wrong.
    _write(_headerLine(QStringLiteral(" C1C    0.000 C1P    0.000 C2C    0.000 C2P    0.000"),
                       QStringLiteral("GLONASS COD/PHS/BIS")));

    _write(_headerLine(QString(), QStringLiteral("END OF HEADER")));

    _headerWritten = true;
}

void RinexObsWriter::_writeEpochRecord(const RtcmObsEpoch &epoch)
{
    QList<RtcmSatObservations> sats = epoch.satellites;
    std::sort(sats.begin(), sats.end(), [](const RtcmSatObservations &a, const RtcmSatObservations &b) {
        if (a.system != b.system) {
            return systemOrder(a.system) < systemOrder(b.system);
        }
        return a.prn < b.prn;
    });

    const QDateTime time = _epochToGpsDateTime(epoch.gpsTowMs);
    const double seconds = time.time().second() + (time.time().msec() / 1000.0);

    _write(QStringLiteral("> %1 %2 %3 %4 %5%6  %7%8")
               .arg(time.date().year(), 4)
               .arg(time.date().month(), 2, 10, QLatin1Char('0'))
               .arg(time.date().day(), 2, 10, QLatin1Char('0'))
               .arg(time.time().hour(), 2, 10, QLatin1Char('0'))
               .arg(time.time().minute(), 2, 10, QLatin1Char('0'))
               .arg(seconds, 11, 'f', 7)
               .arg(0)                      // epoch flag: 0 = OK
               .arg(sats.size(), 3));

    for (const RtcmSatObservations &sat : sats) {
        QString line = QStringLiteral("%1%2")
                           .arg(QChar::fromLatin1(sat.system))
                           .arg(sat.prn, 2, 10, QLatin1Char('0'));

        const QList<QString> signalCodes = _obsTypes.value(sat.system);
        for (const QString &signal : signalCodes) {
            const auto it = sat.observations.constFind(signal);
            const bool present = (it != sat.observations.cend());

            // Each observation is F14.3 plus a loss-of-lock and a signal-strength digit; a value
            // that is not present is 16 blanks so the columns still line up.
            const auto field = [](bool have, double value, int lli, int ssi) {
                if (!have) {
                    return QString(16, QLatin1Char(' '));
                }
                const QString lliText = lli ? QString::number(lli) : QStringLiteral(" ");
                const QString ssiText = ssi ? QString::number(ssi) : QStringLiteral(" ");
                return QStringLiteral("%1%2%3").arg(value, 14, 'f', 3).arg(lliText).arg(ssiText);
            };

            if (!present) {
                line += QString(16 * 4, QLatin1Char(' '));
                continue;
            }

            const RtcmObservation &obs = it.value();
            const int ssi = obs.haveSnr ? snrIndicator(obs.snr) : 0;
            line += field(obs.havePseudorange, obs.pseudorange, 0, ssi);
            line += field(obs.haveCarrierPhase, obs.carrierPhase, obs.lli, ssi);
            line += field(obs.haveDoppler, obs.doppler, 0, 0);
            line += field(obs.haveSnr, obs.snr, 0, 0);
        }

        // Trailing blanks carry no information and only bloat the file.
        while (line.endsWith(QLatin1Char(' '))) {
            line.chop(1);
        }
        _write(line);
    }

    ++_epochsWritten;
}

void RinexObsWriter::writeEpoch(const RtcmObsEpoch &epoch)
{
    if (!_file.isOpen() || (epoch.gpsTowMs < 0) || epoch.satellites.isEmpty()) {
        return;
    }

    if (_firstTowMs < 0) {
        _firstTowMs = epoch.gpsTowMs;
    } else if ((_interval <= 0.0) && (_lastTowMs >= 0) && (epoch.gpsTowMs > _lastTowMs)) {
        _interval = static_cast<double>(epoch.gpsTowMs - _lastTowMs) / 1000.0;
    }
    _lastTowMs = epoch.gpsTowMs;

    if (!_headerWritten) {
        _collectObsTypes(epoch);
        _pending.append(epoch);

        if (_pending.size() < kHeaderEpochs) {
            return;
        }
        // Hold the header back until the base station position is known - it only arrives every
        // few seconds, and it is the one field post-processing cannot do without.
        if (!_station.havePosition && (_pending.size() < kMaxHeaderWaitEpochs)) {
            return;
        }
        if (!_station.havePosition) {
            qCWarning(RinexObsWriterLog)
                << "No RTCM 1005/1006 after" << _pending.size()
                << "epochs - writing the header without APPROX POSITION XYZ. The base position "
                   "will have to be supplied to the post-processing tool by hand.";
        }

        _writeHeader(_pending.first());
        for (const RtcmObsEpoch &buffered : _pending) {
            _writeEpochRecord(buffered);
        }
        _pending.clear();
        (void) _file.flush();
        emit bytesWrittenChanged(_totalBytes);
        return;
    }

    _writeEpochRecord(epoch);
    // Flush every epoch: a correction session usually ends with the app being closed or the link
    // dropping, and an unflushed tail would cost the last seconds of data.
    (void) _file.flush();
    emit bytesWrittenChanged(_totalBytes);
}

void RinexObsWriter::finish()
{
    if (!_file.isOpen()) {
        return;
    }

    if (!_headerWritten && !_pending.isEmpty()) {
        _writeHeader(_pending.first());
        for (const RtcmObsEpoch &buffered : _pending) {
            _writeEpochRecord(buffered);
        }
        _pending.clear();
    }

    (void) _file.flush();
    _file.close();
    emit bytesWrittenChanged(_totalBytes);

    qCDebug(RinexObsWriterLog) << "Closed RINEX file" << _filePath
                               << "epochs" << _epochsWritten << "bytes" << _totalBytes;
}
