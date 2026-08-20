/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "RinexNavWriter.h"
#include "QGCLoggingCategory.h"

#include <QtCore/QDir>
#include <QtCore/QFileInfo>

#include <cmath>

QGC_LOGGING_CATEGORY(RinexNavWriterLog, "qgc.rtk.rinexnavwriter")

namespace {

constexpr char kRinexVersion[] = "3.04";
constexpr qint64 kSecondsPerWeek = 604800;

/// GPS time origin: 1980-01-06 00:00:00 UTC.
QDateTime gpsEpochOrigin()
{
    return QDateTime(QDate(1980, 1, 6), QTime(0, 0, 0), QTimeZone::UTC);
}

/// BeiDou time origin: 2006-01-01 00:00:00 UTC, which is GPS week 1356.
QDateTime beidouEpochOrigin()
{
    return QDateTime(QDate(2006, 1, 1), QTime(0, 0, 0), QTimeZone::UTC);
}

/// GLONASS counts days in four-year intervals starting 1996-01-01.
QDateTime glonassEpochOrigin(int fourYearInterval)
{
    return QDateTime(QDate(1996 + (4 * (fourYearInterval - 1)), 1, 1), QTime(0, 0, 0), QTimeZone::UTC);
}

/// The broadcast week is truncated (10 bits for GPS/QZSS), so it has to be lifted into the
/// current era using the wall clock.
int unrollWeek(int truncatedWeek, int bits)
{
    if (bits >= 16) {
        return truncatedWeek;   // BeiDou (13 bit) and Galileo (12 bit, already offset) do not wrap
    }
    const int span = 1 << bits;
    const qint64 nowWeeks = gpsEpochOrigin().secsTo(QDateTime::currentDateTimeUtc()) / kSecondsPerWeek;
    const int rollovers = static_cast<int>(std::llround(static_cast<double>(nowWeeks - truncatedWeek) / span));
    return truncatedWeek + (rollovers * span);
}

/// URA/SISA index to metres, per the GPS interface specification.
double uraIndexToMetres(int index)
{
    static const double table[16] = {
        2.0, 2.8, 4.0, 5.7, 8.0, 11.3, 16.0, 32.0,
        64.0, 128.0, 256.0, 512.0, 1024.0, 2048.0, 4096.0, 8192.0
    };
    if ((index < 0) || (index > 15)) {
        return 8192.0;
    }
    return table[index];
}

} // namespace

RinexNavWriter::RinexNavWriter(const QString &filePath, QObject *parent)
    : QObject(parent)
    , _filePath(filePath)
{
    (void) _openFile();
}

RinexNavWriter::~RinexNavWriter()
{
    finish();
}

QString RinexNavWriter::pathForObsFile(const QString &obsPath)
{
    const QFileInfo info(obsPath);
    return info.dir().filePath(info.completeBaseName() + QStringLiteral(".nav"));
}

bool RinexNavWriter::_openFile()
{
    const QFileInfo info(_filePath);
    if (!info.dir().exists()) {
        (void) QDir().mkpath(info.dir().absolutePath());
    }

    _file.setFileName(_filePath);
    if (!_file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        qCWarning(RinexNavWriterLog) << "Cannot open RINEX nav file" << _filePath << _file.errorString();
        return false;
    }

    qCDebug(RinexNavWriterLog) << "Writing RINEX navigation to" << _filePath;
    return true;
}

void RinexNavWriter::_write(const QString &line)
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

QString RinexNavWriter::_headerLine(const QString &content, const QString &label)
{
    return content.leftJustified(60, QLatin1Char(' '), true) + label.leftJustified(20, QLatin1Char(' '), true);
}

QString RinexNavWriter::_num(double value)
{
    // Fortran D19.12: a leading sign, one digit, twelve decimals and a two digit D exponent.
    // Qt writes C-style "e+05", so the mantissa is shifted down by one decade and the exponent
    // relabelled, giving the 0.xxx form RINEX readers expect.
    if (!std::isfinite(value)) {
        value = 0.0;
    }

    QString text = QString::asprintf("%19.12E", value);
    const int ePos = text.indexOf(QLatin1Char('E'));
    if (ePos < 0) {
        return text.rightJustified(19);
    }

    bool ok = false;
    const int exponent = text.mid(ePos + 1).toInt(&ok);
    if (!ok) {
        return text.rightJustified(19);
    }

    const double mantissa = text.left(ePos).toDouble() / 10.0;
    const QString sign = (mantissa < 0.0) ? QStringLiteral("-") : QStringLiteral(" ");
    const QString digits = QString::asprintf("%.12f", std::abs(mantissa)).mid(1);   // drop the "0"
    return QStringLiteral("%1%2D%3%4")
        .arg(sign)
        .arg(digits)
        .arg((exponent + 1) < 0 ? QStringLiteral("-") : QStringLiteral("+"))
        .arg(std::abs(exponent + 1), 2, 10, QLatin1Char('0'));
}

QDateTime RinexNavWriter::_referenceTime(const RtcmEphemeris &e)
{
    switch (e.system) {
    case 'G':
    case 'J': {
        const int week = unrollWeek(e.week, 10);
        return gpsEpochOrigin().addSecs((static_cast<qint64>(week) * kSecondsPerWeek)
                                        + static_cast<qint64>(e.toc));
    }
    case 'E': {
        // Already lifted by +1024 in the decoder, and Galileo System Time equals GPS time.
        return gpsEpochOrigin().addSecs((static_cast<qint64>(e.week) * kSecondsPerWeek)
                                        + static_cast<qint64>(e.toc));
    }
    case 'C': {
        // RINEX records BeiDou epochs in BeiDou time, so no GPS offset is applied here.
        return beidouEpochOrigin().addSecs((static_cast<qint64>(e.week) * kSecondsPerWeek)
                                           + static_cast<qint64>(e.toc));
    }
    case 'R': {
        // tb counts 15 minute intervals of the Moscow day, and RINEX wants UTC (Moscow - 3 h).
        const qint64 moscowSeconds = static_cast<qint64>(e.intervalIndex) * 900;
        if (e.fourYearInterval > 0) {
            return glonassEpochOrigin(e.fourYearInterval)
                .addDays(e.dayNumber - 1)
                .addSecs(moscowSeconds - 10800);
        }
        // No four-year interval broadcast: fall back to today's date.
        const QDateTime now = QDateTime::currentDateTimeUtc();
        return QDateTime(now.date(), QTime(0, 0, 0), QTimeZone::UTC).addSecs(moscowSeconds - 10800);
    }
    default:
        return QDateTime::currentDateTimeUtc();
    }
}

void RinexNavWriter::_writeHeader()
{
    _write(QStringLiteral("%1%2%3%4")
               .arg(QString(kRinexVersion).rightJustified(9))
               .arg(QString(), 11)
               .arg(QStringLiteral("N: GNSS NAV DATA").leftJustified(20))
               .arg(QStringLiteral("M: MIXED").leftJustified(20))
           + QStringLiteral("RINEX VERSION / TYPE"));

    const QString runDate = QDateTime::currentDateTimeUtc().toString(QStringLiteral("yyyyMMdd hhmmss UTC"));
    _write(_headerLine(QStringLiteral("QGroundControl").leftJustified(20)
                           + QStringLiteral("QGroundControl").leftJustified(20)
                           + runDate,
                       QStringLiteral("PGM / RUN BY / DATE")));
    _write(_headerLine(QStringLiteral("Broadcast ephemeris decoded from an RTCM3 stream by QGroundControl."),
                       QStringLiteral("COMMENT")));
    _write(_headerLine(QString(), QStringLiteral("END OF HEADER")));

    _headerWritten = true;
}

void RinexNavWriter::_writeKeplerian(const RtcmEphemeris &e)
{
    const QDateTime time = _referenceTime(e);
    _write(QStringLiteral("%1%2 %3 %4 %5 %6 %7 %8%9%10")
               .arg(QChar::fromLatin1(e.system))
               .arg(e.prn, 2, 10, QLatin1Char('0'))
               .arg(time.date().year(), 4)
               .arg(time.date().month(), 2, 10, QLatin1Char('0'))
               .arg(time.date().day(), 2, 10, QLatin1Char('0'))
               .arg(time.time().hour(), 2, 10, QLatin1Char('0'))
               .arg(time.time().minute(), 2, 10, QLatin1Char('0'))
               .arg(time.time().second(), 2, 10, QLatin1Char('0'))
               .arg(_num(e.af0)).arg(_num(e.af1)) + _num(e.af2));

    const QString indent(4, QLatin1Char(' '));

    _write(indent + _num(e.iode) + _num(e.crs) + _num(e.deltaN) + _num(e.m0));
    _write(indent + _num(e.cuc) + _num(e.eccentricity) + _num(e.cus) + _num(e.sqrtA));
    _write(indent + _num(e.toe) + _num(e.cic) + _num(e.omega0) + _num(e.cis));
    _write(indent + _num(e.i0) + _num(e.crc) + _num(e.omega) + _num(e.omegaDot));

    // The remaining lines carry per-constellation fields in different slots.
    switch (e.system) {
    case 'E':
        _write(indent + _num(e.idot) + _num(e.codesOnL2) + _num(e.week) + _num(0.0));
        _write(indent + _num(uraIndexToMetres(e.uraIndex)) + _num(e.health) + _num(e.tgd) + _num(e.tgd2));
        _write(indent + _num(e.toe) + _num(0.0) + _num(0.0) + _num(0.0));
        break;
    case 'C':
        _write(indent + _num(e.idot) + _num(0.0) + _num(e.week) + _num(0.0));
        _write(indent + _num(uraIndexToMetres(e.uraIndex)) + _num(e.health) + _num(e.tgd) + _num(e.tgd2));
        _write(indent + _num(e.toe) + _num(e.iodc) + _num(0.0) + _num(0.0));
        break;
    default:    // GPS and QZSS share a layout
        _write(indent + _num(e.idot) + _num(e.codesOnL2) + _num(unrollWeek(e.week, 10)) + _num(0.0));
        _write(indent + _num(uraIndexToMetres(e.uraIndex)) + _num(e.health) + _num(e.tgd) + _num(e.iodc));
        _write(indent + _num(e.toe) + _num(e.fitInterval) + _num(0.0) + _num(0.0));
        break;
    }

    ++_recordsWritten;
}

void RinexNavWriter::_writeGlonass(const RtcmEphemeris &e)
{
    const QDateTime time = _referenceTime(e);
    // RINEX stores the GLONASS clock terms as -TauN and +GammaN.
    _write(QStringLiteral("R%1 %2 %3 %4 %5 %6 %7%8%9%10")
               .arg(e.prn, 2, 10, QLatin1Char('0'))
               .arg(time.date().year(), 4)
               .arg(time.date().month(), 2, 10, QLatin1Char('0'))
               .arg(time.date().day(), 2, 10, QLatin1Char('0'))
               .arg(time.time().hour(), 2, 10, QLatin1Char('0'))
               .arg(time.time().minute(), 2, 10, QLatin1Char('0'))
               .arg(time.time().second(), 2, 10, QLatin1Char('0'))
               .arg(_num(-e.tauN)).arg(_num(e.gammaN)) + _num(e.frameTime));

    const QString indent(4, QLatin1Char(' '));
    // Positions in km, velocities km/s, accelerations km/s^2.
    _write(indent + _num(e.positionX / 1000.0) + _num(e.velocityX / 1000.0)
           + _num(e.accelerationX / 1000.0) + _num(0.0));
    _write(indent + _num(e.positionY / 1000.0) + _num(e.velocityY / 1000.0)
           + _num(e.accelerationY / 1000.0) + _num(e.frequencyChannel));
    _write(indent + _num(e.positionZ / 1000.0) + _num(e.velocityZ / 1000.0)
           + _num(e.accelerationZ / 1000.0) + _num(e.ageOfData));

    ++_recordsWritten;
}

void RinexNavWriter::writeEphemeris(const RtcmEphemeris &ephemeris)
{
    if (!_file.isOpen()) {
        return;
    }

    // Satellites rebroadcast the same set continuously; only the first copy belongs in the file.
    const QByteArray key = ephemeris.key();
    if (_seen.contains(key)) {
        return;
    }
    _seen.insert(key);

    if (!_headerWritten) {
        _writeHeader();
    }

    if (ephemeris.system == 'R') {
        _writeGlonass(ephemeris);
    } else {
        _writeKeplerian(ephemeris);
    }

    (void) _file.flush();
    emit bytesWrittenChanged(_totalBytes);
}

void RinexNavWriter::finish()
{
    if (!_file.isOpen()) {
        return;
    }

    // An empty header is still a valid RINEX file and tells the user the stream carried no
    // ephemeris, which is more useful than a zero byte file.
    if (!_headerWritten) {
        _writeHeader();
    }

    (void) _file.flush();
    _file.close();
    emit bytesWrittenChanged(_totalBytes);

    qCDebug(RinexNavWriterLog) << "Closed RINEX nav file" << _filePath
                               << "records" << _recordsWritten << "bytes" << _totalBytes;
}
