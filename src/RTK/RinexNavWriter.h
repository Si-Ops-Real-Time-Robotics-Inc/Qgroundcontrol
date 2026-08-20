/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include <QtCore/QDateTime>
#include <QtCore/QFile>
#include <QtCore/QLoggingCategory>
#include <QtCore/QObject>
#include <QtCore/QSet>
#include <QtCore/QString>

#include "RtcmNavDecoder.h"

Q_DECLARE_LOGGING_CATEGORY(RinexNavWriterLog)

/// Writes a RINEX 3.04 navigation file as broadcast ephemeris arrives.
///
/// Unlike the observation file the header depends on nothing data-derived, so it is written as
/// soon as the file opens. Satellites rebroadcast the same ephemeris every few seconds, so records
/// are de-duplicated on satellite + reference time + issue of data.
class RinexNavWriter : public QObject
{
    Q_OBJECT

public:
    /// @param filePath explicit output path, or empty to sit alongside the observation file.
    explicit RinexNavWriter(const QString &filePath, QObject *parent = nullptr);
    ~RinexNavWriter() override;

    QString filePath() const { return _filePath; }
    bool isOpen() const { return _file.isOpen(); }
    quint64 recordsWritten() const { return _recordsWritten; }

    /// Derive the .nav path that pairs with an observation file path.
    static QString pathForObsFile(const QString &obsPath);

public slots:
    void writeEphemeris(const RtcmEphemeris &ephemeris);
    void finish();

signals:
    void bytesWrittenChanged(quint64 totalBytes);

private:
    bool _openFile();
    void _writeHeader();
    void _write(const QString &line);
    void _writeKeplerian(const RtcmEphemeris &e);
    void _writeGlonass(const RtcmEphemeris &e);

    /// RINEX numbers are Fortran D19.12: a signed mantissa with 12 decimals and a D exponent.
    static QString _num(double value);
    static QString _headerLine(const QString &content, const QString &label);
    /// Calendar time of an ephemeris reference, in that constellation's own time system.
    static QDateTime _referenceTime(const RtcmEphemeris &e);

    QString _filePath;
    QFile _file;
    quint64 _totalBytes = 0;
    quint64 _recordsWritten = 0;
    bool _headerWritten = false;
    QSet<QByteArray> _seen;

    /// GPS time has run this many seconds ahead of UTC since 2017-01-01.
    static constexpr qint64 kGpsUtcLeapSeconds = 18;
};
