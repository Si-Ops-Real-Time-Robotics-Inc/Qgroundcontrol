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
#include <QtCore/QList>
#include <QtCore/QLoggingCategory>
#include <QtCore/QMap>
#include <QtCore/QObject>
#include <QtCore/QString>

#include "RtcmObsDecoder.h"

Q_DECLARE_LOGGING_CATEGORY(RinexObsWriterLog)

/// Streams decoded RTCM observations into a RINEX 3.04 observation file as they arrive.
///
/// The RINEX header has to declare the observation types each constellation will use, and that is
/// only known once data has been seen. The first epochs are therefore buffered (kHeaderEpochs) to
/// collect the code list, then the header is written and the buffer drained. Anything flushed
/// before that many epochs arrive - a short session - still writes a valid file.
class RinexObsWriter : public QObject
{
    Q_OBJECT

public:
    /// @param filePath explicit output path, or empty to auto-name under the app save folder.
    explicit RinexObsWriter(const QString &filePath, QObject *parent = nullptr);
    ~RinexObsWriter() override;

    QString filePath() const { return _filePath; }
    bool isOpen() const { return _file.isOpen(); }
    quint64 epochsWritten() const { return _epochsWritten; }

    /// Build the default log path: <save folder>/RTCMLogs/rtcm_<timestamp>.obs
    static QString defaultFilePath();

public slots:
    /// Station metadata for the header. Ignored once the header has been written.
    void setStationInfo(const RtcmStationInfo &info);
    /// GLONASS slot -> frequency channel, for the "GLONASS SLOT / FRQ #" header record.
    void setGlonassChannels(const QHash<int, int> &channels);
    void writeEpoch(const RtcmObsEpoch &epoch);
    /// Write the header and drain the buffer if that has not happened yet, then close.
    void finish();

signals:
    void bytesWrittenChanged(quint64 totalBytes);

private:
    bool _openFile();
    void _collectObsTypes(const RtcmObsEpoch &epoch);
    void _writeHeader(const RtcmObsEpoch &firstEpoch);
    void _writeEpochRecord(const RtcmObsEpoch &epoch);
    void _write(const QString &line);
    QDateTime _epochToGpsDateTime(qint64 gpsTowMs) const;

    static QString _headerLine(const QString &content, const QString &label);

    QString _filePath;
    QFile _file;
    quint64 _totalBytes = 0;
    quint64 _epochsWritten = 0;

    RtcmStationInfo _station;
    QHash<int, int> _glonassChannels;

    bool _headerWritten = false;
    QList<RtcmObsEpoch> _pending;              ///< epochs held until the header is written
    QMap<char, QList<QString>> _obsTypes;      ///< system letter -> ordered RINEX codes ("C1C", ...)

    qint64 _gpsWeek = -1;                      ///< resolved once, from the wall clock
    qint64 _firstTowMs = -1;
    qint64 _lastTowMs = -1;
    double _interval = 0.0;

    /// Buffer this many epochs before committing to a header, so a satellite that shows up a
    /// second late with an extra signal is still represented in the observation type list.
    static constexpr int kHeaderEpochs = 5;
    /// Keep waiting past that if the base position has not arrived yet: 1005/1006 is typically
    /// only sent every few seconds, and a header without APPROX POSITION XYZ is of little use for
    /// post-processing. Give up after this many epochs so a stream that never sends one still
    /// produces a file.
    static constexpr int kMaxHeaderWaitEpochs = 60;
    /// GPS time has run this many seconds ahead of UTC since 2017-01-01.
    static constexpr qint64 kGpsUtcLeapSeconds = 18;
};
