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
#include <QtCore/QHash>
#include <QtCore/QList>
#include <QtCore/QLoggingCategory>
#include <QtCore/QMap>
#include <QtCore/QObject>
#include <QtCore/QString>

Q_DECLARE_LOGGING_CATEGORY(RtcmObsDecoderLog)

/// One observable set for a single satellite / signal, in RINEX units.
struct RtcmObservation
{
    double pseudorange = 0.0;   ///< metres, 0 if absent
    double carrierPhase = 0.0;  ///< full cycles, 0 if absent
    double doppler = 0.0;       ///< Hz, 0 if absent
    double snr = 0.0;           ///< dB-Hz, 0 if absent
    bool havePseudorange = false;
    bool haveCarrierPhase = false;
    bool haveDoppler = false;
    bool haveSnr = false;
    int lli = 0;                ///< loss-of-lock indicator (bit 1 = half-cycle ambiguity)
};

/// All observables for one satellite at one epoch, keyed by RINEX signal code ("1C", "2W", ...).
struct RtcmSatObservations
{
    char system = 0;            ///< RINEX system letter: G R E C J S
    int prn = 0;
    QMap<QString, RtcmObservation> observations;
};

/// A complete measurement epoch, assembled from every constellation's MSM message.
struct RtcmObsEpoch
{
    qint64 gpsTowMs = -1;                 ///< GPS time of week, milliseconds
    QList<RtcmSatObservations> satellites;
};

/// Station metadata needed for the RINEX header, gathered from 1005/1006/1008/1033.
struct RtcmStationInfo
{
    bool havePosition = false;
    double ecefX = 0.0;
    double ecefY = 0.0;
    double ecefZ = 0.0;
    double antennaHeight = 0.0;   ///< from 1006 only
    int stationId = -1;
    QString antennaDescriptor;
    QString antennaSerial;
    QString receiverType;
    QString receiverSerial;
    QString receiverVersion;
};

Q_DECLARE_METATYPE(RtcmObsEpoch)
Q_DECLARE_METATYPE(RtcmStationInfo)

/// Decodes RTCM3 MSM4/5/6/7 observation messages into RINEX-ready epochs, plus the station
/// metadata the RINEX header needs. Feed it the validated frames from RtcmStreamParser.
///
/// Epoch assembly: each constellation sends its own MSM message for the same instant. Messages
/// accumulate into the open epoch and it is emitted when the "multiple message" bit (DF393)
/// clears or the epoch time moves on, whichever happens first.
class RtcmObsDecoder : public QObject
{
    Q_OBJECT

public:
    explicit RtcmObsDecoder(QObject *parent = nullptr);
    ~RtcmObsDecoder() override;

    const RtcmStationInfo &stationInfo() const { return _station; }
    /// GLONASS frequency channel number per slot, harvested from MSM5/MSM7 extended satellite info.
    const QHash<int, int> &glonassChannels() const { return _glonassChannels; }
    quint64 decodedEpochs() const { return _decodedEpochs; }

public slots:
    /// Feed one or more complete, already CRC-validated RTCM3 frames.
    void addFrames(const QByteArray &frames);
    /// Emit any partially assembled epoch (call when the stream stops).
    void flush();
    /// Drop all state.
    void reset();

signals:
    void epochReady(const RtcmObsEpoch &epoch);
    void stationInfoChanged(const RtcmStationInfo &info);

private:
    void _decodeFrame(const quint8 *payload, int payloadLen);
    void _decodeMsm(const quint8 *payload, int payloadLen, int messageNumber);
    void _decodeStationArp(const quint8 *payload, int payloadLen, int messageNumber);
    void _decodeAntennaDescriptor(const quint8 *payload, int payloadLen, int messageNumber);
    void _startEpoch(qint64 gpsTowMs);
    void _emitEpoch();

    RtcmStationInfo _station;
    QHash<int, int> _glonassChannels;

    RtcmObsEpoch _epoch;
    bool _epochOpen = false;
    quint64 _decodedEpochs = 0;
    quint64 _unsupportedMsm = 0;

    /// GPS - BeiDou system time offset. BDT started 14 s behind GPS time and both are continuous.
    static constexpr qint64 kBeidouGpsOffsetMs = 14000;
    static constexpr qint64 kMsPerWeek = 604800000;
};
