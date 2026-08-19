/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include <QtCore/QLoggingCategory>

#include "FactGroup.h"

Q_DECLARE_LOGGING_CATEGORY(NetworkRTCMFactGroupLog)

/// Runtime status of the network RTCM source, exposed to QML.
class NetworkRTCMFactGroup : public FactGroup
{
    Q_OBJECT
    Q_PROPERTY(Fact *connected      READ connected      CONSTANT)
    Q_PROPERTY(Fact *sourceType     READ sourceType     CONSTANT)
    Q_PROPERTY(Fact *mountpoint     READ mountpoint     CONSTANT)
    Q_PROPERTY(Fact *bytesPerSecond READ bytesPerSecond CONSTANT)
    Q_PROPERTY(Fact *rtcmValid      READ rtcmValid      CONSTANT)
    Q_PROPERTY(Fact *discardedBytes READ discardedBytes CONSTANT)
    Q_PROPERTY(Fact *baseValid      READ baseValid      CONSTANT)
    Q_PROPERTY(Fact *baseLatitude   READ baseLatitude   CONSTANT)
    Q_PROPERTY(Fact *baseLongitude  READ baseLongitude  CONSTANT)
    Q_PROPERTY(Fact *baseAltitude   READ baseAltitude   CONSTANT)
    Q_PROPERTY(Fact *baseStationId  READ baseStationId  CONSTANT)
    Q_PROPERTY(Fact *logFileName    READ logFileName    CONSTANT)
    Q_PROPERTY(Fact *logBytes       READ logBytes       CONSTANT)

public:
    explicit NetworkRTCMFactGroup(QObject *parent = nullptr);
    ~NetworkRTCMFactGroup();

    Fact *connected() { return &_connectedFact; }
    Fact *sourceType() { return &_sourceTypeFact; }
    Fact *mountpoint() { return &_mountpointFact; }
    Fact *bytesPerSecond() { return &_bytesPerSecondFact; }
    Fact *rtcmValid() { return &_rtcmValidFact; }
    Fact *discardedBytes() { return &_discardedBytesFact; }
    Fact *baseValid() { return &_baseValidFact; }
    Fact *baseLatitude() { return &_baseLatitudeFact; }
    Fact *baseLongitude() { return &_baseLongitudeFact; }
    Fact *baseAltitude() { return &_baseAltitudeFact; }
    Fact *baseStationId() { return &_baseStationIdFact; }
    Fact *logFileName() { return &_logFileNameFact; }
    Fact *logBytes() { return &_logBytesFact; }

private:
    Fact _connectedFact = Fact(0, QStringLiteral("connected"), FactMetaData::valueTypeBool);
    Fact _sourceTypeFact = Fact(0, QStringLiteral("sourceType"), FactMetaData::valueTypeString);
    Fact _mountpointFact = Fact(0, QStringLiteral("mountpoint"), FactMetaData::valueTypeString);
    Fact _bytesPerSecondFact = Fact(0, QStringLiteral("bytesPerSecond"), FactMetaData::valueTypeDouble);
    Fact _rtcmValidFact = Fact(0, QStringLiteral("rtcmValid"), FactMetaData::valueTypeBool);
    Fact _discardedBytesFact = Fact(0, QStringLiteral("discardedBytes"), FactMetaData::valueTypeDouble);
    Fact _baseValidFact = Fact(0, QStringLiteral("baseValid"), FactMetaData::valueTypeBool);
    Fact _baseLatitudeFact = Fact(0, QStringLiteral("baseLatitude"), FactMetaData::valueTypeDouble);
    Fact _baseLongitudeFact = Fact(0, QStringLiteral("baseLongitude"), FactMetaData::valueTypeDouble);
    Fact _baseAltitudeFact = Fact(0, QStringLiteral("baseAltitude"), FactMetaData::valueTypeDouble);
    Fact _baseStationIdFact = Fact(0, QStringLiteral("baseStationId"), FactMetaData::valueTypeInt32);
    Fact _logFileNameFact = Fact(0, QStringLiteral("logFileName"), FactMetaData::valueTypeString);
    Fact _logBytesFact = Fact(0, QStringLiteral("logBytes"), FactMetaData::valueTypeDouble);
};
