/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "NetworkRTCMFactGroup.h"
#include "QGCLoggingCategory.h"

QGC_LOGGING_CATEGORY(NetworkRTCMFactGroupLog, "qgc.rtk.networkrtcmfactgroup")

NetworkRTCMFactGroup::NetworkRTCMFactGroup(QObject *parent)
    : FactGroup(1000, QStringLiteral(":/json/NetworkRTCMFact.json"), parent)
{
    _addFact(&_connectedFact);
    _addFact(&_sourceTypeFact);
    _addFact(&_mountpointFact);
    _addFact(&_bytesPerSecondFact);
    _addFact(&_baseValidFact);
    _addFact(&_baseLatitudeFact);
    _addFact(&_baseLongitudeFact);
    _addFact(&_baseAltitudeFact);
    _addFact(&_baseStationIdFact);
    _addFact(&_logFileNameFact);
    _addFact(&_logBytesFact);
}

NetworkRTCMFactGroup::~NetworkRTCMFactGroup()
{
}
