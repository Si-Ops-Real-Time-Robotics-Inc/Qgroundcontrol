/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "DropPointComplexItem.h"
#include "JsonHelper.h"
#include "MissionController.h"
#include "MissionItem.h"
#include "PlanMasterController.h"
#include "QGCApplication.h"

#include <QtCore/QJsonArray>
#include <QtCore/QJsonObject>

const QString DropPointComplexItem::name(DropPointComplexItem::tr("Drop Point"));

DropPointComplexItem::DropPointComplexItem(PlanMasterController* masterController, bool flyView)
    : ComplexMissionItem  (masterController, flyView)
    , _metaDataMap        (FactMetaData::createMapFromJsonFile(QStringLiteral(":/json/DropPoint.SettingsGroup.json"), this))
    , _altitudeFact       (settingsGroup, _metaDataMap[altitudeName])
    , _relayNumberFact    (settingsGroup, _metaDataMap[relayNumberName])
    , _positionFact       (settingsGroup, _metaDataMap[positionName])
{
    _editorQml = "qrc:/qml/QGroundControl/Controls/DropPointItemEditor.qml";

    // A drop point is complete as soon as it exists, there is nothing left for the user to define
    _isIncomplete = false;

    connect(&_altitudeFact,    &Fact::valueChanged, this, &DropPointComplexItem::_setDirty);
    connect(&_relayNumberFact, &Fact::valueChanged, this, &DropPointComplexItem::_setDirty);
    connect(&_positionFact,    &Fact::valueChanged, this, &DropPointComplexItem::_setDirty);

    // The map label is built from the position, so it has to be told when that changes
    connect(&_positionFact, &Fact::rawValueChanged, this, &DropPointComplexItem::abbreviationChanged);

    connect(&_altitudeFact, &Fact::rawValueChanged, this, &DropPointComplexItem::_amslEntryAltChanged);
    connect(&_altitudeFact, &Fact::rawValueChanged, this, &DropPointComplexItem::_amslExitAltChanged);
    connect(&_altitudeFact, &Fact::rawValueChanged, this, &DropPointComplexItem::minAMSLAltitudeChanged);
    connect(&_altitudeFact, &Fact::rawValueChanged, this, &DropPointComplexItem::maxAMSLAltitudeChanged);

    setDirty(false);
}

void DropPointComplexItem::_setDirty(void)
{
    setDirty(true);
}

void DropPointComplexItem::setDirty(bool dirty)
{
    if (_dirty != dirty) {
        _dirty = dirty;
        emit dirtyChanged(_dirty);
    }
}

void DropPointComplexItem::setCoordinate(const QGeoCoordinate& coordinate)
{
    if (_coordinate != coordinate) {
        _coordinate = coordinate;
        emit coordinateChanged(_coordinate);
        emit exitCoordinateChanged(_coordinate);
        setDirty(true);
    }
}

void DropPointComplexItem::setSequenceNumber(int sequenceNumber)
{
    if (_sequenceNumber != sequenceNumber) {
        _sequenceNumber = sequenceNumber;
        emit sequenceNumberChanged(sequenceNumber);
        emit lastSequenceNumberChanged(lastSequenceNumber());
    }
}

double DropPointComplexItem::amslEntryAlt(void) const
{
    // Altitude is relative to home
    return _altitudeFact.rawValue().toDouble() + _missionController->plannedHomePosition().altitude();
}

double DropPointComplexItem::additionalTimeDelay(void) const
{
    // The vehicle covers no distance here, it just sits on the point for the two holds
    double position = _positionFact.rawValue().toInt();
    return entryHoldSeconds + (position + exitHoldExtraSeconds);
}

void DropPointComplexItem::applyNewAltitude(double newAltitude)
{
    _altitudeFact.setRawValue(newAltitude);
}

void DropPointComplexItem::appendMissionItems(QList<MissionItem*>& items, QObject* missionItemParent)
{
    // Keep the item count here in sync with itemCount, which lastSequenceNumber reports without building.

    int     seqNum          = _sequenceNumber;
    int     position        = _positionFact.rawValue().toInt();
    double  altitude        = _altitudeFact.rawValue().toDouble();
    // Position N cycles the relay N times over N seconds, then holds for N + 0.5 seconds
    int     relayCycles     = position;
    double  relayCycleSecs  = position;
    double  exitHoldSecs    = relayCycleSecs + exitHoldExtraSeconds;

    // Settle over the drop point
    items.append(new MissionItem(seqNum++,
                                 MAV_CMD_NAV_WAYPOINT,
                                 MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                 entryHoldSeconds,              // Hold time
                                 0.0,                           // No acceptance radius specified
                                 0.0,                           // Pass through waypoint
                                 qQNaN(),                       // Yaw unchanged
                                 _coordinate.latitude(),
                                 _coordinate.longitude(),
                                 altitude,
                                 true,                          // autoContinue
                                 false,                         // isCurrentItem
                                 missionItemParent));

    // Cycle the relay to drop the marker
    items.append(new MissionItem(seqNum++,
                                 MAV_CMD_DO_REPEAT_RELAY,
                                 MAV_FRAME_MISSION,
                                 _relayNumberFact.rawValue().toInt(),   // Relay number
                                 relayCycles,                           // Cycle count
                                 relayCycleSecs,                        // Cycle time (seconds)
                                 0, 0, 0, 0,                            // Unused
                                 true,                                  // autoContinue
                                 false,                                 // isCurrentItem
                                 missionItemParent));

    // Hold on the point while the relay does its work
    items.append(new MissionItem(seqNum++,
                                 MAV_CMD_NAV_WAYPOINT,
                                 MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                 exitHoldSecs,                  // Hold time
                                 0.0,                           // No acceptance radius specified
                                 0.0,                           // Pass through waypoint
                                 qQNaN(),                       // Yaw unchanged
                                 _coordinate.latitude(),
                                 _coordinate.longitude(),
                                 altitude,
                                 true,                          // autoContinue
                                 false,                         // isCurrentItem
                                 missionItemParent));
}

void DropPointComplexItem::save(QJsonArray& missionItems)
{
    QJsonObject saveObject;

    saveObject[JsonHelper::jsonVersionKey] =                    1;
    saveObject[VisualMissionItem::jsonTypeKey] =                VisualMissionItem::jsonTypeComplexItemValue;
    saveObject[ComplexMissionItem::jsonComplexItemTypeKey] =    jsonComplexItemTypeValue;
    saveObject[_jsonAltitudeKey] =                              _altitudeFact.rawValue().toDouble();
    saveObject[_jsonRelayNumberKey] =                           _relayNumberFact.rawValue().toInt();
    saveObject[_jsonPositionKey] =                              _positionFact.rawValue().toInt();

    QJsonValue coordinateJson;
    JsonHelper::saveGeoCoordinate(_coordinate, false /* writeAltitude */, coordinateJson);
    saveObject[_jsonCoordinateKey] = coordinateJson;

    missionItems.append(saveObject);
}

bool DropPointComplexItem::load(const QJsonObject& complexObject, int sequenceNumber, QString& errorString)
{
    QList<JsonHelper::KeyValidateInfo> keyInfoList = {
        { JsonHelper::jsonVersionKey,                   QJsonValue::Double, true },
        { VisualMissionItem::jsonTypeKey,               QJsonValue::String, true },
        { ComplexMissionItem::jsonComplexItemTypeKey,   QJsonValue::String, true },
        { _jsonCoordinateKey,                           QJsonValue::Array,  true },
        { _jsonAltitudeKey,                             QJsonValue::Double, true },
        { _jsonRelayNumberKey,                          QJsonValue::Double, true },
        { _jsonPositionKey,                             QJsonValue::Double, true },
    };
    if (!JsonHelper::validateKeys(complexObject, keyInfoList, errorString)) {
        return false;
    }

    QString itemType    = complexObject[VisualMissionItem::jsonTypeKey].toString();
    QString complexType = complexObject[ComplexMissionItem::jsonComplexItemTypeKey].toString();
    if (itemType != VisualMissionItem::jsonTypeComplexItemValue || complexType != jsonComplexItemTypeValue) {
        errorString = tr("%1 does not support loading this complex mission item type: %2:%3").arg(qgcApp()->applicationName()).arg(itemType).arg(complexType);
        return false;
    }

    int version = complexObject[JsonHelper::jsonVersionKey].toInt();
    if (version != 1) {
        errorString = tr("Drop Point items do not support version %1").arg(version);
        return false;
    }

    if (!JsonHelper::loadGeoCoordinate(complexObject[_jsonCoordinateKey], false /* altitudeRequired */, _coordinate, errorString)) {
        return false;
    }

    setSequenceNumber(sequenceNumber);

    _altitudeFact.setRawValue   (complexObject[_jsonAltitudeKey].toDouble());
    _relayNumberFact.setRawValue(complexObject[_jsonRelayNumberKey].toInt());
    _positionFact.setRawValue   (complexObject[_jsonPositionKey].toInt());

    _isIncomplete = false;

    emit coordinateChanged(_coordinate);
    emit exitCoordinateChanged(_coordinate);

    setDirty(false);

    return true;
}
