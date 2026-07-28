/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include "ComplexMissionItem.h"
#include "SettingsFact.h"

class PlanMasterController;
class MissionItem;

/// A single point at which the vehicle stops and cycles a relay to drop a marker. It is one item to plan with
/// but expands to three mission items: a waypoint to settle on the point, the relay cycle, and a waypoint to
/// hold on while the relay runs. This is the same action an Auto Marker performs at each of its markers.
class DropPointComplexItem : public ComplexMissionItem
{
    Q_OBJECT

public:
    /// @param flyView true: Created for use in the Fly View, false: Created for use in the Plan View
    DropPointComplexItem(PlanMasterController* masterController, bool flyView);

    Q_PROPERTY(Fact* altitude    READ altitude    CONSTANT)
    Q_PROPERTY(Fact* relayNumber READ relayNumber CONSTANT)
    Q_PROPERTY(Fact* position    READ position    CONSTANT)

    Fact* altitude    (void) { return &_altitudeFact; }
    Fact* relayNumber (void) { return &_relayNumberFact; }
    Fact* position    (void) { return &_positionFact; }

    // Overrides from ComplexMissionItem
    QString patternName         (void) const final { return name; }
    double  minAMSLAltitude     (void) const final { return amslEntryAlt(); }
    double  maxAMSLAltitude     (void) const final { return amslEntryAlt(); }
    double  complexDistance     (void) const final { return 0; }    // Everything happens at the one point
    bool    load                (const QJsonObject& complexObject, int sequenceNumber, QString& errorString) final;
    double  greatestDistanceTo  (const QGeoCoordinate& other) const final { return _coordinate.distanceTo(other); }
    bool    isSingleItem        (void) const final { return true; }

    // Overrides from VisualMissionItem
    bool            dirty                   (void) const final { return _dirty; }
    void            setDirty                (bool dirty) final;
    bool            isSimpleItem            (void) const final { return false; }
    bool            isStandaloneCoordinate  (void) const final { return false; }
    bool            specifiesCoordinate     (void) const final { return true; }
    bool            specifiesAltitudeOnly   (void) const final { return false; }
    QString         commandDescription      (void) const final { return tr("Drop Point"); }
    QString         commandName             (void) const final { return tr("Drop Point"); }
    /// "D" plus the position, so the drop points can be told apart on the map
    QString         abbreviation            (void) const final { return tr("D%1").arg(_positionFact.rawValue().toInt()); }
    QGeoCoordinate  coordinate              (void) const final { return _coordinate; }
    void            setCoordinate           (const QGeoCoordinate& coordinate) final;
    QGeoCoordinate  exitCoordinate          (void) const final { return _coordinate; }
    bool            exitCoordinateSameAsEntry(void) const final { return true; }
    double          amslEntryAlt            (void) const final;
    double          amslExitAlt             (void) const final { return amslEntryAlt(); }
    int             sequenceNumber          (void) const final { return _sequenceNumber; }
    void            setSequenceNumber       (int sequenceNumber) final;
    int             lastSequenceNumber      (void) const final { return _sequenceNumber + itemCount - 1; }
    double          specifiedFlightSpeed    (void) final { return qQNaN(); }
    double          specifiedGimbalYaw      (void) final { return qQNaN(); }
    double          specifiedGimbalPitch    (void) final { return qQNaN(); }
    double          additionalTimeDelay     (void) const final;
    void            appendMissionItems      (QList<MissionItem*>& items, QObject* missionItemParent) final;
    void            applyNewAltitude        (double newAltitude) final;
    void            save                    (QJsonArray& missionItems) final;
    QString         mapVisualQML            (void) const final { return QStringLiteral("DropPointMapVisual.qml"); }

    static const QString name;

    static constexpr const char* jsonComplexItemTypeValue = "DropPoint";
    static constexpr const char* settingsGroup =            "DropPoint";
    static constexpr const char* altitudeName =             "Altitude";
    static constexpr const char* relayNumberName =          "RelayNumber";
    static constexpr const char* positionName =             "Position";

    /// Waypoint + cycle relay + waypoint with hold. lastSequenceNumber must agree with appendMissionItems.
    static constexpr int    itemCount               = 3;
    static constexpr double entryHoldSeconds        = 1;    ///< Hold of the waypoint which precedes the relay
    static constexpr double exitHoldExtraSeconds    = 0.5;  ///< Exit hold is the relay's cycle time plus this

private slots:
    void _setDirty(void);

private:
    QGeoCoordinate  _coordinate;
    int             _sequenceNumber =   0;
    bool            _dirty =            false;

    // Must be declared before the facts: they take their metadata from it as they are constructed
    QMap<QString, FactMetaData*> _metaDataMap;

    SettingsFact _altitudeFact;
    SettingsFact _relayNumberFact;
    SettingsFact _positionFact;

    static constexpr const char* _jsonCoordinateKey =  "coordinate";
    static constexpr const char* _jsonAltitudeKey =    "altitude";
    static constexpr const char* _jsonRelayNumberKey = "relayNumber";
    static constexpr const char* _jsonPositionKey =    "position";
};
