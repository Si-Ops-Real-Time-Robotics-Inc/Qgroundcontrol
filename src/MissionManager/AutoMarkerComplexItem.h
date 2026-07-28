/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include "SurveyComplexItem.h"
#include "SettingsFact.h"

class PlanMasterController;

/// Survey style pattern which drops a marker at regular intervals along each transect. It has no camera at
/// all: the user enters the transect spacing directly rather than having it calculated from camera specs and
/// image overlap, and no camera trigger commands are generated.
///
/// Each marker is three mission items: fly to the point, cycle the relay, then hold at the point while the
/// relay does its work.
class AutoMarkerComplexItem : public SurveyComplexItem
{
    Q_OBJECT

public:
    /// @param flyView true: Created for use in the Fly View, false: Created for use in the Plan View
    /// @param kmlOrShpFile Polygon comes from this file, empty for default polygon
    AutoMarkerComplexItem(PlanMasterController* masterController, bool flyView, const QString& kmlOrShpFile);

    Q_PROPERTY(Fact*        markerDistance  READ markerDistance CONSTANT)
    Q_PROPERTY(Fact*        relayNumber     READ relayNumber    CONSTANT)
    Q_PROPERTY(QVariantList markerPoints    READ markerPoints   NOTIFY markerPointsChanged)  ///< Marker coordinates, for map display
    /// Position of the first marker. Reading it reports where that marker currently is; writing it moves the
    /// whole pattern so the first marker lands on the value written. Nothing is stored, so it can't go stale
    /// when the polygon is moved or rotated.
    Q_PROPERTY(QGeoCoordinate markerAnchorCoordinate READ markerAnchorCoordinate WRITE setMarkerAnchorCoordinate NOTIFY markerAnchorCoordinateChanged)

    /// true: markers come from an imported file rather than being generated from the polygon and spacing
    Q_PROPERTY(bool markersImported READ markersImported NOTIFY markersImportedChanged)

    Fact*           markerDistance          (void) { return &_markerDistanceFact; }
    Fact*           relayNumber             (void) { return &_relayNumberFact; }
    QVariantList    markerPoints            (void) const { return _markerPoints; }
    QGeoCoordinate  markerAnchorCoordinate  (void) const;
    void            setMarkerAnchorCoordinate(const QGeoCoordinate& coordinate);
    bool            markersImported         (void) const { return !_importedMarkers.isEmpty(); }

    /// Writes the current marker positions to @a filename as csv. Shows a message and returns false on failure.
    Q_INVOKABLE bool exportMarkersToCsv  (const QString& filename);
    /// Replaces the markers with the ones in @a filename. Shows a message and returns false on failure, in
    /// which case the current markers are left alone.
    Q_INVOKABLE bool importMarkersFromCsv(const QString& filename);
    /// Drops the imported markers and goes back to generating them from the polygon and spacing.
    Q_INVOKABLE void clearImportedMarkers(void);

    // Overrides from ComplexMissionItem
    QString patternName         (void) const final { return name; }
    QString presetsSettingsGroup(void) final { return settingsGroup; }
    QString mapVisualQML        (void) const final { return QStringLiteral("AutoMarkerMapVisual.qml"); }
    bool    load                (const QJsonObject& complexObject, int sequenceNumber, QString& errorString) final;

    // Overrides from TransectStyleComplexItem
    void    save                (QJsonArray& planItems) final;

    // Overrides from VisualMissionItem
    QString commandDescription  (void) const final { return tr("Auto Marker"); }
    QString commandName         (void) const final { return tr("Auto Marker"); }
    QString abbreviation        (void) const final { return tr("AM"); }

    static const QString name;

    static constexpr const char* jsonComplexItemTypeValue = "AutoMarker";
    /// Own group so forced values never leak into Survey's saved settings, and so presets stay separate
    /// (a Survey preset would fail to load here since its saved complex item type wouldn't match).
    static constexpr const char* settingsGroup =            "AutoMarker";
    static constexpr const char* markerDistanceName =       "MarkerDistance";
    static constexpr const char* relayNumberName =          "RelayNumber";

    // Each marker acts differently to the one before it, so that they can be told apart on the ground. The
    // Nth marker of the mission cycles the relay N times over N seconds and then holds for N + 0.5 seconds.
    // N counts 1 up to markerPatternCount and then starts over.
    static constexpr int    markerPatternCount          = 6;    ///< Marker actions repeat after this many markers
    static constexpr double markerEntryHoldSeconds      = 1;    ///< Hold of the waypoint which precedes the relay
    static constexpr double markerExitHoldExtraSeconds  = 0.5;  ///< Exit hold is the relay's cycle time plus this

signals:
    void markerPointsChanged(void);
    void markerAnchorCoordinateChanged(void);
    void markersImportedChanged(void);

protected slots:
    // Overrides from SurveyComplexItem
    void _rebuildTransectsPhase1(void) final;

private slots:
    /// Collects the marker coordinates out of the rebuilt transects so the map visual can draw them.
    void _recalcMarkerPoints(void);

private:
    /// Distance of @a coord from the marker grid origin, measured along the marker grid axis. Signed:
    /// negative means @a coord sits behind the origin along that axis.
    double _projectOntoMarkerAxis(const QGeoCoordinate& coord) const;

protected:
    // Overrides from SurveyComplexItem
    QString complexItemTypeValue        (void) const final { return jsonComplexItemTypeValue; }
    void    _appendInteriorTransectPoints(QList<CoordInfo_t>& coordInfoTransect, const QGeoCoordinate& entryCoord, const QGeoCoordinate& exitCoord) final;

    // Overrides from TransectStyleComplexItem
    void    _appendItemsForInteriorMarker(QList<MissionItem*>& items, QObject* missionItemParent, int& seqNum, MAV_FRAME mavFrame, const QGeoCoordinate& coord) final;
    int     _itemCountForInteriorMarker (void) const final { return 3; } // Waypoint + cycle relay + waypoint with hold
    /// Restarts the marker numbering, which the markers are then handed out from as they are built.
    void    _buildAndAppendMissionItems (QList<MissionItem*>& items, QObject* missionItemParent) final;

private:
    QMap<QString, FactMetaData*> _metaDataMap;

    SettingsFact _markerDistanceFact;
    SettingsFact _relayNumberFact;
    QVariantList _markerPoints;

    // Origin and axis of the marker grid. Marker positions are multiples of the marker distance measured
    // along this axis, rather than measured from each transect's own entry point. Transects are flown
    // alternately in opposite directions and can start at different offsets, so measuring per transect
    // leaves the markers of neighbouring transects out of line with each other.
    QGeoCoordinate  _markerGridOrigin;
    double          _markerGridAzimuth  = 0;
    bool            _markerGridValid    = false;

    /// Markers read from a file. When this isn't empty it replaces the generated grid entirely, and the
    /// vehicle flies these in the order they were read.
    QList<QGeoCoordinate> _importedMarkers;

    /// How many markers have been built so far in the current pass over the flight path. Decides which of the
    /// repeating marker actions the next one gets, so it is reset each time the items are rebuilt.
    int _markersBuilt = 0;

    /// Transects within one pass share an azimuth. A transect which isn't parallel to the current grid axis
    /// starts a new pass (the refly at 90 degrees), and becomes the origin for it.
    static constexpr double _markerGridParallelToleranceDegrees = 0.5;

    static constexpr const char* _jsonMarkerDistanceKey  = "markerDistance";
    static constexpr const char* _jsonRelayNumberKey     = "relayNumber";
    static constexpr const char* _jsonImportedMarkersKey = "importedMarkers";

    static constexpr const char* _csvHeader = "index,latitude,longitude,altitude";
};
