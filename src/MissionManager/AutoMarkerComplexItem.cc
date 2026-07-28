/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "AutoMarkerComplexItem.h"
#include "AppSettings.h"
#include "JsonHelper.h"
#include "MissionItem.h"
#include "QGCApplication.h"
#include "SettingsManager.h"

#include <QtCore/QFile>
#include <QtCore/QJsonObject>
#include <QtCore/QTextStream>
#include <QtCore/qmath.h>

#include <algorithm>

const QString AutoMarkerComplexItem::name(AutoMarkerComplexItem::tr("Auto Marker"));

AutoMarkerComplexItem::AutoMarkerComplexItem(PlanMasterController* masterController, bool flyView, const QString& kmlOrShpFile)
    : SurveyComplexItem     (masterController, flyView, kmlOrShpFile, settingsGroup)
    , _metaDataMap          (FactMetaData::createMapFromJsonFile(QStringLiteral(":/json/AutoMarker.SettingsGroup.json"), this))
    , _markerDistanceFact   (settingsGroup, _metaDataMap[markerDistanceName])
    , _relayNumberFact      (settingsGroup, _metaDataMap[relayNumberName])
{
    _editorQml = "qrc:/qml/QGroundControl/Controls/AutoMarkerItemEditor.qml";

    // Spacing is always entered by the user, never derived from camera specs and overlap. CameraCalc stores
    // hand entered grid values in manual camera mode, with transect spacing in adjustedFootprintSide.
    _cameraCalc.setManualCamera();
    _cameraCalc.distanceToSurface()->setRawValue(SettingsManager::instance()->appSettings()->defaultMissionItemAltitude()->rawValue());

    // No camera whatsoever. A zero trigger distance is what the transect builder already uses to mean
    // "don't trigger the camera" (TransectStyleComplexItem::triggerCamera), so this alone keeps every
    // camera command out of the generated mission and forces the photo count to zero.
    _cameraCalc.adjustedFootprintFrontal()->setRawValue(0);
    hoverAndCapture()->setRawValue(false);
    cameraTriggerInTurnAround()->setRawValue(false);

    // Marker spacing changes which points sit on the transects, so the transects have to be rebuilt.
    connect(&_markerDistanceFact, &Fact::valueChanged, this, &AutoMarkerComplexItem::_setDirty);
    connect(&_markerDistanceFact, &Fact::valueChanged, this, &AutoMarkerComplexItem::_rebuildTransects);

    // The relay number only lands in the generated items, it doesn't move anything, so no rebuild needed
    connect(&_relayNumberFact, &Fact::valueChanged, this, &AutoMarkerComplexItem::_setDirty);

    // The transects carry the marker points, so they are recollected whenever the transects are rebuilt.
    connect(this, &TransectStyleComplexItem::visualTransectPointsChanged, this, &AutoMarkerComplexItem::_recalcMarkerPoints);

    setDirty(false);
}

void AutoMarkerComplexItem::_recalcMarkerPoints(void)
{
    _markerPoints.clear();
    for (const QList<CoordInfo_t>& transect: _transects) {
        for (const CoordInfo_t& coordInfo: transect) {
            if (coordInfo.coordType == CoordTypeInteriorMarker) {
                _markerPoints.append(QVariant::fromValue(coordInfo.coord));
            }
        }
    }
    emit markerPointsChanged();

    // markerAnchorCoordinate reports the first marker, which just changed. Nothing else signals that, so its
    // bindings would never see the markers appear or move.
    emit markerAnchorCoordinateChanged();
}

void AutoMarkerComplexItem::save(QJsonArray& planItems)
{
    QJsonObject saveObject;

    _saveCommon(saveObject);
    saveObject[_jsonMarkerDistanceKey] = _markerDistanceFact.rawValue().toDouble();
    saveObject[_jsonRelayNumberKey]    = _relayNumberFact.rawValue().toInt();
    // The anchor isn't saved: it is just the first marker, which the polygon and spacing already decide
    if (!_importedMarkers.isEmpty()) {
        QVariantList importedMarkersVariant;
        for (const QGeoCoordinate& coord: _importedMarkers) {
            importedMarkersVariant.append(QVariant::fromValue(coord));
        }
        QJsonValue importedMarkersJson;
        JsonHelper::saveGeoCoordinateArray(importedMarkersVariant, false /* writeAltitude */, importedMarkersJson);
        saveObject[_jsonImportedMarkersKey] = importedMarkersJson;
    }
    planItems.append(saveObject);
}

bool AutoMarkerComplexItem::load(const QJsonObject& complexObject, int sequenceNumber, QString& errorString)
{
    if (!SurveyComplexItem::load(complexObject, sequenceNumber, errorString)) {
        return false;
    }

    // Missing keys mean a plan written before these existed, so leave the defaults in place.
    if (complexObject.contains(_jsonMarkerDistanceKey)) {
        _markerDistanceFact.setRawValue(complexObject[_jsonMarkerDistanceKey].toDouble());
    }
    if (complexObject.contains(_jsonRelayNumberKey)) {
        _relayNumberFact.setRawValue(complexObject[_jsonRelayNumberKey].toInt());
    }
    if (complexObject.contains(_jsonImportedMarkersKey)) {
        QVariantList importedMarkersVariant;
        if (!JsonHelper::loadGeoCoordinateArray(complexObject[_jsonImportedMarkersKey], false /* altitudeRequired */, importedMarkersVariant, errorString)) {
            return false;
        }
        _importedMarkers.clear();
        for (const QVariant& coordVariant: importedMarkersVariant) {
            _importedMarkers.append(coordVariant.value<QGeoCoordinate>());
        }
        emit markersImportedChanged();
    }
    _rebuildTransects();

    return true;
}

void AutoMarkerComplexItem::_rebuildTransectsPhase1(void)
{
    if (markersImported()) {
        // The imported file replaces the generated grid outright: the markers are flown in the order they
        // were read, so they become a single transect made up entirely of marker points.
        _transects.clear();
        QList<CoordInfo_t> transect;
        for (const QGeoCoordinate& coord: _importedMarkers) {
            transect.append({ coord, CoordTypeInteriorMarker });
        }
        _transects.append(transect);
        return;
    }

    // Re-anchor from scratch on every rebuild. Carrying the grid over from the previous rebuild would leave
    // the markers sitting on the old grid lines when the pattern moves, instead of travelling with it.
    _markerGridValid = false;

    SurveyComplexItem::_rebuildTransectsPhase1();
}

bool AutoMarkerComplexItem::exportMarkersToCsv(const QString& filename)
{
    QString exportFilename = filename;
    if (!exportFilename.endsWith(QStringLiteral(".csv"), Qt::CaseInsensitive)) {
        exportFilename += QStringLiteral(".csv");
    }

    if (_markerPoints.isEmpty()) {
        qgcApp()->showAppMessage(tr("Marker export failed. There are no markers to export."));
        return false;
    }

    QFile file(exportFilename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qgcApp()->showAppMessage(tr("Marker export failed. Unable to open '%1' for writing.").arg(exportFilename));
        return false;
    }

    bool    terrainMode         = _cameraCalc.distanceMode() == QGroundControlQmlGlobal::AltitudeModeCalcAboveTerrain;
    double  altitudeToSurface   = _cameraCalc.distanceToSurface()->rawValue().toDouble();

    QTextStream stream(&file);
    stream << _csvHeader << "\n";
    for (int i=0; i<_markerPoints.count(); i++) {
        QGeoCoordinate coord = _markerPoints[i].value<QGeoCoordinate>();
        // Following terrain each marker carries its own altitude, otherwise they all fly at the item altitude
        double altitude = terrainMode ? coord.altitude() : altitudeToSurface;
        stream << i + 1 << ","
               << QString::number(coord.latitude(),  'f', 9) << ","
               << QString::number(coord.longitude(), 'f', 9) << ","
               << QString::number(qIsNaN(altitude) ? altitudeToSurface : altitude, 'f', 2) << "\n";
    }
    stream.flush();

    if (file.error() != QFileDevice::NoError) {
        qgcApp()->showAppMessage(tr("Marker export failed. Error writing '%1': %2").arg(exportFilename).arg(file.errorString()));
        return false;
    }

    return true;
}

bool AutoMarkerComplexItem::importMarkersFromCsv(const QString& filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qgcApp()->showAppMessage(tr("Marker import failed. Unable to open '%1' for reading.").arg(filename));
        return false;
    }

    QList<QGeoCoordinate>   importedMarkers;
    QTextStream             stream(&file);
    int                     lineNumber = 0;

    while (!stream.atEnd()) {
        QString line = stream.readLine().trimmed();
        lineNumber++;

        if (line.isEmpty()) {
            continue;
        }

        QStringList columns = line.split(',');
        if (columns.count() < 3) {
            qgcApp()->showAppMessage(tr("Marker import failed. Line %1 has %2 columns, expected at least 3 (%3).").arg(lineNumber).arg(columns.count()).arg(_csvHeader));
            return false;
        }

        bool    latitudeOk  = false;
        bool    longitudeOk = false;
        double  latitude    = columns[1].trimmed().toDouble(&latitudeOk);
        double  longitude   = columns[2].trimmed().toDouble(&longitudeOk);

        if (!latitudeOk || !longitudeOk) {
            // The header line doesn't parse as numbers, which is how it is recognized
            if (importedMarkers.isEmpty() && lineNumber == 1) {
                continue;
            }
            qgcApp()->showAppMessage(tr("Marker import failed. Line %1 has a latitude/longitude which isn't a number: '%2'").arg(lineNumber).arg(line));
            return false;
        }

        double altitude = 0;
        if (columns.count() > 3) {
            altitude = columns[3].trimmed().toDouble();
        }

        QGeoCoordinate coord(latitude, longitude, altitude);
        if (!coord.isValid()) {
            qgcApp()->showAppMessage(tr("Marker import failed. Line %1 is not a valid position: '%2'").arg(lineNumber).arg(line));
            return false;
        }
        importedMarkers.append(coord);
    }

    if (importedMarkers.isEmpty()) {
        qgcApp()->showAppMessage(tr("Marker import failed. '%1' contains no markers.").arg(filename));
        return false;
    }

    _importedMarkers = importedMarkers;
    emit markersImportedChanged();

    _rebuildTransects();
    setDirty(true);

    return true;
}

void AutoMarkerComplexItem::clearImportedMarkers(void)
{
    if (_importedMarkers.isEmpty()) {
        return;
    }

    _importedMarkers.clear();
    emit markersImportedChanged();

    _rebuildTransects();
    setDirty(true);
}

QGeoCoordinate AutoMarkerComplexItem::markerAnchorCoordinate(void) const
{
    // Always read back from the markers as they currently are, never remembered. A remembered anchor goes
    // stale as soon as the polygon is moved or rotated, and is then left stranded away from the pattern.
    return _markerPoints.isEmpty() ? QGeoCoordinate() : _markerPoints.first().value<QGeoCoordinate>();
}

void AutoMarkerComplexItem::setMarkerAnchorCoordinate(const QGeoCoordinate& coordinate)
{
    QGeoCoordinate previousAnchor = markerAnchorCoordinate();
    QGeoCoordinate polygonCenter  = _surveyAreaPolygon.center();

    if (!coordinate.isValid() || !previousAnchor.isValid() || !polygonCenter.isValid() || coordinate == previousAnchor) {
        return;
    }

    // Nothing is stored. The whole pattern moves by however far the anchor moved, which carries the first
    // marker onto the requested position and takes the rest of the markers with it.
    double distance = previousAnchor.distanceTo(coordinate);
    double azimuth  = previousAnchor.azimuthTo(coordinate);
    _surveyAreaPolygon.setCenter(polygonCenter.atDistanceAndAzimuth(distance, azimuth));

    _rebuildTransects();
    setDirty(true);
}

double AutoMarkerComplexItem::_projectOntoMarkerAxis(const QGeoCoordinate& coord) const
{
    double distance = _markerGridOrigin.distanceTo(coord);
    if (qFuzzyIsNull(distance)) {
        return 0;
    }
    // Distance along the axis is the component of the origin->coord vector which lies on it.
    double azimuth = _markerGridOrigin.azimuthTo(coord);
    return distance * qCos(qDegreesToRadians(azimuth - _markerGridAzimuth));
}

void AutoMarkerComplexItem::_appendInteriorTransectPoints(QList<CoordInfo_t>& coordInfoTransect, const QGeoCoordinate& entryCoord, const QGeoCoordinate& exitCoord)
{
    double markerDistance = _markerDistanceFact.rawValue().toDouble();
    if (markerDistance <= 0) {
        return;
    }

    double transectAzimuth = entryCoord.azimuthTo(exitCoord);

    // Anchor the grid once per pass. Every transect of that pass then measures from the same origin along the
    // same axis, which is what lines the markers up across neighbouring transects. A transect which isn't
    // parallel to the current axis is the start of the refly pass, which needs an origin of its own.
    double axisOffsetDegrees = fabs(remainder(transectAzimuth - _markerGridAzimuth, 180.0));
    if (!_markerGridValid || axisOffsetDegrees > _markerGridParallelToleranceDegrees) {
        _markerGridOrigin   = entryCoord;
        _markerGridAzimuth  = transectAzimuth;
        _markerGridValid    = true;
    }

    double entryOnAxis  = _projectOntoMarkerAxis(entryCoord);
    double exitOnAxis   = _projectOntoMarkerAxis(exitCoord);
    double lowOnAxis    = qMin(entryOnAxis, exitOnAxis);
    double highOnAxis   = qMax(entryOnAxis, exitOnAxis);

    // Grid positions which land within this transect
    QList<double> markersOnAxis;
    for (int i = static_cast<int>(ceil(lowOnAxis / markerDistance)); i * markerDistance <= highOnAxis; i++) {
        double markerOnAxis = i * markerDistance;
        // A marker on top of the entry or exit point would just duplicate it
        if (qAbs(markerOnAxis - lowOnAxis) < 0.01 || qAbs(markerOnAxis - highOnAxis) < 0.01) {
            continue;
        }
        markersOnAxis.append(markerOnAxis);
    }

    // The transects of a pass alternate direction, so put the markers in the order they are flown
    if (exitOnAxis < entryOnAxis) {
        std::reverse(markersOnAxis.begin(), markersOnAxis.end());
    }

    // The grid axis and this transect are parallel but may point opposite ways. Walk along the transect's own
    // azimuth so the marker lands exactly on it either way.
    bool transectRunsAlongAxis = qCos(qDegreesToRadians(transectAzimuth - _markerGridAzimuth)) >= 0;
    for (int i=0; i<markersOnAxis.count(); i++) {
        double distanceOnAxis       = markersOnAxis[i] - entryOnAxis;
        double distanceAlongTransect = transectRunsAlongAxis ? distanceOnAxis : -distanceOnAxis;
        QGeoCoordinate markerCoord  = entryCoord.atDistanceAndAzimuth(distanceAlongTransect, transectAzimuth);
        CoordInfo_t coordInfo       = { markerCoord, CoordTypeInteriorMarker };
        coordInfoTransect.insert(1 + i, coordInfo);
    }
}

void AutoMarkerComplexItem::_buildAndAppendMissionItems(QList<MissionItem*>& items, QObject* missionItemParent)
{
    // The markers are numbered as they are built, so the numbering has to start over every time. Otherwise a
    // second build would carry on from where the first left off and hand out different actions.
    _markersBuilt = 0;

    TransectStyleComplexItem::_buildAndAppendMissionItems(items, missionItemParent);
}

void AutoMarkerComplexItem::_appendItemsForInteriorMarker(QList<MissionItem*>& items, QObject* missionItemParent, int& seqNum, MAV_FRAME mavFrame, const QGeoCoordinate& coord)
{
    // Keep the item count here in sync with _itemCountForInteriorMarker.

    // Marker number within the mission, counting 1 up to markerPatternCount and then starting over
    int     markerNumber    = (_markersBuilt++ % markerPatternCount) + 1;
    int     relayCycles     = markerNumber;
    double  relayCycleSecs  = markerNumber;
    double  exitHoldSecs    = relayCycleSecs + markerExitHoldExtraSeconds;

    // Settle over the marker point
    _appendWaypoint(items, missionItemParent, seqNum, mavFrame, markerEntryHoldSeconds /* holdTime */, coord);

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

    // Hold at the point while the relay does its work
    _appendWaypoint(items, missionItemParent, seqNum, mavFrame, exitHoldSecs /* holdTime */, coord);
}
