import QtQuick
import QtQuick.Controls

import QGroundControl
import QGroundControl.ScreenTools
import QGroundControl.Controls

// Statistics section for TransectStyleComplexItems
Grid {
    // The following properties must be available up the hierarchy chain
    //property var    missionItem       ///< Mission Item for editor

    property bool showPhotoStats:  true     ///< false: pattern has no camera, so there is nothing to report about photos
    property bool showMarkerCount: false    ///< true: pattern drops markers, only Auto Marker has them

    columns:        2
    columnSpacing:  ScreenTools.defaultFontPixelWidth

    QGCLabel { text: qsTr("Survey Area") }
    QGCLabel { text: QGroundControl.unitsConversion.squareMetersToAppSettingsAreaUnits(missionItem.coveredArea).toFixed(2) + " " + QGroundControl.unitsConversion.appSettingsAreaUnitsString }

    QGCLabel { text: qsTr("Marker Count"); visible: showMarkerCount }
    // Guarded: markerPoints only exists on the patterns which set showMarkerCount
    QGCLabel { text: showMarkerCount ? missionItem.markerPoints.length : ""; visible: showMarkerCount }

    QGCLabel { text: qsTr("Photo Count"); visible: showPhotoStats }
    QGCLabel { text: missionItem.cameraShots; visible: showPhotoStats }

    QGCLabel { text: qsTr("Photo Interval"); visible: showPhotoStats }
    QGCLabel { text: missionItem.timeBetweenShots.toFixed(1) + " " + qsTr("secs"); visible: showPhotoStats }

    QGCLabel { text: qsTr("Trigger Distance"); visible: showPhotoStats }
    QGCLabel { text: missionItem.cameraCalc.adjustedFootprintFrontal.valueString + " " + missionItem.cameraCalc.adjustedFootprintFrontal.units; visible: showPhotoStats }
}
