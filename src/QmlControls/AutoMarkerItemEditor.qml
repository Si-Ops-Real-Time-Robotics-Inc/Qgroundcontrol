import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs
import QtQuick.Layouts

import QGroundControl
import QGroundControl.ScreenTools
import QGroundControl.Vehicle
import QGroundControl.Controls
import QGroundControl.FactSystem
import QGroundControl.FactControls
import QGroundControl.Palette
import QGroundControl.FlightMap

TransectStyleComplexItemEditor {
    // Imported markers don't need a polygon, they are the pattern
    transectAreaDefinitionComplete: missionItem.surveyAreaPolygon.isValid || missionItem.markersImported
    transectAreaDefinitionHelp:     qsTr("Use the Polygon Tools to create the polygon which outlines your Auto Marker area, or import markers from a csv file.")
    transectValuesHeaderName:       qsTr("Transects")
    transectValuesComponent:        _transectValuesComponent
    presetsTransectValuesComponent: _transectValuesComponent
    cameraSupported:                false
    markerCountSupported:           true

    // The following properties must be available up the hierarchy chain
    //  property real   availableWidth    ///< Width for control
    //  property var    missionItem       ///< Mission Item for editor

    property real   _margin:        ScreenTools.defaultFontPixelWidth / 2
    property var    _missionItem:   missionItem

    Component {
        id: _transectValuesComponent

        GridLayout {
            Layout.fillWidth:   true
            columnSpacing:      _margin
            rowSpacing:         _margin
            columns:            2

            property bool _turnaroundYawSettingsVisible:    missionItem.yawAtTurnaroundAllowed && !forPresets
            // The rate and hold only do anything while the vehicle is actually being told to rotate
            property bool _turnaroundYawActive:             missionItem.yawAtTurnaround.rawValue && missionItem.turnAroundDistance.rawValue > 0

            // Imported markers replace the generated grid, so the values which generate it no longer apply
            property bool _gridGenerated: !missionItem.markersImported

            QGCLabel {
                Layout.columnSpan:  2
                Layout.fillWidth:   true
                wrapMode:           Text.WordWrap
                color:              qgcPal.warningText
                text:               qsTr("Markers are imported from a file. Spacing, Marker Dist and Angle do not apply.")
                visible:            missionItem.markersImported && !forPresets
            }

            QGCLabel {
                text:       qsTr("Marker Dist")
                enabled:    _gridGenerated
            }
            FactTextField {
                fact:               missionItem.markerDistance
                Layout.fillWidth:   true
                enabled:            _gridGenerated
            }

            // Applies to imported markers too, so no _gridGenerated here
            QGCLabel { text: qsTr("Relay #") }
            FactTextField {
                fact:               missionItem.relayNumber
                Layout.fillWidth:   true
            }

            // Full width: these don't fit the narrow right hand column of the grid
            QGCButton {
                text:               qsTr("Set 1st Marker Position")
                Layout.columnSpan:  2
                Layout.fillWidth:   true
                visible:            !forPresets
                enabled:            _gridGenerated && missionItem.markerAnchorCoordinate.isValid
                onClicked:          markerAnchorPositionDialog.createObject(mainWindow).open()

                Component {
                    id: markerAnchorPositionDialog

                    EditPositionDialog {
                        title:                      qsTr("Edit 1st Marker Position")
                        coordinate:                 missionItem.markerAnchorCoordinate
                        showSetPositionFromVehicle: false
                        onCoordinateChanged:        missionItem.markerAnchorCoordinate = coordinate
                    }
                }
            }

            QGCLabel {
                text:       qsTr("Angle")
                enabled:    _gridGenerated
            }
            FactTextField {
                fact:                   missionItem.gridAngle
                Layout.fillWidth:       true
                enabled:                _gridGenerated
                onUpdated:              angleSlider.value = missionItem.gridAngle.value
            }

            QGCSlider {
                id:                     angleSlider
                enabled:                _gridGenerated
                from:                   0
                to:                     359
                stepSize:               1
                tickmarksEnabled:       false
                Layout.fillWidth:       true
                Layout.columnSpan:      2
                Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 1.5
                onValueChanged:         missionItem.gridAngle.value = value
                Component.onCompleted:  value = missionItem.gridAngle.value
                live:                   true
            }

            QGCLabel {
                text:       qsTr("Turnaround dist")
                visible:    !forPresets
            }
            FactTextField {
                Layout.fillWidth:   true
                fact:               missionItem.turnAroundDistance
                visible:            !forPresets
            }

            QGCLabel {
                text:       qsTr("Yaw rate")
                enabled:    _turnaroundYawActive
                visible:    _turnaroundYawSettingsVisible
            }
            FactTextField {
                Layout.fillWidth:   true
                fact:               missionItem.turnaroundYawRate
                enabled:            _turnaroundYawActive
                visible:            _turnaroundYawSettingsVisible
            }

            QGCLabel {
                text:       qsTr("Yaw hold")
                enabled:    _turnaroundYawActive
                visible:    _turnaroundYawSettingsVisible
            }
            FactTextField {
                Layout.fillWidth:   true
                fact:               missionItem.turnaroundYawHold
                enabled:            _turnaroundYawActive
                visible:            _turnaroundYawSettingsVisible
            }

            QGCLabel {
                text:               qsTr("Markers")
                Layout.columnSpan:  2
                visible:            !forPresets
            }
            RowLayout {
                Layout.columnSpan:  2
                Layout.fillWidth:   true
                spacing:            _margin
                visible:            !forPresets

                QGCButton {
                    Layout.fillWidth:   true
                    text:               qsTr("Export CSV")
                    enabled:            missionItem.markerPoints.length > 0
                    onClicked:          markerCsvSaveDialog.openForSave()
                }

                QGCButton {
                    Layout.fillWidth:   true
                    text:               qsTr("Import CSV")
                    onClicked:          markerCsvLoadDialog.openForLoad()
                }
            }

            QGCButton {
                Layout.columnSpan:  2
                Layout.fillWidth:   true
                text:               qsTr("Back To Generated Markers")
                visible:            missionItem.markersImported && !forPresets
                onClicked:          missionItem.clearImportedMarkers()
            }

            QGCOptionsComboBox {
                Layout.columnSpan:  2
                Layout.fillWidth:   true
                visible:            !forPresets

                // No camera related options here (hover and capture, images in turnarounds) since Auto Marker
                // has no camera at all.
                model: [
                    {
                        text:       qsTr("Refly at 90 deg offset"),
                        fact:       missionItem.refly90Degrees,
                        enabled:    missionItem.cameraCalc.distanceMode !== QGroundControl.AltitudeModeCalcAboveTerrain,
                        visible:    true
                    },
                    {
                        text:       qsTr("Fly alternate transects"),
                        fact:       missionItem.flyAlternateTransects,
                        enabled:    true,
                        visible:    _vehicle ? (_vehicle.fixedWing || _vehicle.vtol) : false
                    },
                    {
                        text:       qsTr("Rotate at turnarounds"),
                        fact:       missionItem.yawAtTurnaround,
                        // Without turnarounds there are no points at which to rotate
                        enabled:    missionItem.turnAroundDistance.rawValue > 0,
                        visible:    missionItem.yawAtTurnaroundAllowed
                    },
                    {
                        text:       qsTr("Rotate on every turn"),
                        fact:       missionItem.yawAtEveryTurn,
                        enabled:    _turnaroundYawActive,
                        visible:    missionItem.yawAtTurnaroundAllowed
                    }
                ]
            }
        }
    }

    // On mobile QGC uses its own file browser which only lists this one folder, with no way to navigate
    // elsewhere, so exports and imports have to share a folder that marker CSVs actually live in.
    QGCFileDialog {
        id:             markerCsvSaveDialog
        folder:         QGroundControl.settingsManager.appSettings.markerSavePath
        title:          qsTr("Export Markers")
        nameFilters:    [ qsTr("CSV files (*.csv)"), qsTr("All Files (*)") ]
        defaultSuffix:  "csv"

        onAcceptedForSave: (file) => {
            missionItem.exportMarkersToCsv(file)
            close()
        }
    }

    QGCFileDialog {
        id:             markerCsvLoadDialog
        folder:         QGroundControl.settingsManager.appSettings.markerSavePath
        title:          qsTr("Import Markers")
        nameFilters:    [ qsTr("CSV files (*.csv)"), qsTr("All Files (*)") ]

        onAcceptedForLoad: (file) => {
            missionItem.importMarkersFromCsv(file)
            close()
        }
    }

    KMLOrSHPFileDialog {
        id:             kmlOrSHPLoadDialog
        title:          qsTr("Select Polygon File")

        onAcceptedForLoad: (file) => {
            missionItem.surveyAreaPolygon.loadKMLOrSHPFile(file)
            missionItem.resetState = false
            close()
        }
    }
}
