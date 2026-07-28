import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs
import QtQuick.Layouts

import QGroundControl
import QGroundControl.ScreenTools
import QGroundControl.Vehicle
import QGroundControl.Controls
import QGroundControl.FactControls
import QGroundControl.Palette
import QGroundControl.FlightMap

TransectStyleComplexItemEditor {
    transectAreaDefinitionComplete: _missionItem.corridorPolyline.isValid
    transectAreaDefinitionHelp:     qsTr("Use the Polyline Tools to create the polyline which defines the corridor.")
    transectValuesHeaderName:       qsTr("Corridor")
    transectValuesComponent:        _transectValuesComponent
    presetsTransectValuesComponent: _transectValuesComponent

    // The following properties must be available up the hierarchy chain
    //  property real   availableWidth    ///< Width for control
    //  property var    missionItem       ///< Mission Item for editor

    property real   _margin:        ScreenTools.defaultFontPixelWidth / 2
    property var    _missionItem:   missionItem

    Component {
        id: _transectValuesComponent

        GridLayout {
            columnSpacing:  _margin
            rowSpacing:     _margin
            columns:        2

            property bool _turnaroundYawSettingsVisible:    _missionItem.yawAtTurnaroundAllowed && !forPresets
            // The rate and hold only do anything while the vehicle is actually being told to rotate
            property bool _turnaroundYawActive:             _missionItem.yawAtTurnaround.rawValue && _missionItem.turnAroundDistance.rawValue > 0

            QGCLabel { text: qsTr("Width") }
            FactTextField {
                fact:               _missionItem.corridorWidth
                Layout.fillWidth:   true
            }

            QGCLabel {
                text:       qsTr("Turnaround dist")
                visible:    !forPresets
            }
            FactTextField {
                fact:               _missionItem.turnAroundDistance
                Layout.fillWidth:   true
                visible:            !forPresets
            }

            FactCheckBox {
                Layout.columnSpan:  2
                text:               qsTr("Images in turnarounds")
                fact:               _missionItem.cameraTriggerInTurnAround
                enabled:            _missionItem.hoverAndCaptureAllowed ? !_missionItem.hoverAndCapture.rawValue : true
                visible:            !forPresets
            }

            FactCheckBox {
                Layout.columnSpan:  2
                text:               qsTr("Rotate at turnarounds")
                fact:               _missionItem.yawAtTurnaround
                // Without turnarounds there are no points at which to rotate
                enabled:            _missionItem.turnAroundDistance.rawValue > 0
                visible:            _turnaroundYawSettingsVisible
            }

            FactCheckBox {
                Layout.columnSpan:  2
                text:               qsTr("Rotate on every turn")
                fact:               _missionItem.yawAtEveryTurn
                enabled:            _turnaroundYawActive
                visible:            _turnaroundYawSettingsVisible
            }

            QGCLabel {
                text:       qsTr("Yaw rate")
                enabled:    _turnaroundYawActive
                visible:    _turnaroundYawSettingsVisible
            }
            FactTextField {
                fact:               _missionItem.turnaroundYawRate
                Layout.fillWidth:   true
                enabled:            _turnaroundYawActive
                visible:            _turnaroundYawSettingsVisible
            }

            QGCLabel {
                text:       qsTr("Yaw hold")
                enabled:    _turnaroundYawActive
                visible:    _turnaroundYawSettingsVisible
            }
            FactTextField {
                fact:               _missionItem.turnaroundYawHold
                Layout.fillWidth:   true
                enabled:            _turnaroundYawActive
                visible:            _turnaroundYawSettingsVisible
            }
        }
    }
}
