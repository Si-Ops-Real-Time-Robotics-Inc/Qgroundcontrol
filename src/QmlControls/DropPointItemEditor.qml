/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QGroundControl
import QGroundControl.ScreenTools
import QGroundControl.Controls
import QGroundControl.FactSystem
import QGroundControl.FactControls
import QGroundControl.Palette

// Editor for the Drop Point item.
// Note: The loader which brings this in has no size of its own, it takes it from here. So the width has to
// come from availableWidth and the height from the content, or the editor draws on top of itself.
Rectangle {
    id:     _root
    width:  availableWidth
    height: editorColumn.height + (_margin * 2)
    color:  qgcPal.windowShadeDark
    radius: _radius

    // The following properties must be available up the hierarchy chain
    //  property real   availableWidth    ///< Width for control
    //  property var    missionItem       ///< Mission Item for editor

    property real _margin: ScreenTools.defaultFontPixelHeight / 2
    property real _radius: ScreenTools.defaultFontPixelWidth / 2

    QGCPalette { id: qgcPal; colorGroupEnabled: true }

    Column {
        id:                 editorColumn
        anchors.margins:    _margin
        anchors.top:        parent.top
        anchors.left:       parent.left
        anchors.right:      parent.right
        spacing:            _margin

        GridLayout {
            anchors.left:   parent.left
            anchors.right:  parent.right
            columnSpacing:  _margin
            rowSpacing:     _margin
            columns:        2

            QGCLabel { text: qsTr("Altitude") }
            FactTextField {
                fact:               missionItem.altitude
                Layout.fillWidth:   true
            }

            QGCLabel { text: qsTr("Relay #") }
            FactTextField {
                fact:               missionItem.relayNumber
                Layout.fillWidth:   true
            }

            QGCLabel { text: qsTr("Position") }
            FactTextField {
                fact:               missionItem.position
                Layout.fillWidth:   true
            }
        }

        QGCLabel {
            anchors.left:   parent.left
            anchors.right:  parent.right
            wrapMode:       Text.WordWrap
            font.pointSize: ScreenTools.smallFontPointSize
            text:           qsTr("Holds %1s, cycles relay %2 for %3 x %4s, then holds %5s.")
                                .arg(1)
                                .arg(missionItem.relayNumber.rawValue)
                                .arg(missionItem.position.rawValue)
                                .arg(missionItem.position.rawValue)
                                .arg(missionItem.position.rawValue + 0.5)
        }
    }
}
