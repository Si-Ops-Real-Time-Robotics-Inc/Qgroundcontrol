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
import QtLocation
import QtPositioning

import QGroundControl
import QGroundControl.ScreenTools
import QGroundControl.Palette
import QGroundControl.Controls
import QGroundControl.FlightMap

/// Drop Point Complex Mission Item visuals. A single draggable point, the same as a plain waypoint has.
Item {
    id: _root

    property var    map                     ///< Map control to place item in
    property var    vehicle                 ///< Vehicle associated with this item
    property bool   interactive: true

    property var    _missionItem:       object
    property bool   _dragAreaShowing:   false

    signal clicked(int sequenceNumber)

    function updateDragArea() {
        var wantDragArea = _missionItem.isCurrentItem && map.planView
        if (wantDragArea !== _dragAreaShowing) {
            dragAreaLoader.active = wantDragArea
            _dragAreaShowing = wantDragArea
        }
    }

    Component.onCompleted: {
        indicatorLoader.active = true
        updateDragArea()
    }

    Connections {
        target: _missionItem

        function onIsCurrentItemChanged() { updateDragArea() }
    }

    Loader {
        id:             indicatorLoader
        asynchronous:   true
        active:         false

        sourceComponent: indicatorComponent

        onLoaded: {
            if (item) {
                item.parent = map
                map.addMapItem(item)
            }
        }
    }

    Loader {
        id:             dragAreaLoader
        asynchronous:   true
        active:         false

        sourceComponent: dragAreaComponent

        onLoaded: {
            if (item) {
                item.parent = map
            }
        }
    }

    Component {
        id: indicatorComponent

        MissionItemIndicator {
            coordinate:     _missionItem.coordinate
            visible:        _missionItem.specifiesCoordinate
            z:              QGroundControl.zOrderMapItems
            missionItem:    _missionItem
            sequenceNumber: _missionItem.sequenceNumber
            opacity:        _root.opacity
            onClicked:      if (_root.interactive) _root.clicked(_missionItem.sequenceNumber)
        }
    }

    // Control which is used to drag the drop point around
    Component {
        id: dragAreaComponent

        MissionItemIndicatorDrag {
            mapControl:              _root.map
            itemIndicator:           indicatorLoader.item
            itemCoordinate:          _missionItem.coordinate
            visible:                 _root.interactive
            onItemCoordinateChanged: _missionItem.coordinate = itemCoordinate
        }
    }
}
