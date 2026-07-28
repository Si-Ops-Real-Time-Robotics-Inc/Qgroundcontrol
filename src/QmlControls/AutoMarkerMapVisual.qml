/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
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

/// Auto Marker Complex Mission Item visuals. Survey visuals, a dot at each marker position, and a larger dot
/// on the first marker to show where the pattern is anchored.
TransectStyleMapVisuals {
    id:                 _autoMarkerRoot
    polygonInteractive: true

    // Note: `object` is a context property from the loader, not a property of this item, so it resolves
    // unqualified but not as _autoMarkerRoot.object. _missionItem comes from TransectStyleMapVisuals and is
    // what the components below have to go through.
    property var  _markerPoints: _missionItem.markerPoints
    property bool _showMarkers:  _missionItem.isCurrentItem

    // Note: Do not add a Component.onCompleted here. TransectStyleMapVisuals uses one to create its own
    // visuals, and an assignment here would override it rather than run alongside it. The Instantiator below
    // brings its own signals, and the anchor handle creation hangs off a child item's own handler.
    Instantiator {
        model: _autoMarkerRoot._markerPoints

        delegate: MapQuickItem {
            anchorPoint.x:  sourceItem.width / 2
            anchorPoint.y:  sourceItem.height / 2
            z:              QGroundControl.zOrderMapItems
            coordinate:     modelData
            visible:        _autoMarkerRoot._showMarkers
            opacity:        _autoMarkerRoot.opacity

            sourceItem: Rectangle {
                width:          ScreenTools.defaultFontPixelHeight * 0.4
                height:         width
                radius:         width / 2
                color:          "white"
                border.color:   "black"
                border.width:   1
            }
        }

        onObjectAdded:   (index, markerItem) => map.addMapItem(markerItem)
        onObjectRemoved: (index, markerItem) => map.removeMapItem(markerItem)
    }

    QGCDynamicObjectManager {
        id: anchorObjMgr
    }

    // The anchor marker has to be parented to the map, so it is created dynamically. This child item's own
    // onCompleted does not clash with the base's.
    // Note: The anchor is not draggable. It is set from the editor's Set 1st Marker Position button.
    Item {
        Component.onCompleted:   anchorObjMgr.createObject(anchorIndicatorComponent, map, true /* addMapItem */)
        Component.onDestruction: anchorObjMgr.destroyObjects()
    }

    Component {
        id: anchorIndicatorComponent

        MapQuickItem {
            anchorPoint.x:  sourceItem.width / 2
            anchorPoint.y:  sourceItem.height / 2
            z:              QGroundControl.zOrderMapItems + 3   // Above the plain marker dots
            coordinate:     _autoMarkerRoot._missionItem.markerAnchorCoordinate
            visible:        _autoMarkerRoot._showMarkers && _autoMarkerRoot._missionItem.markerAnchorCoordinate.isValid
            opacity:        _autoMarkerRoot.opacity

            sourceItem: Rectangle {
                width:          ScreenTools.defaultFontPixelHeight * 0.8
                height:         width
                radius:         width / 2
                color:          "white"
                border.color:   "black"
                border.width:   2
            }
        }
    }

}
