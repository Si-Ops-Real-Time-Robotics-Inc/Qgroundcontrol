/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

import QtQuick
import QtLocation
import QtPositioning

import QGroundControl

/// Displays one georeferenced GeoTIFF overlay layer (orthomosaic or colorized DEM) pinned to its
/// geographic bounding box. `overlayLayer` is a RasterOverlayLayer; the image is decoded by Qt from
/// overlayLayer.imageUrl. This item only sizes/positions it so it pans and zooms with the map.
MapQuickItem {
    id: root

    property var map           ///< the parent Map, required for fromCoordinate()
    property var overlayLayer   ///< RasterOverlayLayer (note: "layer" is reserved by QQuickItem)

    coordinate:     overlayLayer ? overlayLayer.nwCorner : QtPositioning.coordinate()
    anchorPoint:    Qt.point(0, 0)
    visible:        !!overlayLayer && overlayLayer.layerVisible
    z:              QGroundControl.zOrderRasterOverlay

    sourceItem: Image {
        id:             rasterImage
        source:         overlayLayer ? overlayLayer.imageUrl : ""
        opacity:        overlayLayer ? overlayLayer.opacity : 1
        asynchronous:   true
        cache:          false
        fillMode:       Image.Stretch
    }

    function _updateSize() {
        if (!map || !overlayLayer) {
            return
        }
        var nw = map.fromCoordinate(overlayLayer.nwCorner, false)
        var se = map.fromCoordinate(overlayLayer.seCorner, false)
        rasterImage.width  = Math.abs(se.x - nw.x)
        rasterImage.height = Math.abs(se.y - nw.y)
    }

    onOverlayLayerChanged: _updateSize()
    onMapChanged:          _updateSize()

    Connections {
        target:                 root.map
        ignoreUnknownSignals:   true
        function onZoomLevelChanged()   { root._updateSize() }
        function onCenterChanged()      { root._updateSize() }
        function onWidthChanged()       { root._updateSize() }
        function onHeightChanged()      { root._updateSize() }
    }

    Component.onCompleted: _updateSize()
}
