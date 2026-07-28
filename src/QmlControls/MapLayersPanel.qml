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
import QtPositioning

import QGroundControl
import QGroundControl.ScreenTools
import QGroundControl.Controls
import QGroundControl.Palette
import QGroundControl.GeoTiffHelper

/// Manager for user-imported GeoTIFF overlay layers (orthomosaics + colorized DEMs): load files,
/// per-layer show/hide, opacity and remove, plus the DEM elevation legend and terrain-profile
/// selection. The layer list lives in the GeoTiffHelper singleton, so every instance of this panel
/// (Plan view, Fly view) shows and edits the same layers. The caller positions/anchors this item.
Rectangle {
    id:     root
    width:  layerColumn.width  + (_margins * 2)
    height: layerColumn.height + (_margins * 2)
    radius: ScreenTools.defaultFontPixelWidth * 0.5
    color:  panelPal.window
    opacity: 0.9
    z:      QGroundControl.zOrderWidgets

    property var map    ///< optional: the Map to zoom to a layer's bounds when one is loaded

    property real   _margins:       ScreenTools.defaultFontPixelWidth * 0.75
    property var    _appSettings:   QGroundControl.settingsManager.appSettings
    property string _errorString:   ""

    QGCPalette { id: panelPal; colorGroupEnabled: true }

    QGCFileDialog {
        id:          geoTiffFileDialog
        // On mobile QGC uses its own file browser which only lists this one folder, with no way to
        // navigate elsewhere, so this has to be the folder GeoTIFFs actually live in.
        folder:      _appSettings ? _appSettings.geoTiffSavePath : ""
        title:       qsTr("Select GeoTIFF")
        nameFilters: GeoTiffHelper.fileDialogGeoTiffFilters

        onAcceptedForLoad: (file) => {
            GeoTiffHelper.loadFile(file)
            close()
        }
    }

    Connections {
        target: GeoTiffHelper
        function onLayerAdded(layer) {
            root._errorString = ""
            if (root.map) {
                root.map.setVisibleRegion(QtPositioning.rectangle(layer.nwCorner, layer.seCorner))
            }
        }
        function onLoadFailed(errorString) {
            root._errorString = errorString
        }
    }

    Column {
        id:               layerColumn
        anchors.centerIn: parent
        spacing:          _margins

        RowLayout {
            width:   Math.max(implicitWidth, ScreenTools.defaultFontPixelWidth * 30)
            spacing: _margins
            QGCLabel {
                text:             qsTr("Map Layers")
                font.bold:        true
                Layout.fillWidth: true
            }
            QGCButton {
                text:      qsTr("Load…")
                onClicked: geoTiffFileDialog.openForLoad()
            }
        }

        QGCLabel {
            visible: GeoTiffHelper.layers.count === 0 && !GeoTiffHelper.loading
            text:    qsTr("No layers loaded")
            color:   panelPal.text
            opacity: 0.6
        }

        // Decoding runs on a worker thread, so this keeps animating rather than freezing with the UI
        RowLayout {
            spacing: _margins
            visible: GeoTiffHelper.loading

            BusyIndicator {
                running:                true
                // Big enough that the Basic style's 6px padding leaves something to draw, and the dots
                // are drawn in palette.dark which would otherwise be invisible against this dark panel
                Layout.preferredWidth:  ScreenTools.defaultFontPixelHeight * 2
                Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 2
                palette.dark:           panelPal.text
            }
            QGCLabel {
                // Progress is only meaningful for a single decode, so several at once just report the count
                text:    GeoTiffHelper.loadingProgress < 0 ?
                             qsTr("Loading %1 files…").arg(GeoTiffHelper.loadingCount) :
                             qsTr("Loading… %1%").arg(GeoTiffHelper.loadingProgress)
                color:   panelPal.text
                opacity: 0.6
            }
        }

        Repeater {
            model: GeoTiffHelper.layers
            delegate: Column {
                spacing: 2

                RowLayout {
                    spacing: _margins
                    QGCCheckBox {
                        checked:   object.layerVisible
                        onClicked: object.layerVisible = checked
                    }
                    QGCLabel {
                        text:                  object.name
                        elide:                 Text.ElideMiddle
                        Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 15
                    }
                    QGCLabel {
                        text:    object.elevation ? qsTr("DEM") : qsTr("RGB")
                        color:   panelPal.text
                        opacity: 0.6
                    }
                    QGCSlider {
                        width:          ScreenTools.defaultFontPixelWidth * 8
                        from:           0
                        to:             1
                        value:          object.opacity
                        onValueChanged: object.opacity = value
                    }
                    QGCButton {
                        text:      qsTr("✕")
                        onClicked: GeoTiffHelper.removeLayer(index)
                    }
                }

                // Elevation color legend (DEM layers only)
                RowLayout {
                    visible: object.elevation
                    spacing: _margins
                    QGCLabel {
                        text:           Math.round(object.minElevation) + qsTr(" m")
                        font.pointSize: ScreenTools.smallFontPointSize
                    }
                    Rectangle {
                        Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 16
                        height:                ScreenTools.defaultFontPixelHeight * 0.5
                        border.width:          1
                        border.color:          panelPal.text
                        gradient: Gradient {
                            orientation: Gradient.Horizontal
                            GradientStop { position: 0.00; color: Qt.rgba(0,       97/255,  71/255,  1) }
                            GradientStop { position: 0.25; color: Qt.rgba(90/255,  165/255, 75/255,  1) }
                            GradientStop { position: 0.50; color: Qt.rgba(232/255, 215/255, 125/255, 1) }
                            GradientStop { position: 0.75; color: Qt.rgba(161/255, 84/255,  38/255,  1) }
                            GradientStop { position: 1.00; color: Qt.rgba(1,       1,       1,       1) }
                        }
                    }
                    QGCLabel {
                        text:           Math.round(object.maxElevation) + qsTr(" m")
                        font.pointSize: ScreenTools.smallFontPointSize
                    }
                }
            }
        }

        QGCLabel {
            width:    ScreenTools.defaultFontPixelWidth * 28
            wrapMode: Text.WordWrap
            color:    panelPal.warningText
            visible:  root._errorString !== ""
            text:     root._errorString
        }
    }
}
