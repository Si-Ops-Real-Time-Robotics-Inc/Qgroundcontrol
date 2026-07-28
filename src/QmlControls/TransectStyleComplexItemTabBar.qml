import QtQuick

import QGroundControl
import QGroundControl.ScreenTools
import QGroundControl.Controls

QGCTabBar {
    id: tabBar

    property bool showCameraTab: true

    /// Name of the selected tab. Bind against this rather than currentIndex, since the indices shift
    /// when a tab is hidden.
    readonly property string currentTabName: currentIndex >= 0 && currentIndex < _tabs.length ? _tabs[currentIndex].name : ""

    readonly property var _tabs: {
        var tabs = [ { name: "grid", icon: "/qmlimages/PatternGrid.png" } ]
        if (showCameraTab) {
            tabs.push({ name: "camera", icon: "/qmlimages/PatternCamera.png" })
        }
        tabs.push({ name: "terrain", icon: "/qmlimages/PatternTerrain.png" })
        tabs.push({ name: "presets", icon: "/qmlimages/PatternPresets.png" })
        return tabs
    }

    // Keeps the original index 2 selection when all tabs are shown, shifted down when there is no camera tab.
    Component.onCompleted: currentIndex = QGroundControl.settingsManager.planViewSettings.displayPresetsTabFirst.rawValue ? (showCameraTab ? 2 : 1) : 0

    Repeater {
        model: tabBar._tabs

        QGCTabButton {
            icon.source:    modelData.icon
            icon.height:    ScreenTools.defaultFontPixelHeight
        }
    }
}
