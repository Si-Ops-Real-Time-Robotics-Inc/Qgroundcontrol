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
import QGroundControl.FactSystem
import QGroundControl.FactControls
import QGroundControl.Controls
import QGroundControl.ScreenTools

SettingsPage {
    property var    rtkSettings:        QGroundControl.settingsManager.rtkSettings
    property var    rtcmStreamManager:  QGroundControl.rtcmStreamManager
    property var    rtcmStatus:         QGroundControl.rtcmStreamManager.rtcmFactGroup
    property int    rtcmSourceType:     rtkSettings.rtcmSourceType.rawValue

    readonly property int rtcmSourceNone:   0
    readonly property int rtcmSourceSerial: 1
    readonly property int rtcmSourceNtrip:  2
    readonly property int rtcmSourceTcp:    3
    readonly property int rtcmSourceUdp:    4

    readonly property bool _isNetworkSource: rtcmSourceType === rtcmSourceNtrip || rtcmSourceType === rtcmSourceTcp || rtcmSourceType === rtcmSourceUdp

    SettingsGroupLayout {
        heading:            qsTr("Network RTCM Source")
        Layout.fillWidth:   true

        LabelledFactComboBox {
            Layout.fillWidth:   true
            label:              qsTr("Correction Source")
            fact:               rtkSettings.rtcmSourceType
            indexModel:         false
        }

        QGCLabel {
            Layout.fillWidth:   true
            wrapMode:           Text.WordWrap
            font.pointSize:     ScreenTools.smallFontPointSize
            text:               qsTr("The correction stream runs independently of the vehicle connection. If the vehicle link drops mid-flight, logging continues and the last known position keeps being sent to the caster.")
            visible:            _isNetworkSource
        }

        // NTRIP
        LabelledFactTextField {
            Layout.fillWidth:   true
            label:              qsTr("Caster Host")
            fact:               rtkSettings.ntripHost
            visible:            rtcmSourceType === rtcmSourceNtrip
        }
        LabelledFactTextField {
            Layout.fillWidth:   true
            label:              qsTr("Caster Port")
            fact:               rtkSettings.ntripPort
            visible:            rtcmSourceType === rtcmSourceNtrip
        }
        LabelledFactTextField {
            Layout.fillWidth:   true
            label:              qsTr("Mountpoint")
            fact:               rtkSettings.ntripMountpoint
            visible:            rtcmSourceType === rtcmSourceNtrip
        }
        LabelledFactTextField {
            Layout.fillWidth:   true
            label:              qsTr("Username")
            fact:               rtkSettings.ntripUsername
            visible:            rtcmSourceType === rtcmSourceNtrip
        }
        LabelledFactTextField {
            Layout.fillWidth:   true
            label:              qsTr("Password")
            fact:               rtkSettings.ntripPassword
            visible:            rtcmSourceType === rtcmSourceNtrip
            textField.echoMode: TextInput.Password
        }
        RowLayout {
            Layout.fillWidth:   true
            spacing:            ScreenTools.defaultFontPixelWidth
            visible:            rtcmSourceType === rtcmSourceNtrip

            QGCButton {
                text:       rtcmStreamManager.fetchingMountpoints ? qsTr("Fetching…") : qsTr("Get Mountpoints")
                enabled:    !rtcmStreamManager.fetchingMountpoints && rtkSettings.ntripHost.rawValue !== ""
                onClicked:  rtcmStreamManager.fetchMountpoints()
            }
            QGCComboBox {
                Layout.fillWidth:   true
                visible:            rtcmStreamManager.mountpoints.length > 0
                model:              rtcmStreamManager.mountpoints
                onActivated:        (index) => { rtkSettings.ntripMountpoint.rawValue = rtcmStreamManager.mountpoints[index] }
            }
        }
        QGCLabel {
            Layout.fillWidth:   true
            wrapMode:           Text.WordWrap
            color:              QGroundControl.globalPalette.warningText
            text:               rtcmStreamManager.mountpointsError
            visible:            rtcmSourceType === rtcmSourceNtrip && rtcmStreamManager.mountpointsError !== ""
        }
        FactCheckBoxSlider {
            Layout.fillWidth:   true
            text:               qsTr("Send GGA position (VRS)")
            fact:               rtkSettings.ntripSendGGA
            visible:            rtcmSourceType === rtcmSourceNtrip
        }

        // TCP
        LabelledFactTextField {
            Layout.fillWidth:   true
            label:              qsTr("TCP Host")
            fact:               rtkSettings.tcpHost
            visible:            rtcmSourceType === rtcmSourceTcp
        }
        LabelledFactTextField {
            Layout.fillWidth:   true
            label:              qsTr("TCP Port")
            fact:               rtkSettings.tcpPort
            visible:            rtcmSourceType === rtcmSourceTcp
        }

        // UDP
        LabelledFactTextField {
            Layout.fillWidth:   true
            label:              qsTr("Listen Port")
            fact:               rtkSettings.udpPort
            visible:            rtcmSourceType === rtcmSourceUdp
        }

        FactCheckBoxSlider {
            Layout.fillWidth:   true
            text:               qsTr("Log RTCM to file")
            fact:               rtkSettings.logRtcmToFile
            visible:            _isNetworkSource
        }

        LabelledButton {
            label:              qsTr("Connection")
            buttonText:         rtcmStreamManager.active ? qsTr("Disconnect") : qsTr("Connect")
            visible:            _isNetworkSource

            onClicked: {
                if (rtcmStreamManager.active) {
                    rtcmStreamManager.stopStream()
                } else {
                    rtcmStreamManager.startStream()
                }
            }
        }
    }

    SettingsGroupLayout {
        heading:            qsTr("Status")
        Layout.fillWidth:   true
        visible:            rtcmStreamManager.active

        LabelledLabel {
            label:      qsTr("Source")
            labelText:  rtcmStatus.sourceType.valueString
        }
        LabelledLabel {
            label:      qsTr("Stream")
            labelText:  rtcmStatus.mountpoint.valueString
            visible:    rtcmStatus.mountpoint.valueString !== ""
        }
        LabelledLabel {
            label:      qsTr("Status")
            labelText:  rtcmStatus.connected.value ? qsTr("Connected") : qsTr("Connecting…")
        }
        LabelledLabel {
            label:      qsTr("Data Rate")
            labelText:  rtcmStatus.bytesPerSecond.valueString + qsTr(" B/s")
        }
        LabelledLabel {
            label:      qsTr("Base Station")
            labelText:  rtcmStatus.baseValid.value
                            ? (rtcmStatus.baseLatitude.valueString + ", " + rtcmStatus.baseLongitude.valueString)
                            : qsTr("Waiting…")
        }
        LabelledLabel {
            label:      qsTr("Base Altitude")
            labelText:  rtcmStatus.baseAltitude.valueString + qsTr(" m")
            visible:    rtcmStatus.baseValid.value
        }
        LabelledLabel {
            label:      qsTr("Log File")
            labelText:  rtcmStatus.logFileName.valueString
            visible:    rtcmStatus.logFileName.valueString !== ""
        }
        LabelledLabel {
            label:      qsTr("Logged")
            labelText:  (rtcmStatus.logBytes.value / 1024).toFixed(1) + qsTr(" KB")
            visible:    rtcmStatus.logFileName.valueString !== ""
        }
    }
}
