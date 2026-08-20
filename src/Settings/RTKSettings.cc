/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "RTKSettings.h"

#include <QtQml/QQmlEngine>

DECLARE_SETTINGGROUP(RTK, "RTK")
{
    qmlRegisterUncreatableType<RTKSettings>("QGroundControl.SettingsManager", 1, 0, "RTKSettings", "Reference only"); \
}

DECLARE_SETTINGSFACT(RTKSettings, surveyInAccuracyLimit)
DECLARE_SETTINGSFACT(RTKSettings, surveyInMinObservationDuration)
DECLARE_SETTINGSFACT(RTKSettings, useFixedBasePosition)
DECLARE_SETTINGSFACT(RTKSettings, fixedBasePositionLatitude)
DECLARE_SETTINGSFACT(RTKSettings, fixedBasePositionLongitude)
DECLARE_SETTINGSFACT(RTKSettings, fixedBasePositionAltitude)
DECLARE_SETTINGSFACT(RTKSettings, fixedBasePositionAccuracy)
DECLARE_SETTINGSFACT(RTKSettings, rtcmSourceType)
DECLARE_SETTINGSFACT(RTKSettings, ntripHost)
DECLARE_SETTINGSFACT(RTKSettings, ntripPort)
DECLARE_SETTINGSFACT(RTKSettings, ntripMountpoint)
DECLARE_SETTINGSFACT(RTKSettings, ntripUsername)
DECLARE_SETTINGSFACT(RTKSettings, ntripPassword)
DECLARE_SETTINGSFACT(RTKSettings, ntripSendGGA)
DECLARE_SETTINGSFACT(RTKSettings, tcpHost)
DECLARE_SETTINGSFACT(RTKSettings, tcpPort)
DECLARE_SETTINGSFACT(RTKSettings, udpPort)
DECLARE_SETTINGSFACT(RTKSettings, bluetoothDeviceName)
DECLARE_SETTINGSFACT(RTKSettings, bluetoothDeviceAddress)
DECLARE_SETTINGSFACT(RTKSettings, rtcmLogFormat)
DECLARE_SETTINGSFACT(RTKSettings, logRtcmToFile)
DECLARE_SETTINGSFACT(RTKSettings, rtcmLogPath)
