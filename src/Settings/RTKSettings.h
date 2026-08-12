/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include "SettingsGroup.h"

class RTKSettings : public SettingsGroup
{
    Q_OBJECT
public:
    RTKSettings(QObject* parent = nullptr);
    DEFINE_SETTING_NAME_GROUP()
    DEFINE_SETTINGFACT(surveyInAccuracyLimit)
    DEFINE_SETTINGFACT(surveyInMinObservationDuration)
    DEFINE_SETTINGFACT(useFixedBasePosition)
    DEFINE_SETTINGFACT(fixedBasePositionLatitude)
    DEFINE_SETTINGFACT(fixedBasePositionLongitude)
    DEFINE_SETTINGFACT(fixedBasePositionAltitude)
    DEFINE_SETTINGFACT(fixedBasePositionAccuracy)

    // Network RTCM source (NTRIP / TCP / UDP / Bluetooth) - feeds RTCM corrections to the vehicle
    DEFINE_SETTINGFACT(rtcmSourceType)      // 0=None, 1=Serial, 2=NTRIP, 3=TCP, 4=UDP, 5=Bluetooth
    DEFINE_SETTINGFACT(ntripHost)
    DEFINE_SETTINGFACT(ntripPort)
    DEFINE_SETTINGFACT(ntripMountpoint)
    DEFINE_SETTINGFACT(ntripUsername)
    DEFINE_SETTINGFACT(ntripPassword)
    DEFINE_SETTINGFACT(ntripSendGGA)
    DEFINE_SETTINGFACT(tcpHost)
    DEFINE_SETTINGFACT(tcpPort)
    DEFINE_SETTINGFACT(udpPort)
    DEFINE_SETTINGFACT(bluetoothDeviceName)
    DEFINE_SETTINGFACT(bluetoothDeviceAddress)
    DEFINE_SETTINGFACT(logRtcmToFile)
    DEFINE_SETTINGFACT(rtcmLogPath)
};
