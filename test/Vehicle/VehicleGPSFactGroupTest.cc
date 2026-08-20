/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "VehicleGPSFactGroupTest.h"
#include "VehicleGPSFactGroup.h"
#include "MAVLinkLib.h"

#include <QtTest/QTest>

namespace {

/// Builds a GPS_RAW_INT with the accuracy extension fields set.
mavlink_message_t buildGpsRawInt(quint32 hAccMm, quint32 vAccMm)
{
    mavlink_message_t message{};
    (void) mavlink_msg_gps_raw_int_pack(
        1,                      // system id
        1,                      // component id
        &message,
        0,                      // time_usec
        3,                      // fix_type: 3D
        108322363,              // lat, 1e7 deg
        1068223535,             // lon
        2620,                   // alt, mm
        120,                    // eph
        180,                    // epv
        0,                      // vel
        0,                      // cog
        14,                     // satellites_visible
        0,                      // alt_ellipsoid
        hAccMm,
        vAccMm,
        0,                      // vel_acc
        0,                      // hdg_acc
        0                       // yaw
    );
    return message;
}

} // namespace

void VehicleGPSFactGroupTest::_testAccuracyMillimetresToMetres()
{
    VehicleGPSFactGroup factGroup;

    // 1234 mm = 1.234 m horizontally, 2500 mm = 2.5 m vertically.
    mavlink_message_t message = buildGpsRawInt(1234, 2500);
    factGroup.handleMessage(nullptr, message);

    QVERIFY(!qIsNaN(factGroup.horizontalAccuracy()->rawValue().toDouble()));
    QVERIFY(!qIsNaN(factGroup.verticalAccuracy()->rawValue().toDouble()));
    QCOMPARE(factGroup.horizontalAccuracy()->rawValue().toDouble(), 1.234);
    QCOMPARE(factGroup.verticalAccuracy()->rawValue().toDouble(), 2.5);

    // The metadata must declare metres, or the value would never be converted for a user working
    // in feet, and the unit suffix shown next to it would be wrong.
    QCOMPARE(factGroup.horizontalAccuracy()->rawUnits(), QStringLiteral("m"));
    QCOMPARE(factGroup.verticalAccuracy()->rawUnits(), QStringLiteral("m"));

    // Sanity: the rest of the message still decodes as before.
    QCOMPARE(factGroup.count()->rawValue().toInt(), 14);
    QCOMPARE(factGroup.lock()->rawValue().toInt(), 3);
}

void VehicleGPSFactGroupTest::_testAccuracyUnknownValues()
{
    VehicleGPSFactGroup factGroup;

    // UINT32_MAX is the documented invalid marker.
    mavlink_message_t message = buildGpsRawInt(UINT32_MAX, UINT32_MAX);
    factGroup.handleMessage(nullptr, message);
    QVERIFY2(qIsNaN(factGroup.horizontalAccuracy()->rawValue().toDouble()),
             "UINT32_MAX h_acc should read as unknown, not 4294967 m");
    QVERIFY(qIsNaN(factGroup.verticalAccuracy()->rawValue().toDouble()));

    // Zero is what an autopilot that ignores the extension actually sends, and no receiver has
    // zero metre accuracy - it must not be shown as a perfect fix.
    message = buildGpsRawInt(0, 0);
    factGroup.handleMessage(nullptr, message);
    QVERIFY2(qIsNaN(factGroup.horizontalAccuracy()->rawValue().toDouble()),
             "a zero h_acc should read as unknown, not 0.00 m");
    QVERIFY(qIsNaN(factGroup.verticalAccuracy()->rawValue().toDouble()));

    // A real reading after the unknown ones must still come through.
    message = buildGpsRawInt(850, 1600);
    factGroup.handleMessage(nullptr, message);
    QCOMPARE(factGroup.horizontalAccuracy()->rawValue().toDouble(), 0.85);
    QCOMPARE(factGroup.verticalAccuracy()->rawValue().toDouble(), 1.6);
}
