/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include "UnitTest.h"

/// Covers the GPS_RAW_INT accuracy fields surfaced on the GPS indicator page.
class VehicleGPSFactGroupTest : public UnitTest
{
    Q_OBJECT

private slots:
    /// h_acc / v_acc arrive in millimetres and must reach the facts as metres.
    void _testAccuracyMillimetresToMetres();
    /// Autopilots that do not populate the extension leave zero or UINT32_MAX; both mean unknown.
    void _testAccuracyUnknownValues();
};
