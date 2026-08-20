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

/// Covers the RTCM validation gate and the RTCM -> RINEX observation pipeline:
/// RtcmStreamParser -> RtcmObsDecoder -> RinexObsWriter.
class RtcmRinexTest : public UnitTest
{
    Q_OBJECT

private slots:
    /// Only complete, CRC-valid frames leave the parser; garbage is counted and dropped.
    void _testParserRejectsNonRtcm();
    /// A frame split across arbitrary chunk boundaries still comes out intact.
    void _testParserReassemblesSplitFrames();
    /// MSM5 observables decode back to the values they were encoded from.
    void _testMsm5RoundTrip();
    /// GLONASS carrier phase needs the FDMA channel from the extended satellite info.
    void _testGlonassFrequencyChannel();
    /// Constellations sent as separate messages land in a single epoch.
    void _testEpochAssembly();
    /// The written RINEX file has the header records and epoch layout a reader expects.
    void _testRinexOutput();
    /// GPS ephemeris round-trips through 1019 with the right scaling.
    void _testGpsEphemerisRoundTrip();
    /// GLONASS ephemeris uses sign-magnitude fields, so negative coordinates must survive.
    void _testGlonassEphemerisSignMagnitude();
    /// A rebroadcast ephemeris must not produce a second RINEX record.
    void _testNavOutputAndDeduplication();
    /// The base position must reach the header even when 1005/1006 arrives after the first epochs.
    void _testBasePositionReachesHeader();
    /// Logging both formats writes .obs, .nav and .rtcm3 from one stream, sharing a base name.
    void _testBothLogFormats();
};
