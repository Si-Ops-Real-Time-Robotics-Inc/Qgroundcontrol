/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include <QtCore/QLoggingCategory>
#include <QtCore/QString>

#include <functional>

Q_DECLARE_LOGGING_CATEGORY(GeoOrthoDecoderLog)

class QImage;

/// Decodes a multi-band (RGB/RGBA) GeoTIFF orthomosaic to a display image using libtiff.
///
/// Orthomosaics are routinely far too large to decode whole -- a 21464x30570 RGBA ortho is ~2.6 GB
/// once decompressed, which defeats QImageReader (it decodes fully before scaling). This decoder
/// subsamples while reading, so peak memory tracks the *output* size rather than the source.
namespace GeoOrthoDecoder
{
    /// True if this decoder handles @a file's pixel layout (8-bit, >=3 chunky samples).
    /// Callers should fall back to QImageReader when false.
    bool canDecode(const QString &file);

    /// Decode @a file, subsampled so its long edge is at most @a maxDim. Nodata is made transparent:
    /// from the alpha band when present, otherwise by keying near-black pixels.
    /// @a progress, if set, is called with 0..100 as the read proceeds. Reading is the slow part by far,
    /// so it covers the whole range.
    /// Returns false and sets @a errorString on failure.
    bool decode(const QString &file, int maxDim, QImage &outImage, QString &errorString,
                const std::function<void(int)> &progress = {});
}
