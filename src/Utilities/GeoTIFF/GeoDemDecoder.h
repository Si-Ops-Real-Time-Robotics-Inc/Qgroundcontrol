/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include <QtCore/QVector>

class QString;
class QImage;

/// Decodes a single-band elevation GeoTIFF (DEM/DSM/DTM) and colorizes it into an RGBA image
/// (terrain ramp, nodata -> transparent) for display as a map overlay. Qt's image plugins cannot
/// read float rasters, so float DEMs are decoded in-house (uncompressed or Deflate); integer DEMs
/// are read through Qt (which handles LZW/Deflate/tiled/BigTIFF).
namespace GeoDemDecoder
{
    /// @return true if @a file is a single-band raster (i.e. an elevation model, not RGB imagery).
    bool isElevationRaster(const QString &file);

    /// Decode and colorize a single-band DEM into an ARGB32 image with transparent nodata.
    ///     @param file             Absolute path to the .tif / .tiff
    ///     @param outImage[out]     Colorized RGBA image
    ///     @param minElevation[out] Elevation mapped to the low end of the color ramp
    ///     @param maxElevation[out] Elevation mapped to the high end of the color ramp
    ///     @param outGrid[out]      Decimated elevation grid (row-major, row 0 = north); nodata cells are NaN
    ///     @param outGridW[out]     Grid width
    ///     @param outGridH[out]     Grid height
    ///     @param errorString[out]
    /// @return true on success
    bool decodeColorized(const QString &file, QImage &outImage, double &minElevation, double &maxElevation,
                         QVector<float> &outGrid, int &outGridW, int &outGridH, QString &errorString,
                         const std::function<void(int)> &progress = {});
}
