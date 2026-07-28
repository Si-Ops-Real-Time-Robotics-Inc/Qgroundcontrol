/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "GeoDemDecoder.h"
#include "QGCLoggingCategory.h"

#include <QtCore/QCoreApplication>
#include <QtGui/QImage>

#include <tiffio.h>

#ifndef TIFFTAG_GDAL_NODATA
#define TIFFTAG_GDAL_NODATA 42113
#endif

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

QGC_LOGGING_CATEGORY(GeoDemDecoderLog, "qgc.utilities.geodemdecoder")

namespace {

const char *kErrPrefix = QT_TRANSLATE_NOOP("GeoTiff", "DEM load failed. %1");

// Cap the colorized elevation grid's long edge (memory bound; decimate on read above this).
constexpr int kMaxDemDim = 2048;

QString err(const QString &detail)
{
    return QCoreApplication::translate("GeoTiff", kErrPrefix).arg(detail);
}

TIFF *openTiff(const QString &file)
{
    TIFFSetWarningHandler(nullptr); // silence libtiff's noisy warnings
#ifdef Q_OS_WIN
    return TIFFOpenW(reinterpret_cast<const wchar_t *>(file.utf16()), "r");
#else
    return TIFFOpen(file.toLocal8Bit().constData(), "r");
#endif
}

// Interpret one sample (already in host byte order, as libtiff returns) to float.
float sampleToFloat(const uint8_t *p, uint16_t sampleFormat, uint16_t bits)
{
    if (sampleFormat == SAMPLEFORMAT_IEEEFP && bits == 32) { float v;    memcpy(&v, p, 4); return v; }
    if (sampleFormat == SAMPLEFORMAT_IEEEFP && bits == 64) { double v;   memcpy(&v, p, 8); return float(v); }
    if (sampleFormat == SAMPLEFORMAT_INT     && bits == 16) { int16_t v;  memcpy(&v, p, 2); return float(v); }
    if (bits == 16)                                         { uint16_t v; memcpy(&v, p, 2); return float(v); }
    if (sampleFormat == SAMPLEFORMAT_INT     && bits == 32) { int32_t v;  memcpy(&v, p, 4); return float(v); }
    if (bits == 32)                                         { uint32_t v; memcpy(&v, p, 4); return float(v); }
    if (sampleFormat == SAMPLEFORMAT_INT     && bits == 8)  { return float(int8_t(*p)); }
    return float(*p);
}

QRgb terrainColor(double t)
{
    t = std::clamp(t, 0.0, 1.0);
    struct Stop { double t; int r, g, b; };
    static const Stop ramp[] = {
        { 0.00,   0,  97,  71 },   // low  - dark green
        { 0.25,  90, 165,  75 },
        { 0.50, 232, 215, 125 },   //      - tan
        { 0.75, 161,  84,  38 },   //      - brown
        { 1.00, 255, 255, 255 },   // high - white
    };
    constexpr int n = int(sizeof(ramp) / sizeof(ramp[0]));
    for (int i = 1; i < n; ++i) {
        if (t <= ramp[i].t) {
            const double f = (t - ramp[i - 1].t) / (ramp[i].t - ramp[i - 1].t);
            const int r = int(std::lround(ramp[i - 1].r + f * (ramp[i].r - ramp[i - 1].r)));
            const int g = int(std::lround(ramp[i - 1].g + f * (ramp[i].g - ramp[i - 1].g)));
            const int b = int(std::lround(ramp[i - 1].b + f * (ramp[i].b - ramp[i - 1].b)));
            return qRgb(r, g, b);
        }
    }
    return qRgb(255, 255, 255);
}

bool isNoData(float v, bool hasNoData, double noData)
{
    return std::isnan(v) || std::isinf(v) || (v < -1.0e30f) ||
           (hasNoData && (std::abs(double(v) - noData) < 1e-3));
}

QImage colorize(const QVector<float> &grid, int w, int h, bool hasNoData, double noData, double &outLo, double &outHi)
{
    QVector<float> valid;
    valid.reserve(grid.size() / 4 + 1);
    for (float v : grid) {
        if (!isNoData(v, hasNoData, noData)) {
            valid.append(v);
        }
    }
    if (valid.isEmpty()) {
        return QImage();
    }
    std::sort(valid.begin(), valid.end());
    const float lo = valid[int(valid.size() * 0.02)];
    const float hi = valid[int(valid.size() * 0.98)];
    const double span = (hi > lo) ? double(hi - lo) : 1.0;
    outLo = lo;
    outHi = hi;

    QImage img(w, h, QImage::Format_ARGB32);
    for (int y = 0; y < h; ++y) {
        QRgb *line = reinterpret_cast<QRgb *>(img.scanLine(y));
        for (int x = 0; x < w; ++x) {
            const float v = grid[y * w + x];
            if (isNoData(v, hasNoData, noData)) {
                line[x] = qRgba(0, 0, 0, 0);
            } else {
                const QRgb c = terrainColor((double(v) - lo) / span);
                line[x] = qRgba(qRed(c), qGreen(c), qBlue(c), 255);
            }
        }
    }
    return img;
}

} // namespace

bool GeoDemDecoder::isElevationRaster(const QString &file)
{
    TIFF *tif = openTiff(file);
    if (!tif) {
        return false;
    }
    uint16_t spp = 1;
    TIFFGetFieldDefaulted(tif, TIFFTAG_SAMPLESPERPIXEL, &spp);
    TIFFClose(tif);
    return spp == 1;
}

bool GeoDemDecoder::decodeColorized(const QString &file, QImage &outImage, double &minElevation, double &maxElevation,
                                    QVector<float> &outGrid, int &outGridW, int &outGridH, QString &errorString,
                                    const std::function<void(int)> &progress)
{
    errorString.clear();
    minElevation = 0.0;
    maxElevation = 0.0;
    outGrid.clear();
    outGridW = 0;
    outGridH = 0;

    TIFF *tif = openTiff(file);
    if (!tif) {
        errorString = err(QCoreApplication::translate("GeoTiff", "Could not open GeoTIFF."));
        return false;
    }

    uint32_t w = 0, h = 0;
    uint16_t spp = 1, bits = 32, sampleFormat = SAMPLEFORMAT_UINT;
    TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &w);
    TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &h);
    TIFFGetFieldDefaulted(tif, TIFFTAG_SAMPLESPERPIXEL, &spp);
    TIFFGetFieldDefaulted(tif, TIFFTAG_BITSPERSAMPLE, &bits);
    TIFFGetFieldDefaulted(tif, TIFFTAG_SAMPLEFORMAT, &sampleFormat);

    if (w == 0 || h == 0) {
        TIFFClose(tif);
        errorString = err(QCoreApplication::translate("GeoTiff", "Missing image dimensions."));
        return false;
    }
    if (spp != 1) {
        TIFFClose(tif);
        errorString = err(QCoreApplication::translate("GeoTiff", "Not a single-band elevation raster."));
        return false;
    }

    // Read the (optional) GDAL nodata value. GDAL_NODATA is a custom tag that libtiff may register
    // as "passcount" -- which requires TWO va_args (count + value). Query the field and call the
    // matching form, otherwise libtiff reads a garbage pointer and crashes.
    bool   hasNoData = false;
    double noData = 0.0;
    {
        const TIFFField *fip = TIFFFieldWithTag(tif, TIFFTAG_GDAL_NODATA);
        char *ndStr = nullptr;
        bool got = false;
        if (fip) {
            if (TIFFFieldPassCount(fip)) {
                uint32_t count = 0; // wide enough for both TIFF_VARIABLE (u16) and TIFF_VARIABLE2 (u32)
                got = (TIFFGetField(tif, TIFFTAG_GDAL_NODATA, &count, &ndStr) == 1);
            } else {
                got = (TIFFGetField(tif, TIFFTAG_GDAL_NODATA, &ndStr) == 1);
            }
        }
        if (got && ndStr) {
            bool ok = false;
            const double v = QString::fromLatin1(ndStr).trimmed().toDouble(&ok);
            if (ok) { hasNoData = true; noData = v; }
        }
    }

    const int  bytesPerSample = bits / 8;
    const int  step   = std::max(1, int((std::max(w, h) + kMaxDemDim - 1) / kMaxDemDim));
    const int  outW   = int((w + step - 1) / step);
    const int  outH   = int((h + step - 1) / step);

    QVector<float> grid(outW * outH, std::numeric_limits<float>::quiet_NaN());

    if (TIFFIsTiled(tif)) {
        uint32_t tw = 0, tl = 0;
        TIFFGetField(tif, TIFFTAG_TILEWIDTH, &tw);
        TIFFGetField(tif, TIFFTAG_TILELENGTH, &tl);
        const tmsize_t tileBytes = TIFFTileSize(tif);
        QByteArray tbuf(int(tileBytes), 0);
        // Report per row of tiles: often enough to look live, rarely enough to cost nothing
        const uint32_t tileRows = (h + tl - 1) / tl;
        uint32_t       tileRow  = 0;
        for (uint32_t ty = 0; ty < h; ty += tl) {
            if (progress) {
                progress(int(100.0 * tileRow++ / tileRows));
            }
            for (uint32_t tx = 0; tx < w; tx += tw) {
                if (TIFFReadTile(tif, tbuf.data(), tx, ty, 0, 0) < 0) {
                    continue;
                }
                const uint8_t *base = reinterpret_cast<const uint8_t *>(tbuf.constData());
                for (uint32_t r = 0; (r < tl) && (ty + r < h); ++r) {
                    const uint32_t gy = ty + r;
                    if (gy % step) continue;
                    for (uint32_t c = 0; (c < tw) && (tx + c < w); ++c) {
                        const uint32_t gx = tx + c;
                        if (gx % step) continue;
                        grid[(gy / step) * outW + (gx / step)] =
                            sampleToFloat(base + (qint64(r) * tw + c) * bytesPerSample, sampleFormat, bits);
                    }
                }
            }
        }
    } else {
        const tmsize_t scanBytes = TIFFScanlineSize(tif);
        QByteArray buf(int(scanBytes), 0);
        for (uint32_t row = 0; row < h; ++row) {
            if (progress && ((row % 256) == 0)) {
                progress(int(100.0 * row / h));
            }
            if (TIFFReadScanline(tif, buf.data(), row) < 0) {
                TIFFClose(tif);
                errorString = err(QCoreApplication::translate("GeoTiff", "Failed to read elevation raster (row %1).").arg(row));
                return false;
            }
            if (row % step) continue;
            const uint8_t *base = reinterpret_cast<const uint8_t *>(buf.constData());
            const int oy = int(row / step);
            for (uint32_t col = 0; col < w; col += step) {
                grid[oy * outW + int(col / step)] = sampleToFloat(base + qint64(col) * bytesPerSample, sampleFormat, bits);
            }
        }
    }

    TIFFClose(tif);

    // Normalize nodata cells to NaN so both colorize and coordinate sampling treat them uniformly.
    for (float &v : grid) {
        if (isNoData(v, hasNoData, noData)) {
            v = std::numeric_limits<float>::quiet_NaN();
        }
    }

    outImage = colorize(grid, outW, outH, false, 0.0, minElevation, maxElevation);
    if (outImage.isNull()) {
        errorString = err(QCoreApplication::translate("GeoTiff", "Elevation raster has no valid data."));
        return false;
    }

    outGrid  = grid;    // row-major, row 0 = north; nodata = NaN
    outGridW = outW;
    outGridH = outH;

    qCDebug(GeoDemDecoderLog) << "DEM colorized" << file << w << "x" << h << "-> grid" << outW << "x" << outH
                              << "bits" << bits << "sampleFormat" << sampleFormat << "nodata" << (hasNoData ? noData : 0.0);
    return true;
}
