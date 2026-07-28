/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "GeoOrthoDecoder.h"
#include "QGCLoggingCategory.h"

#include <QtCore/QByteArray>
#include <QtCore/QCoreApplication>
#include <QtGui/QImage>

#include <tiffio.h>

#include <algorithm>
#include <cstdint>

QGC_LOGGING_CATEGORY(GeoOrthoDecoderLog, "qgc.utilities.geoorthodecoder")

namespace {

// Pixels whose colour channels are all <= this are treated as nodata when there is no alpha band.
constexpr int kNoDataThreshold = 16;

QString err(const QString &detail)
{
    return QCoreApplication::translate("GeoTiff", "Orthomosaic load failed. %1").arg(detail);
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

struct Layout {
    uint32_t width = 0;
    uint32_t height = 0;
    uint16_t samplesPerPixel = 0;
    uint16_t bitsPerSample = 0;
    uint16_t planarConfig = PLANARCONFIG_CONTIG;
    bool     hasAlpha = false;
};

bool readLayout(TIFF *tif, Layout &layout)
{
    TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &layout.width);
    TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &layout.height);
    TIFFGetFieldDefaulted(tif, TIFFTAG_SAMPLESPERPIXEL, &layout.samplesPerPixel);
    TIFFGetFieldDefaulted(tif, TIFFTAG_BITSPERSAMPLE, &layout.bitsPerSample);
    TIFFGetFieldDefaulted(tif, TIFFTAG_PLANARCONFIG, &layout.planarConfig);

    // A 4th+ band is an alpha/mask band when ExtraSamples says so. Photogrammetry orthos normally
    // carry one to mask the area outside the flown region.
    uint16_t extraCount = 0;
    uint16_t *extraTypes = nullptr;
    if (TIFFGetField(tif, TIFFTAG_EXTRASAMPLES, &extraCount, &extraTypes) && (extraCount > 0) && extraTypes) {
        layout.hasAlpha = (extraTypes[0] == EXTRASAMPLE_ASSOCALPHA) || (extraTypes[0] == EXTRASAMPLE_UNASSALPHA);
    } else if (layout.samplesPerPixel >= 4) {
        layout.hasAlpha = true;    // no ExtraSamples tag, but a 4th band is alpha by convention
    }

    return (layout.width > 0) && (layout.height > 0);
}

bool supported(const Layout &layout)
{
    return (layout.bitsPerSample == 8) &&
           (layout.samplesPerPixel >= 3) &&
           (layout.planarConfig == PLANARCONFIG_CONTIG);
}

/// Convert one source pixel to ARGB, applying the alpha band or keying near-black as nodata.
inline QRgb toRgba(const uint8_t *p, const Layout &layout)
{
    const int r = p[0];
    const int g = p[1];
    const int b = p[2];
    if (layout.hasAlpha) {
        const int a = p[3];
        return (a == 0) ? qRgba(0, 0, 0, 0) : qRgba(r, g, b, a);
    }
    if ((r <= kNoDataThreshold) && (g <= kNoDataThreshold) && (b <= kNoDataThreshold)) {
        return qRgba(0, 0, 0, 0);
    }
    return qRgba(r, g, b, 255);
}

} // namespace

bool GeoOrthoDecoder::canDecode(const QString &file)
{
    TIFF *tif = openTiff(file);
    if (!tif) {
        return false;
    }
    Layout layout;
    const bool ok = readLayout(tif, layout) && supported(layout);
    TIFFClose(tif);
    return ok;
}

bool GeoOrthoDecoder::decode(const QString &file, int maxDim, QImage &outImage, QString &errorString,
                             const std::function<void(int)> &progress)
{
    errorString.clear();

    TIFF *tif = openTiff(file);
    if (!tif) {
        errorString = err(QCoreApplication::translate("GeoTiff", "Could not open GeoTIFF."));
        return false;
    }

    Layout layout;
    if (!readLayout(tif, layout)) {
        TIFFClose(tif);
        errorString = err(QCoreApplication::translate("GeoTiff", "Missing image dimensions."));
        return false;
    }
    if (!supported(layout)) {
        TIFFClose(tif);
        errorString = err(QCoreApplication::translate("GeoTiff", "Unsupported pixel layout (%1 bits, %2 bands).")
                              .arg(layout.bitsPerSample).arg(layout.samplesPerPixel));
        return false;
    }

    // Subsample while reading so memory tracks the output, not the (potentially multi-GB) source.
    const uint32_t longEdge = std::max(layout.width, layout.height);
    const int      step     = std::max<int>(1, int((longEdge + maxDim - 1) / maxDim));
    const int      outW     = int((layout.width  + step - 1) / step);
    const int      outH     = int((layout.height + step - 1) / step);
    const int      spp      = layout.samplesPerPixel;

    QImage img(outW, outH, QImage::Format_ARGB32);
    if (img.isNull()) {
        TIFFClose(tif);
        errorString = err(QCoreApplication::translate("GeoTiff", "Out of memory for a %1x%2 image.").arg(outW).arg(outH));
        return false;
    }
    img.fill(Qt::transparent);

    if (TIFFIsTiled(tif)) {
        uint32_t tileWidth = 0, tileLength = 0;
        TIFFGetField(tif, TIFFTAG_TILEWIDTH, &tileWidth);
        TIFFGetField(tif, TIFFTAG_TILELENGTH, &tileLength);
        if ((tileWidth == 0) || (tileLength == 0)) {
            TIFFClose(tif);
            errorString = err(QCoreApplication::translate("GeoTiff", "Invalid tile size."));
            return false;
        }
        QByteArray tileBuf(int(TIFFTileSize(tif)), 0);
        // Report per row of tiles: often enough to look live, rarely enough to cost nothing
        const uint32_t tileRows = (layout.height + tileLength - 1) / tileLength;
        uint32_t       tileRow  = 0;
        for (uint32_t ty = 0; ty < layout.height; ty += tileLength) {
            if (progress) {
                progress(int(100.0 * tileRow++ / tileRows));
            }
            for (uint32_t tx = 0; tx < layout.width; tx += tileWidth) {
                if (TIFFReadTile(tif, tileBuf.data(), tx, ty, 0, 0) < 0) {
                    continue;   // skip unreadable tiles rather than failing the whole ortho
                }
                const uint8_t *base = reinterpret_cast<const uint8_t *>(tileBuf.constData());
                for (uint32_t row = 0; (row < tileLength) && ((ty + row) < layout.height); ++row) {
                    const uint32_t gy = ty + row;
                    if (gy % step) {
                        continue;
                    }
                    QRgb *outLine = reinterpret_cast<QRgb *>(img.scanLine(int(gy / step)));
                    for (uint32_t col = 0; (col < tileWidth) && ((tx + col) < layout.width); ++col) {
                        const uint32_t gx = tx + col;
                        if (gx % step) {
                            continue;
                        }
                        const uint8_t *p = base + ((qint64(row) * tileWidth) + col) * spp;
                        outLine[gx / step] = toRgba(p, layout);
                    }
                }
            }
        }
    } else {
        QByteArray scanBuf(int(TIFFScanlineSize(tif)), 0);
        for (uint32_t row = 0; row < layout.height; ++row) {
            if (progress && ((row % 256) == 0)) {
                progress(int(100.0 * row / layout.height));
            }
            if (TIFFReadScanline(tif, scanBuf.data(), row) < 0) {
                TIFFClose(tif);
                errorString = err(QCoreApplication::translate("GeoTiff", "Failed to read image (row %1).").arg(row));
                return false;
            }
            if (row % step) {
                continue;
            }
            const uint8_t *base = reinterpret_cast<const uint8_t *>(scanBuf.constData());
            QRgb *outLine = reinterpret_cast<QRgb *>(img.scanLine(int(row / step)));
            for (uint32_t col = 0; col < layout.width; col += step) {
                outLine[col / step] = toRgba(base + (qint64(col) * spp), layout);
            }
        }
    }

    TIFFClose(tif);
    outImage = img;

    qCDebug(GeoOrthoDecoderLog) << "Ortho decoded" << file << layout.width << "x" << layout.height
                               << "-> " << outW << "x" << outH << "step" << step
                               << "bands" << spp << "alpha" << layout.hasAlpha;
    return true;
}
