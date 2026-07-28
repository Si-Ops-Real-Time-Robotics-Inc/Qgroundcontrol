/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "GeoTiffReader.h"
#include "QGCGeo.h"
#include "QGCLoggingCategory.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QFile>
#include <QtCore/QHash>
#include <QtCore/QVector>
#include <QtCore/QtEndian>
#include <QtCore/qmath.h>

#include <cstring>

QGC_LOGGING_CATEGORY(GeoTiffReaderLog, "qgc.utilities.geotiffreader")

namespace {

const char *kErrPrefix = QT_TRANSLATE_NOOP("GeoTiff", "GeoTIFF load failed. %1");

// TIFF tag ids
constexpr quint16 kTagImageWidth          = 256;
constexpr quint16 kTagImageLength         = 257;
constexpr quint16 kTagModelPixelScale     = 33550;  // 3 x DOUBLE
constexpr quint16 kTagModelTiepoint       = 33922;  // 6*n x DOUBLE
constexpr quint16 kTagModelTransformation = 34264;  // 16 x DOUBLE
constexpr quint16 kTagGeoKeyDirectory     = 34735;  // n x SHORT

// GeoKey ids (within kTagGeoKeyDirectory)
constexpr quint16 kGeoKeyGTModelType      = 1024;   // 1=Projected, 2=Geographic
constexpr quint16 kGeoKeyGeographicType   = 2048;   // EPSG geographic CS (e.g. 4326)
constexpr quint16 kGeoKeyProjectedCSType  = 3072;   // EPSG projected CS (e.g. 32633)

// TIFF field type -> byte size
int tiffTypeSize(quint16 type)
{
    switch (type) {
    case 1:  case 2:  case 6:  case 7:  return 1;   // BYTE / ASCII / SBYTE / UNDEFINED
    case 3:  case 8:                    return 2;   // SHORT / SSHORT
    case 4:  case 9:  case 11:          return 4;   // LONG / SLONG / FLOAT
    case 5:  case 10: case 12:          return 8;   // RATIONAL / SRATIONAL / DOUBLE
    default:                            return 0;
    }
}

QString err(const QString &detail)
{
    return QCoreApplication::translate("GeoTiff", kErrPrefix).arg(detail);
}

struct TiffEntry {
    quint16     type = 0;
    quint32     count = 0;
    QByteArray  value;  // raw 4-byte value/offset field
};

} // namespace

bool GeoTiffReader::readBounds(const QString &file, QGeoCoordinate &swCorner, QGeoCoordinate &neCorner, QString &errorString)
{
    errorString.clear();

    QFile f(file);
    if (!f.open(QIODevice::ReadOnly)) {
        errorString = err(QCoreApplication::translate("GeoTiff", "Cannot open file: %1").arg(f.errorString()));
        return false;
    }

    const QByteArray hdr = f.read(8);
    if (hdr.size() != 8) {
        errorString = err(QCoreApplication::translate("GeoTiff", "File is too small to be a TIFF."));
        return false;
    }

    bool le = false;
    if (hdr[0] == 'I' && hdr[1] == 'I') {
        le = true;
    } else if (hdr[0] == 'M' && hdr[1] == 'M') {
        le = false;
    } else {
        errorString = err(QCoreApplication::translate("GeoTiff", "Not a TIFF file."));
        return false;
    }

    const auto u16 = [le](const uchar *p) { return le ? qFromLittleEndian<quint16>(p) : qFromBigEndian<quint16>(p); };
    const auto u32 = [le](const uchar *p) { return le ? qFromLittleEndian<quint32>(p) : qFromBigEndian<quint32>(p); };
    const auto f64 = [le](const uchar *p) {
        quint64 raw = le ? qFromLittleEndian<quint64>(p) : qFromBigEndian<quint64>(p);
        double d;
        memcpy(&d, &raw, sizeof(d));
        return d;
    };

    const uchar *h = reinterpret_cast<const uchar *>(hdr.constData());
    const quint16 magic = u16(h + 2);
    if (magic == 43) {
        errorString = err(QCoreApplication::translate("GeoTiff", "BigTIFF is not yet supported."));
        return false;
    }
    if (magic != 42) {
        errorString = err(QCoreApplication::translate("GeoTiff", "Not a TIFF file."));
        return false;
    }

    const quint32 ifdOffset = u32(h + 4);
    if (!f.seek(ifdOffset)) {
        errorString = err(QCoreApplication::translate("GeoTiff", "Corrupt TIFF (bad IFD offset)."));
        return false;
    }

    const QByteArray cntBuf = f.read(2);
    if (cntBuf.size() != 2) {
        errorString = err(QCoreApplication::translate("GeoTiff", "Corrupt TIFF (truncated IFD)."));
        return false;
    }
    const quint16 numEntries = u16(reinterpret_cast<const uchar *>(cntBuf.constData()));

    const QByteArray entriesBuf = f.read(int(numEntries) * 12);
    if (entriesBuf.size() != int(numEntries) * 12) {
        errorString = err(QCoreApplication::translate("GeoTiff", "Corrupt TIFF (truncated IFD entries)."));
        return false;
    }

    QHash<quint16, TiffEntry> tags;
    for (int i = 0; i < numEntries; ++i) {
        const uchar *e = reinterpret_cast<const uchar *>(entriesBuf.constData()) + (i * 12);
        TiffEntry entry;
        const quint16 tag = u16(e);
        entry.type  = u16(e + 2);
        entry.count = u32(e + 4);
        entry.value = QByteArray(reinterpret_cast<const char *>(e + 8), 4);
        tags.insert(tag, entry);
    }

    // Read `count` values of a DOUBLE tag (may be inline or at an offset).
    const auto readDoubles = [&](quint16 tag) -> QVector<double> {
        QVector<double> out;
        if (!tags.contains(tag)) {
            return out;
        }
        const TiffEntry &en = tags.value(tag);
        const int sz = tiffTypeSize(en.type);
        if ((sz == 0) || (en.count == 0)) {
            return out;
        }
        const quint64 nbytes = quint64(sz) * en.count;
        QByteArray data;
        if (nbytes <= 4) {
            data = en.value.left(int(nbytes));
        } else {
            const quint32 off = u32(reinterpret_cast<const uchar *>(en.value.constData()));
            if (!f.seek(off)) {
                return out;
            }
            data = f.read(int(nbytes));
            if (data.size() != int(nbytes)) {
                return out;
            }
        }
        const uchar *dp = reinterpret_cast<const uchar *>(data.constData());
        out.reserve(int(en.count));
        for (quint32 k = 0; k < en.count; ++k) {
            if (en.type == 12) {          // DOUBLE
                out.append(f64(dp + (k * 8)));
            } else if (en.type == 11) {   // FLOAT (rare for these tags)
                quint32 raw = u32(dp + (k * 4));
                float fl;
                memcpy(&fl, &raw, sizeof(fl));
                out.append(double(fl));
            }
        }
        return out;
    };

    // Read `count` SHORT values of a tag.
    const auto readShorts = [&](quint16 tag) -> QVector<quint16> {
        QVector<quint16> out;
        if (!tags.contains(tag)) {
            return out;
        }
        const TiffEntry &en = tags.value(tag);
        if ((en.type != 3) || (en.count == 0)) {
            return out;
        }
        const quint64 nbytes = quint64(2) * en.count;
        QByteArray data;
        if (nbytes <= 4) {
            data = en.value.left(int(nbytes));
        } else {
            const quint32 off = u32(reinterpret_cast<const uchar *>(en.value.constData()));
            if (!f.seek(off)) {
                return out;
            }
            data = f.read(int(nbytes));
            if (data.size() != int(nbytes)) {
                return out;
            }
        }
        const uchar *dp = reinterpret_cast<const uchar *>(data.constData());
        out.reserve(int(en.count));
        for (quint32 k = 0; k < en.count; ++k) {
            out.append(u16(dp + (k * 2)));
        }
        return out;
    };

    // Read a scalar SHORT/LONG tag (always inline for count == 1).
    const auto readScalar = [&](quint16 tag) -> quint32 {
        if (!tags.contains(tag)) {
            return 0;
        }
        const TiffEntry &en = tags.value(tag);
        const uchar *vp = reinterpret_cast<const uchar *>(en.value.constData());
        if (en.type == 3) {
            return u16(vp);
        }
        if (en.type == 4) {
            return u32(vp);
        }
        return 0;
    };

    const quint32 width  = readScalar(kTagImageWidth);
    const quint32 height = readScalar(kTagImageLength);
    if ((width == 0) || (height == 0)) {
        errorString = err(QCoreApplication::translate("GeoTiff", "Missing image dimensions."));
        return false;
    }

    // Determine the pixel->model affine (north-up): origin (top-left model coord) + pixel size.
    double originX = 0.0, originY = 0.0, psx = 0.0, psy = 0.0;
    const QVector<double> scale    = readDoubles(kTagModelPixelScale);
    const QVector<double> tiepoint = readDoubles(kTagModelTiepoint);
    const QVector<double> xform    = readDoubles(kTagModelTransformation);

    if ((scale.size() >= 2) && (tiepoint.size() >= 6)) {
        psx = scale[0];
        psy = scale[1];
        // tiepoint: (I,J,K, X,Y,Z) -- raster point (I,J) maps to model point (X,Y)
        originX = tiepoint[3] - (tiepoint[0] * psx);
        originY = tiepoint[4] + (tiepoint[1] * psy);
    } else if (xform.size() >= 16) {
        // 4x4 row-major model transformation matrix
        originX = xform[3];
        originY = xform[7];
        psx = xform[0];
        psy = -xform[5];
    } else {
        errorString = err(QCoreApplication::translate("GeoTiff", "File has no georeferencing (model transformation) tags."));
        return false;
    }

    if (qFuzzyIsNull(psx) || qFuzzyIsNull(psy)) {
        errorString = err(QCoreApplication::translate("GeoTiff", "Invalid pixel scale."));
        return false;
    }

    // Projected/geographic bounding box of the (north-up) raster.
    double minX = originX;
    double maxX = originX + (double(width) * psx);
    double maxY = originY;
    double minY = originY - (double(height) * psy);
    if (maxX < minX) { qSwap(minX, maxX); }
    if (maxY < minY) { qSwap(minY, maxY); }

    // Determine CRS from the GeoKeyDirectory.
    int epsg = 0;
    bool geographic = false;
    const QVector<quint16> gk = readShorts(kTagGeoKeyDirectory);
    if (gk.size() >= 4) {
        const int numKeys = int(gk[3]);
        for (int k = 0; k < numKeys; ++k) {
            const int base = 4 + (k * 4);
            if ((base + 3) >= gk.size()) {
                break;
            }
            const quint16 keyId          = gk[base];
            const quint16 tiffTagLocation = gk[base + 1];
            const quint16 valueOffset    = gk[base + 3];
            if (tiffTagLocation != 0) {
                continue; // value stored in another tag -- not needed for the keys we use
            }
            if (keyId == kGeoKeyProjectedCSType) {
                epsg = valueOffset;
                geographic = false;
            } else if ((keyId == kGeoKeyGeographicType) && (epsg == 0)) {
                epsg = valueOffset;
                geographic = true;
            } else if (keyId == kGeoKeyGTModelType) {
                if (valueOffset == 2) {
                    geographic = true;
                }
            }
        }
    }

    const auto toGeo = [&](double x, double y, QGeoCoordinate &c) -> bool {
        if (geographic || (epsg == 4326) || (epsg == 0)) {
            // Already geographic: x = longitude, y = latitude
            if ((qAbs(x) > 180.0) || (qAbs(y) > 90.0)) {
                return false;
            }
            c.setLongitude(x);
            c.setLatitude(y);
            return true;
        }
        if ((epsg >= 32601) && (epsg <= 32660)) {   // WGS84 / UTM north
            return QGCGeo::convertUTMToGeo(x, y, epsg - 32600, false, c);
        }
        if ((epsg >= 32701) && (epsg <= 32760)) {   // WGS84 / UTM south
            return QGCGeo::convertUTMToGeo(x, y, epsg - 32700, true, c);
        }
        return false;
    };

    QGeoCoordinate sw, ne;
    if (!toGeo(minX, minY, sw) || !toGeo(maxX, maxY, ne)) {
        errorString = err(QCoreApplication::translate("GeoTiff", "Unsupported coordinate system (EPSG:%1). Only WGS84 lat/lon and WGS84 UTM are supported.").arg(epsg));
        return false;
    }

    // Normalize ordering to a true SW / NE box.
    swCorner.setLatitude(qMin(sw.latitude(), ne.latitude()));
    swCorner.setLongitude(qMin(sw.longitude(), ne.longitude()));
    neCorner.setLatitude(qMax(sw.latitude(), ne.latitude()));
    neCorner.setLongitude(qMax(sw.longitude(), ne.longitude()));

    qCDebug(GeoTiffReaderLog) << "Loaded GeoTIFF" << file << "epsg" << epsg << "geographic" << geographic
                              << "size" << width << "x" << height << "sw" << swCorner << "ne" << neCorner;
    return true;
}
