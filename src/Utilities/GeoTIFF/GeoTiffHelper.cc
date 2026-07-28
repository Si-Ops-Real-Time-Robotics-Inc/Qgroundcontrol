/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "GeoTiffHelper.h"
#include "GeoTiffReader.h"
#include "GeoDemDecoder.h"
#include "GeoOrthoDecoder.h"
#include "RasterOverlayLayer.h"
#include "QmlObjectListModel.h"
#include "QGCLoggingCategory.h"

#include <QtConcurrent/QtConcurrentRun>
#include <QtCore/QCoreApplication>
#include <QtCore/QDataStream>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QFutureWatcher>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QStandardPaths>
#include <QtCore/QUrl>
#include <QtCore/QVector>
#include <QtGui/QImage>
#include <QtGui/QImageReader>

#include <cmath>
#include <limits>

QGC_LOGGING_CATEGORY(GeoTiffHelperLog, "qgc.utilities.geotiffhelper")

namespace {
// Maximum long-edge (px) an overlay image is downsampled to, to bound memory/GPU usage.
constexpr int kMaxOverlayDim = 4096;
// Pixels whose channels are all <= this are treated as nodata and made transparent (orthos).
constexpr int kNoDataThreshold = 16;
// Identifies the elevation grid format, so a grid written by a different build is rejected rather than
// misread as garbage elevations.
constexpr quint32 kGridMagic = 0x51474344;  // "QGCD"
// Share of the work reading the source accounts for. The rest is scaling and writing the png.
constexpr int kReadPercent = 90;

void keyNoDataTransparent(QImage &img)
{
    for (int y = 0; y < img.height(); ++y) {
        QRgb *line = reinterpret_cast<QRgb *>(img.scanLine(y));
        for (int x = 0; x < img.width(); ++x) {
            const QRgb p = line[x];
            if ((qAlpha(p) != 0) && (qRed(p) <= kNoDataThreshold) && (qGreen(p) <= kNoDataThreshold) && (qBlue(p) <= kNoDataThreshold)) {
                line[x] = qRgba(0, 0, 0, 0);
            }
        }
    }
}

bool writeGrid(const QString &file, const QVector<float> &grid, int gridW, int gridH)
{
    QFile f(file);
    if (!f.open(QIODevice::WriteOnly)) {
        return false;
    }
    QDataStream stream(&f);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
    stream << kGridMagic << qint32(gridW) << qint32(gridH);
    stream.writeRawData(reinterpret_cast<const char *>(grid.constData()), int(grid.size() * sizeof(float)));
    return stream.status() == QDataStream::Ok;
}

bool readGrid(const QString &file, QVector<float> &grid, int &gridW, int &gridH)
{
    grid.clear();
    gridW = gridH = 0;

    QFile f(file);
    if (!f.open(QIODevice::ReadOnly)) {
        return false;
    }
    QDataStream stream(&f);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream.setFloatingPointPrecision(QDataStream::SinglePrecision);

    quint32 magic = 0;
    qint32  w = 0, h = 0;
    stream >> magic >> w >> h;
    if ((stream.status() != QDataStream::Ok) || (magic != kGridMagic) || (w <= 0) || (h <= 0)) {
        return false;
    }

    grid.resize(qsizetype(w) * h);
    const int wanted = int(grid.size() * sizeof(float));
    if (stream.readRawData(reinterpret_cast<char *>(grid.data()), wanted) != wanted) {
        grid.clear();
        return false;
    }

    gridW = w;
    gridH = h;
    return true;
}
} // namespace

GeoTiffHelper::GeoTiffHelper(QObject *parent)
    : QObject(parent)
    , _layers(new QmlObjectListModel(this))
{
    _restoreLayers();
}

QDir GeoTiffHelper::_cacheDir() const
{
    // Deliberately not the OS temp dir: these files have to survive a restart to be worth caching.
    QDir dir(QDir(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation)).filePath(QStringLiteral("GeoTiffOverlays")));
    if (!dir.exists()) {
        dir.mkpath(QStringLiteral("."));
    }
    return dir;
}

QString GeoTiffHelper::_layersFile() const
{
    return _cacheDir().filePath(QStringLiteral("layers.json"));
}

QStringList GeoTiffHelper::fileDialogGeoTiffFilters()
{
    return QStringList(QCoreApplication::translate("GeoTiff", "GeoTIFF Files (*.tif *.tiff)"));
}

void GeoTiffHelper::_decode(QPromise<GeoTiffDecodeResult> &promise, const QString &sourceFile, const QString &cacheDirPath, const QString &stem)
{
    GeoTiffDecodeResult result;

    promise.setProgressRange(0, 100);
    // Reading the source dominates; writing the png is the small tail, so reading owns 0..kReadPercent
    const auto reportRead = [&promise](int percent) { promise.setProgressValue(percent * kReadPercent / 100); };

    // 1. Georeferencing.
    QGeoCoordinate sw, ne;
    if (!GeoTiffReader::readBounds(sourceFile, sw, ne, result.errorString)) {
        qCWarning(GeoTiffHelperLog) << "readBounds failed:" << result.errorString;
        promise.addResult(result);
        return;
    }

    // 2. Display image: single-band -> colorized DEM; multi-band -> imagery with nodata keyed out.
    QImage img;
    double minElev = 0.0, maxElev = 0.0;
    const bool elevation = GeoDemDecoder::isElevationRaster(sourceFile);
    if (elevation) {
        if (!GeoDemDecoder::decodeColorized(sourceFile, img, minElev, maxElev, result.grid, result.gridW, result.gridH, result.errorString, reportRead)) {
            qCWarning(GeoTiffHelperLog) << "DEM decode failed:" << result.errorString;
            promise.addResult(result);
            return;
        }
    } else if (GeoOrthoDecoder::canDecode(sourceFile)) {
        // libtiff subsamples while reading, which is the only way to open the multi-GB orthos that
        // photogrammetry produces -- QImageReader decodes the whole image before scaling.
        if (!GeoOrthoDecoder::decode(sourceFile, kMaxOverlayDim, img, result.errorString, reportRead)) {
            qCWarning(GeoTiffHelperLog) << "Ortho decode failed:" << result.errorString;
            promise.addResult(result);
            return;
        }
    } else {
        // Uncommon pixel layouts (paletted, 16-bit, planar) that Qt's TIFF plugin still handles.
        QImageReader reader(sourceFile);
        reader.setAutoTransform(true);
        const QSize srcSize = reader.size();
        if (srcSize.isValid() && ((srcSize.width() > kMaxOverlayDim) || (srcSize.height() > kMaxOverlayDim))) {
            reader.setScaledSize(srcSize.scaled(kMaxOverlayDim, kMaxOverlayDim, Qt::KeepAspectRatio));
        }
        img = reader.read();
        if (img.isNull()) {
            result.errorString = QCoreApplication::translate("GeoTiff", "GeoTIFF load failed. Could not decode image: %1").arg(reader.errorString());
            promise.addResult(result);
            return;
        }
        img = img.convertToFormat(QImage::Format_ARGB32);
        keyNoDataTransparent(img);
    }

    promise.setProgressValue(kReadPercent);

    if ((img.width() > kMaxOverlayDim) || (img.height() > kMaxOverlayDim)) {
        img = img.scaled(kMaxOverlayDim, kMaxOverlayDim, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    }

    // 3. Cache the decoded results. The layer owns these files until it is removed.
    const QDir      dir     = QDir(cacheDirPath);
    const QString   pngPath = dir.filePath(stem + QStringLiteral(".png"));
    if (!img.save(pngPath, "PNG")) {
        result.errorString = QCoreApplication::translate("GeoTiff", "GeoTIFF load failed. Could not write overlay image to %1").arg(pngPath);
        promise.addResult(result);
        return;
    }

    QString gridPath;
    if (elevation && !result.grid.isEmpty()) {
        gridPath = dir.filePath(stem + QStringLiteral(".grid"));
        if (!writeGrid(gridPath, result.grid, result.gridW, result.gridH)) {
            // The layer still works, it just won't survive a restart without decoding again
            qCWarning(GeoTiffHelperLog) << "Could not cache elevation grid to" << gridPath;
            gridPath.clear();
        }
    }

    const QFileInfo sourceInfo(sourceFile);
    result.info.name            = sourceInfo.fileName();
    result.info.sourceFile      = sourceInfo.absoluteFilePath();
    result.info.sourceSize      = sourceInfo.size();
    result.info.sourceModified  = sourceInfo.lastModified().toSecsSinceEpoch();
    result.info.imageUrl        = QUrl::fromLocalFile(pngPath);
    result.info.gridFile        = gridPath;
    result.info.swCorner        = sw;
    result.info.neCorner        = ne;
    result.info.elevation       = elevation;
    result.info.minElevation    = minElev;
    result.info.maxElevation    = maxElev;
    result.ok                   = true;
    promise.setProgressValue(100);

    qCDebug(GeoTiffHelperLog) << "Decoded" << sourceFile << "elevation" << elevation << "-> cache" << pngPath;
    promise.addResult(result);
}

void GeoTiffHelper::_startDecode(const QString &sourceFile, bool layerVisible, double opacity)
{
    // Allocate the cache file name here rather than on the worker, so concurrent loads can't race for it
    const QString stem = QStringLiteral("overlay_%1").arg(++_counter);
    const QString dir  = _cacheDir().absolutePath();

    _pendingLoads++;
    _loadProgress = 0;
    emit loadingChanged();

    auto *watcher = new QFutureWatcher<GeoTiffDecodeResult>(this);
    connect(watcher, &QFutureWatcherBase::progressValueChanged, this, [this](int value) {
        _loadProgress = value;
        emit loadingChanged();
    });
    connect(watcher, &QFutureWatcherBase::finished, this, [this, watcher, layerVisible, opacity]() {
        GeoTiffDecodeResult result = watcher->result();
        watcher->deleteLater();

        _pendingLoads--;
        emit loadingChanged();

        if (!result.ok) {
            emit loadFailed(result.errorString);
            return;
        }

        // Whatever the user had set on the layer is theirs, only the decoded data is ours to fill in
        result.info.layerVisible = layerVisible;
        result.info.opacity      = opacity;

        _appendLayer(result.info, result.grid, result.gridW, result.gridH);
        _saveLayers();
    });

    // Decoding a large ortho takes the better part of a minute. On the UI thread that freezes the whole
    // app, which is why this is worth the round trip through a worker.
    watcher->setFuture(QtConcurrent::run(&GeoTiffHelper::_decode, sourceFile, dir, stem));
}

void GeoTiffHelper::_connectLayer(RasterOverlayLayer *layer)
{
    // Show/hide and opacity are part of what gets restored, so they have to be persisted as they change
    connect(layer, &RasterOverlayLayer::layerVisibleChanged, this, &GeoTiffHelper::_saveLayers);
    connect(layer, &RasterOverlayLayer::opacityChanged,      this, &GeoTiffHelper::_saveLayers);
}

void GeoTiffHelper::_appendLayer(const RasterOverlayLayerInfo &info, const QVector<float> &grid, int gridW, int gridH)
{
    auto *layer = new RasterOverlayLayer(info, this);
    if (info.elevation && !grid.isEmpty()) {
        layer->setElevationGrid(grid, gridW, gridH);
    }
    _connectLayer(layer);
    _layers->append(layer);

    emit layerAdded(layer);
    if (info.elevation) {
        emit demLayersChanged();
    }
}

void GeoTiffHelper::loadFile(const QString &file)
{
    QString localFile = file;
    if (localFile.startsWith(QStringLiteral("file:"))) {
        localFile = QUrl(localFile).toLocalFile();
    }

    _startDecode(localFile, true /* layerVisible */, 0.7 /* opacity */);
}

void GeoTiffHelper::removeLayer(int index)
{
    if ((index < 0) || (index >= _layers->count())) {
        return;
    }
    auto *layer = _layers->value<RasterOverlayLayer *>(index);
    const bool wasElevation = layer && layer->elevation();
    _layers->removeAt(index);
    if (layer) {
        // Only the layers which are still loaded are worth caching, so this layer's files go with it
        layer->deleteCacheFiles();
        layer->deleteLater();
    }
    _saveLayers();
    if (wasElevation) {
        emit demLayersChanged();
    }
}

void GeoTiffHelper::clear()
{
    const bool hadElevation = hasElevationLayer();
    for (int i = 0; i < _layers->count(); ++i) {
        const auto *layer = _layers->value<RasterOverlayLayer *>(i);
        if (layer) {
            layer->deleteCacheFiles();
        }
    }
    _layers->clearAndDeleteContents();
    _saveLayers();
    if (hadElevation) {
        emit demLayersChanged();
    }
}

void GeoTiffHelper::_saveLayers() const
{
    if (_restoring) {
        return;
    }

    QJsonArray layersJson;
    for (int i = 0; i < _layers->count(); ++i) {
        const auto *layer = _layers->value<RasterOverlayLayer *>(i);
        if (!layer) {
            continue;
        }
        const RasterOverlayLayerInfo &info = layer->info();

        QJsonObject layerJson;
        layerJson[QStringLiteral("name")]           = info.name;
        layerJson[QStringLiteral("sourceFile")]     = info.sourceFile;
        layerJson[QStringLiteral("sourceSize")]     = double(info.sourceSize);
        layerJson[QStringLiteral("sourceModified")] = double(info.sourceModified);
        layerJson[QStringLiteral("image")]          = info.imageUrl.toLocalFile();
        layerJson[QStringLiteral("grid")]           = info.gridFile;
        layerJson[QStringLiteral("swLat")]          = info.swCorner.latitude();
        layerJson[QStringLiteral("swLon")]          = info.swCorner.longitude();
        layerJson[QStringLiteral("neLat")]          = info.neCorner.latitude();
        layerJson[QStringLiteral("neLon")]          = info.neCorner.longitude();
        layerJson[QStringLiteral("elevation")]      = info.elevation;
        layerJson[QStringLiteral("minElevation")]   = info.minElevation;
        layerJson[QStringLiteral("maxElevation")]   = info.maxElevation;
        layerJson[QStringLiteral("visible")]        = info.layerVisible;
        layerJson[QStringLiteral("opacity")]        = info.opacity;
        layersJson.append(layerJson);
    }

    QFile f(_layersFile());
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        qCWarning(GeoTiffHelperLog) << "Could not save overlay layer list to" << _layersFile();
        return;
    }
    f.write(QJsonDocument(layersJson).toJson(QJsonDocument::Indented));
}

void GeoTiffHelper::_restoreLayers()
{
    QFile f(_layersFile());
    if (!f.open(QIODevice::ReadOnly)) {
        return;     // Nothing cached yet, which is the normal first run
    }
    const QJsonArray layersJson = QJsonDocument::fromJson(f.readAll()).array();
    f.close();

    _restoring = true;
    bool listChanged = false;

    for (const QJsonValue &layerValue : layersJson) {
        const QJsonObject layerJson = layerValue.toObject();

        RasterOverlayLayerInfo info;
        info.name           = layerJson[QStringLiteral("name")].toString();
        info.sourceFile     = layerJson[QStringLiteral("sourceFile")].toString();
        info.sourceSize     = qint64(layerJson[QStringLiteral("sourceSize")].toDouble());
        info.sourceModified = qint64(layerJson[QStringLiteral("sourceModified")].toDouble());
        info.gridFile       = layerJson[QStringLiteral("grid")].toString();
        info.swCorner       = QGeoCoordinate(layerJson[QStringLiteral("swLat")].toDouble(), layerJson[QStringLiteral("swLon")].toDouble());
        info.neCorner       = QGeoCoordinate(layerJson[QStringLiteral("neLat")].toDouble(), layerJson[QStringLiteral("neLon")].toDouble());
        info.elevation      = layerJson[QStringLiteral("elevation")].toBool();
        info.minElevation   = layerJson[QStringLiteral("minElevation")].toDouble();
        info.maxElevation   = layerJson[QStringLiteral("maxElevation")].toDouble();
        info.layerVisible   = layerJson[QStringLiteral("visible")].toBool(true);
        info.opacity        = layerJson[QStringLiteral("opacity")].toDouble(0.7);

        const QString imageFile = layerJson[QStringLiteral("image")].toString();
        info.imageUrl = QUrl::fromLocalFile(imageFile);

        // Highest _counter seen keeps new cache file names unique against the restored ones
        const QString stem = QFileInfo(imageFile).completeBaseName();
        _counter = qMax(_counter, stem.mid(stem.lastIndexOf(QLatin1Char('_')) + 1).toInt());

        QVector<float>  grid;
        int             gridW = 0, gridH = 0;

        bool cacheUsable = QFile::exists(imageFile);
        if (cacheUsable && info.elevation) {
            // Without its grid a DEM would draw on the map but silently stop feeding the terrain profile
            cacheUsable = !info.gridFile.isEmpty() && readGrid(info.gridFile, grid, gridW, gridH);
        }

        // A source which has been edited since it was cached must be decoded again
        const QFileInfo sourceInfo(info.sourceFile);
        if (cacheUsable && sourceInfo.exists()) {
            if ((sourceInfo.size() != info.sourceSize) || (sourceInfo.lastModified().toSecsSinceEpoch() != info.sourceModified)) {
                qCDebug(GeoTiffHelperLog) << "Source changed since it was cached, decoding again:" << info.sourceFile;
                cacheUsable = false;
            }
        }

        if (cacheUsable) {
            _appendLayer(info, grid, gridW, gridH);
            continue;
        }

        if (!sourceInfo.exists()) {
            qCWarning(GeoTiffHelperLog) << "Dropping overlay layer, cache is unusable and source is gone:" << info.sourceFile;
            listChanged = true;
            continue;
        }

        // Decode again in the background rather than holding up startup. The layer appears when it's ready.
        QFile::remove(imageFile);
        if (!info.gridFile.isEmpty()) {
            QFile::remove(info.gridFile);
        }
        _startDecode(info.sourceFile, info.layerVisible, info.opacity);
        listChanged = true;
    }

    _restoring = false;

    if (listChanged) {
        _saveLayers();
    }
    qCDebug(GeoTiffHelperLog) << "Restored" << _layers->count() << "overlay layers from cache," << _pendingLoads << "decoding";
}

bool GeoTiffHelper::hasElevationLayer() const
{
    for (int i = 0; i < _layers->count(); ++i) {
        const auto *layer = _layers->value<RasterOverlayLayer *>(i);
        if (layer && layer->elevation()) {
            return true;
        }
    }
    return false;
}

double GeoTiffHelper::elevationAt(const QGeoCoordinate &coordinate) const
{
    // Walk newest -> oldest so that where DEMs overlap the most recently loaded one wins, matching
    // the map's draw order. Each returns NaN outside its own bounds, so DEMs covering separate parts
    // of a mission combine into one continuous profile.
    for (int i = _layers->count() - 1; i >= 0; --i) {
        const auto *layer = _layers->value<RasterOverlayLayer *>(i);
        if (!layer || !layer->elevation()) {
            continue;
        }
        const double elevation = layer->elevationAt(coordinate);
        if (!std::isnan(elevation)) {
            return elevation;
        }
    }
    return std::numeric_limits<double>::quiet_NaN();
}
