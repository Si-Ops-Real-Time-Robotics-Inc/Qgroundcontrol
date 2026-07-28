/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include "RasterOverlayLayer.h"

#include <QtCore/QDir>
#include <QtCore/QObject>
#include <QtCore/QPromise>
#include <QtCore/QStringList>
#include <QtPositioning/QGeoCoordinate>

class QmlObjectListModel;

/// What decoding a GeoTIFF produces. Passed back from the worker thread, so it must stay a plain value.
struct GeoTiffDecodeResult
{
    bool                    ok = false;
    QString                 errorString;
    RasterOverlayLayerInfo  info;
    QVector<float>          grid;       ///< DEM sampling grid, empty for imagery
    int                     gridW = 0;
    int                     gridH = 0;
};

/// QML-facing manager for georeferenced GeoTIFF map overlays. Loads any number of layers -- RGB
/// orthomosaics and/or colorized DEM/elevation rasters -- each shown as an independently
/// toggleable map overlay. Registered as a QML singleton.
///
/// Decoding a GeoTIFF is slow (the better part of a minute for a multi-GB orthomosaic) so it runs on a
/// worker thread, leaving the UI responsive, and the results are cached to disk. The cache holds exactly
/// the layers which are currently loaded and nothing else, so it can't grow without bound: removing a
/// layer removes its cached files with it. Layers are remembered across runs and come back from the cache
/// without touching the source files.
class GeoTiffHelper : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QmlObjectListModel  *layers               READ layers CONSTANT)
    Q_PROPERTY(QStringList          fileDialogGeoTiffFilters READ fileDialogGeoTiffFilters CONSTANT)
    Q_PROPERTY(bool                 hasElevationLayer    READ hasElevationLayer NOTIFY demLayersChanged)
    Q_PROPERTY(bool                 loading              READ loading NOTIFY loadingChanged)
    Q_PROPERTY(int                  loadingCount         READ loadingCount NOTIFY loadingChanged)
    Q_PROPERTY(int                  loadingProgress      READ loadingProgress NOTIFY loadingChanged)

public:
    explicit GeoTiffHelper(QObject *parent = nullptr);

    QmlObjectListModel *layers() const { return _layers; }

    /// True while one or more files are being decoded in the background.
    bool loading() const { return _pendingLoads > 0; }
    int  loadingCount() const { return _pendingLoads; }
    /// 0..100 through the current decode, or -1 when that can't be said (several decodes at once).
    int  loadingProgress() const { return _pendingLoads == 1 ? _loadProgress : -1; }

    /// True while at least one DEM is loaded (i.e. the terrain profile can show a DEM line).
    bool hasElevationLayer() const;

    /// Elevation from the loaded DEMs at @a coordinate, or NaN where no DEM covers it. Every DEM is
    /// consulted, most recently loaded first, so several DEMs can tile one mission.
    Q_INVOKABLE double elevationAt(const QGeoCoordinate &coordinate) const;

    static QStringList fileDialogGeoTiffFilters();

    /// Start loading a GeoTIFF (accepts a local path or a file: URL). Returns immediately; the layer is
    /// appended once the background decode finishes, or loadFailed is emitted.
    Q_INVOKABLE void loadFile(const QString &file);

    /// Remove the layer at @a index, along with its cached files.
    Q_INVOKABLE void removeLayer(int index);

    /// Remove all layers, along with their cached files.
    Q_INVOKABLE void clear();

signals:
    void layerAdded(RasterOverlayLayer *layer);
    void loadFailed(const QString &errorString);
    void loadingChanged();
    /// The set of loaded DEMs changed, so anything sampling them (the terrain profile) must redraw.
    void demLayersChanged();

private:
    /// Decode @a sourceFile and write the results into @a cacheDirPath as @a stem.png/.grid. Runs on a
    /// worker thread, so it must not touch any member state.
    static void _decode(QPromise<GeoTiffDecodeResult> &promise, const QString &sourceFile, const QString &cacheDirPath, const QString &stem);

    /// Kick off a background decode of @a sourceFile, restoring @a layerVisible / @a opacity onto the
    /// layer once it completes.
    void _startDecode(const QString &sourceFile, bool layerVisible, double opacity);
    void _appendLayer(const RasterOverlayLayerInfo &info, const QVector<float> &grid, int gridW, int gridH);
    void _connectLayer(RasterOverlayLayer *layer);

    /// Persist/restore the layer list so the layers come back on the next run
    void _saveLayers() const;
    void _restoreLayers();

    QDir    _cacheDir() const;
    QString _layersFile() const;

    QmlObjectListModel *_layers = nullptr;
    int                 _counter = 0;      ///< Names cache files. Only ever touched on the main thread.
    int                 _pendingLoads = 0;
    int                 _loadProgress = 0;
    bool                _restoring = false;   ///< Suppresses saving while the list is being rebuilt
};
