/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include <QtCore/QObject>
#include <QtCore/QUrl>
#include <QtCore/QVector>
#include <QtPositioning/QGeoCoordinate>

/// Everything needed to recreate a RasterOverlayLayer without re-reading the source GeoTIFF. GeoTiffHelper
/// writes this to its cache so layers come back instantly on the next run.
struct RasterOverlayLayerInfo
{
    QString         name;               ///< Display name, the source file's base name
    QString         sourceFile;         ///< The GeoTIFF this was decoded from
    qint64          sourceSize = 0;     ///< Source size/timestamp, to spot the source being changed
    qint64          sourceModified = 0;
    QUrl            imageUrl;           ///< Decoded display image in the cache
    QString         gridFile;           ///< Cached elevation grid, empty for imagery
    QGeoCoordinate  swCorner;
    QGeoCoordinate  neCorner;
    bool            elevation = false;
    double          minElevation = 0;   ///< only meaningful when elevation == true
    double          maxElevation = 0;
    bool            layerVisible = true;
    double          opacity = 0.7;
};

/// One loaded GeoTIFF map overlay (an orthomosaic or a colorized DEM). Owned by GeoTiffHelper and
/// exposed to QML so each layer can be shown/hidden and have its opacity adjusted independently.
///
/// The cached image/grid files belong to the layer's membership of GeoTiffHelper's list, not to this
/// object's lifetime: they must survive being destroyed at shutdown so the layer can be restored, and are
/// only removed when the layer is actually removed. See GeoTiffHelper::removeLayer.
class RasterOverlayLayer : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString        name             READ name             CONSTANT)
    Q_PROPERTY(QUrl           imageUrl         READ imageUrl         CONSTANT)
    Q_PROPERTY(bool           elevation        READ elevation        CONSTANT)
    Q_PROPERTY(QGeoCoordinate nwCorner         READ nwCorner         CONSTANT)
    Q_PROPERTY(QGeoCoordinate seCorner         READ seCorner         CONSTANT)
    Q_PROPERTY(QGeoCoordinate centerCoordinate READ centerCoordinate CONSTANT)
    Q_PROPERTY(bool           layerVisible     READ layerVisible     WRITE setLayerVisible NOTIFY layerVisibleChanged)
    Q_PROPERTY(double         opacity          READ opacity          WRITE setOpacity      NOTIFY opacityChanged)
    Q_PROPERTY(double         minElevation     READ minElevation     CONSTANT) ///< only meaningful when elevation == true
    Q_PROPERTY(double         maxElevation     READ maxElevation     CONSTANT)

public:
    RasterOverlayLayer(const RasterOverlayLayerInfo &info, QObject *parent = nullptr);

    /// Store the decimated elevation grid (row-major, row 0 = north; nodata = NaN) for coordinate sampling.
    void setElevationGrid(const QVector<float> &grid, int gridWidth, int gridHeight);

    /// Sample the DEM elevation at a coordinate. Returns NaN outside the raster or at nodata cells.
    Q_INVOKABLE double elevationAt(const QGeoCoordinate &coordinate) const;

    const RasterOverlayLayerInfo &info() const { return _info; }

    /// Remove this layer's cached image/grid. Only call when the layer is being removed for good.
    void deleteCacheFiles() const;

    QString        name()         const { return _info.name; }
    QUrl           imageUrl()     const { return _info.imageUrl; }
    bool           elevation()    const { return _info.elevation; }
    double         minElevation() const { return _info.minElevation; }
    double         maxElevation() const { return _info.maxElevation; }
    QGeoCoordinate nwCorner()  const { return QGeoCoordinate(_info.neCorner.latitude(), _info.swCorner.longitude()); }
    QGeoCoordinate seCorner()  const { return QGeoCoordinate(_info.swCorner.latitude(), _info.neCorner.longitude()); }
    QGeoCoordinate centerCoordinate() const;

    bool   layerVisible() const { return _info.layerVisible; }
    void   setLayerVisible(bool visible);

    double opacity() const { return _info.opacity; }
    void   setOpacity(double opacity);

signals:
    void layerVisibleChanged();
    void opacityChanged();

private:
    RasterOverlayLayerInfo _info;
    QVector<float>         _elevationGrid;   ///< row-major, row 0 = north, nodata = NaN
    int                    _gridW = 0;
    int                    _gridH = 0;
};
