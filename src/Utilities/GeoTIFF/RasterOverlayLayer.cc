/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "RasterOverlayLayer.h"

#include <QtCore/QFile>

#include <cmath>
#include <limits>

RasterOverlayLayer::RasterOverlayLayer(const RasterOverlayLayerInfo &info, QObject *parent)
    : QObject(parent)
    , _info(info)
{
}

void RasterOverlayLayer::deleteCacheFiles() const
{
    const QString imageFile = _info.imageUrl.toLocalFile();
    if (!imageFile.isEmpty()) {
        QFile::remove(imageFile);
    }
    if (!_info.gridFile.isEmpty()) {
        QFile::remove(_info.gridFile);
    }
}

QGeoCoordinate RasterOverlayLayer::centerCoordinate() const
{
    return QGeoCoordinate((_info.swCorner.latitude()  + _info.neCorner.latitude())  / 2.0,
                          (_info.swCorner.longitude() + _info.neCorner.longitude()) / 2.0);
}

void RasterOverlayLayer::setLayerVisible(bool visible)
{
    if (visible != _info.layerVisible) {
        _info.layerVisible = visible;
        emit layerVisibleChanged();
    }
}

void RasterOverlayLayer::setOpacity(double opacity)
{
    if (!qFuzzyCompare(opacity, _info.opacity)) {
        _info.opacity = opacity;
        emit opacityChanged();
    }
}

void RasterOverlayLayer::setElevationGrid(const QVector<float> &grid, int gridWidth, int gridHeight)
{
    _elevationGrid = grid;
    _gridW = gridWidth;
    _gridH = gridHeight;
}

double RasterOverlayLayer::elevationAt(const QGeoCoordinate &coordinate) const
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    if (_elevationGrid.isEmpty() || (_gridW <= 0) || (_gridH <= 0)) {
        return nan;
    }

    const double lat = coordinate.latitude();
    const double lon = coordinate.longitude();
    const double swLat = _info.swCorner.latitude();
    const double swLon = _info.swCorner.longitude();
    const double neLat = _info.neCorner.latitude();
    const double neLon = _info.neCorner.longitude();

    if ((lon < swLon) || (lon > neLon) || (lat < swLat) || (lat > neLat)) {
        return nan;
    }

    const double cellLon = (neLon - swLon) / _gridW;
    const double cellLat = (neLat - swLat) / _gridH;
    if ((cellLon <= 0.0) || (cellLat <= 0.0)) {
        return nan;
    }

    int col = int((lon - swLon) / cellLon);
    int row = int((neLat - lat) / cellLat);   // row 0 = north (top)
    col = qBound(0, col, _gridW - 1);
    row = qBound(0, row, _gridH - 1);

    const float v = _elevationGrid[(row * _gridW) + col];
    if (std::isnan(v)) {
        return nan;
    }
    return double(v);
}
