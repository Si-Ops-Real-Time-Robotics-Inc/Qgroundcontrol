/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include <QtPositioning/QGeoCoordinate>

class QString;

/// Reads the georeferencing tags from a (classic, non-BigTIFF) GeoTIFF using only Qt --
/// no libtiff/libgeotiff/GDAL. The raster pixels themselves are decoded by Qt's image
/// plugins (qtimageformats) when the file is displayed; this reader only computes the
/// north-up WGS84 bounding box needed to place the image on the map.
///
/// Supported CRS: WGS84 / geographic lat-lon (EPSG:4326) and WGS84 UTM (EPSG:326xx / 327xx),
/// matching the existing SHPFileHelper limitation. Other CRS return an error.
namespace GeoTiffReader
{
    /// Parse the GeoTIFF model-transformation + GeoKey tags and compute the geographic corners.
    ///     @param file          Absolute path to the .tif / .tiff file
    ///     @param swCorner[out] South-west corner (min latitude, min longitude)
    ///     @param neCorner[out] North-east corner (max latitude, max longitude)
    ///     @param errorString[out] Human readable error on failure
    /// @return true on success
    bool readBounds(const QString &file, QGeoCoordinate &swCorner, QGeoCoordinate &neCorner, QString &errorString);
}
