target_sources(${CMAKE_PROJECT_NAME}
    PRIVATE
        GeoDemDecoder.cc
        GeoDemDecoder.h
        GeoOrthoDecoder.cc
        GeoOrthoDecoder.h
        GeoTiffHelper.cc
        GeoTiffHelper.h
        GeoTiffReader.cc
        GeoTiffReader.h
        RasterOverlayLayer.cc
        RasterOverlayLayer.h
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

#===========================================================================#

# libtiff for robust decoding of DEM/DSM GeoTIFFs (float, tiled, BigTIFF, LZW/PackBits).
# ALL external codecs are turned off (libtiff's own option names) so the build never links a
# system/MinGW codec DLL (e.g. liblzma) or a mismatched zlib. Built-in none/LZW/PackBits remain.
CPMAddPackage(
    NAME tiff
    GIT_REPOSITORY https://gitlab.com/libtiff/libtiff.git
    GIT_TAG v4.7.0
    OPTIONS
        "tiff-tools OFF"
        "tiff-tests OFF"
        "tiff-contrib OFF"
        "tiff-docs OFF"
        "tiff-deprecated OFF"
        "tiff-install OFF"
        "BUILD_SHARED_LIBS OFF"
        "lzma OFF"
        "zstd OFF"
        "webp OFF"
        "jpeg OFF"
        "old-jpeg OFF"
        "jpeg12 OFF"
        "jbig OFF"
        "lerc OFF"
        "zlib OFF"
        "libdeflate OFF"
        "pixarlog OFF"
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE tiff)
