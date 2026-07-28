# RTCMMavlink is the shared RTCM->MAVLink forwarder. It has no serial dependency and is
# reused by the network RTCM sources (src/RTK), so it must compile even in serial-less builds.
target_sources(${CMAKE_PROJECT_NAME}
    PRIVATE
        RTCMMavlink.cc
        RTCMMavlink.h
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

if(QGC_NO_SERIAL_LINK)
    return()
endif()

target_sources(${CMAKE_PROJECT_NAME}
    PRIVATE
        GPSManager.cc
        GPSManager.h
        GPSProvider.cc
        GPSProvider.h
        GPSRtk.cc
        GPSRtk.h
        GPSRTKFactGroup.cc
        GPSRTKFactGroup.h
        satellite_info.h
        sensor_gnss_relative.h
        sensor_gps.h
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE Qt6::Core)

#===========================================================================#

CPMAddPackage(
    NAME px4-gpsdrivers
    GITHUB_REPOSITORY PX4/PX4-GPSDrivers
    GIT_TAG caf5158061bd10e79c9f042abb62c86bc6f3e7a7
    SOURCE_SUBDIR src
)

file(GLOB GPS_DRIVERS_SOURCES "${px4-gpsdrivers_SOURCE_DIR}/src/*")
target_sources(${CMAKE_PROJECT_NAME}
    PRIVATE
        definitions.h
        ${GPS_DRIVERS_SOURCES}
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE GPS_DEFINITIONS_HEADER=<${CMAKE_CURRENT_SOURCE_DIR}/definitions.h>)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${px4-gpsdrivers_SOURCE_DIR}/src)
