target_sources(${CMAKE_PROJECT_NAME}
    PRIVATE
        NetworkRTCMFactGroup.cc
        NetworkRTCMFactGroup.h
        NtripClient.cc
        NtripClient.h
        NtripSourceTable.cc
        NtripSourceTable.h
        RTCMFileLogger.cc
        RTCMFileLogger.h
        RTCMNetworkSource.h
        RTCMStreamManager.cc
        RTCMStreamManager.h
        RtcmStreamParser.cc
        RtcmStreamParser.h
        UdpRtcmReceiver.cc
        UdpRtcmReceiver.h
)

target_link_libraries(${CMAKE_PROJECT_NAME}
    PRIVATE
        Qt6::Core
        Qt6::Network
        Qt6::Positioning
)

# QGC_ENABLE_BLUETOOTH is defined on the project target by src/Comms/CMakeLists.txt.
if(QGC_ENABLE_BLUETOOTH)
    target_sources(${CMAKE_PROJECT_NAME}
        PRIVATE
            BluetoothRtcmSource.cc
            BluetoothRtcmSource.h
    )
    target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE Qt6::Bluetooth)
endif()

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# RTCMMavlink.h lives in src/GPS and is compiled unconditionally by that module.
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../GPS)
