cmake_minimum_required(VERSION 3.0)

# Dependencies
set(PSMOVE_SERVICE_INCL_DIRS)
set(PSMOVE_SERVICE_REQ_LIBS)

list(APPEND PSMOVE_SERVICE_REQ_LIBS ${PLATFORM_LIBS})

# Source files for PSMoveService
file(GLOB PSMOVESERVICE_CONFIG_SRC
    "${CMAKE_CURRENT_LIST_DIR}/PSMoveConfig/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/PSMoveConfig/*.h"
)
source_group("Config" FILES ${PSMOVESERVICE_CONFIG_SRC})

file(GLOB PSMOVESERVICE_CONTROLLER_SRC
    "${CMAKE_CURRENT_LIST_DIR}/PSDualShock4/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/PSDualShock4/*.h"
    "${CMAKE_CURRENT_LIST_DIR}/PSMoveController/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/PSMoveController/*.h"
    "${CMAKE_CURRENT_LIST_DIR}/PSNaviController/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/PSNaviController/*.h"
    "${CMAKE_CURRENT_LIST_DIR}/VirtualController/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/VirtualController/*.h"
)
source_group("Controller" FILES ${PSMOVESERVICE_CONTROLLER_SRC})

file(GLOB PSMOVESERVICE_DEVICE_ENUM_SRC
    "${CMAKE_CURRENT_LIST_DIR}/Device/Enumerator/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Device/Enumerator/*.h"
)
source_group("Device\\Enumerator" FILES ${PSMOVESERVICE_DEVICE_ENUM_SRC})

file(GLOB PSMOVESERVICE_DEVICE_INT_SRC
    "${CMAKE_CURRENT_LIST_DIR}/Device/Interface/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Device/Interface/*.h"
)
source_group("Device\\Interface" FILES ${PSMOVESERVICE_DEVICE_INT_SRC})

file(GLOB PSMOVESERVICE_DEVICE_MGR_SRC
    "${CMAKE_CURRENT_LIST_DIR}/Device/Manager/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Device/Manager/*.h"
)
source_group("Device\\Manager" FILES ${PSMOVESERVICE_DEVICE_MGR_SRC})

file(GLOB PSMOVESERVICE_DEVICE_USB_SRC
    "${CMAKE_CURRENT_LIST_DIR}/Device/USB/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Device/USB/*.h"
)
source_group("Device\\USB" FILES ${PSMOVESERVICE_DEVICE_USB_SRC})

file(GLOB PSMOVESERVICE_DEVICE_VIEW_SRC
    "${CMAKE_CURRENT_LIST_DIR}/Device/View/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Device/View/*.h"
)
source_group("Device\\View" FILES ${PSMOVESERVICE_DEVICE_VIEW_SRC})

file(GLOB PSMOVESERVICE_HMD_SRC
    "${CMAKE_CURRENT_LIST_DIR}/MorpheusHMD/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/MorpheusHMD/*.h"
    "${CMAKE_CURRENT_LIST_DIR}/VirtualHMD/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/VirtualHMD/*.h"
)
source_group("HMD" FILES ${PSMOVESERVICE_HMD_SRC})

file(GLOB PSMOVESERVICE_FILTER_SRC
    "${CMAKE_CURRENT_LIST_DIR}/Filter/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Filter/*.h"
)
source_group("Filter" FILES ${PSMOVESERVICE_FILTER_SRC})

file(GLOB PSMOVESERVICE_SERVER_SRC
    "${CMAKE_CURRENT_LIST_DIR}/Server/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Server/*.h"
)
source_group("Server" FILES ${PSMOVESERVICE_SERVER_SRC})

file(GLOB PSMOVESERVICE_TRACKER_SRC
    "${CMAKE_CURRENT_LIST_DIR}/PSMoveTracker/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/PSMoveTracker/*.h"
    "${CMAKE_CURRENT_LIST_DIR}/PSMoveTracker/PSEye/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/PSMoveTracker/PSEye/*.h"
)
source_group("Tracker" FILES ${PSMOVESERVICE_TRACKER_SRC})

set(PSMOVESERVICE_SRC
    ${PSMOVESERVICE_CONFIG_SRC}
    ${PSMOVESERVICE_CONTROLLER_SRC}
    ${PSMOVESERVICE_DEVICE_ENUM_SRC}
    ${PSMOVESERVICE_DEVICE_INT_SRC}
    ${PSMOVESERVICE_DEVICE_MGR_SRC}
    ${PSMOVESERVICE_DEVICE_USB_SRC}
    ${PSMOVESERVICE_DEVICE_VIEW_SRC}
    ${PSMOVESERVICE_HMD_SRC}
    ${PSMOVESERVICE_FILTER_SRC}
    ${PSMOVESERVICE_SERVER_SRC} 
    ${PSMOVESERVICE_TRACKER_SRC}
)

list(APPEND PSMOVE_SERVICE_INCL_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/Device/Enumerator
    ${CMAKE_CURRENT_LIST_DIR}/Device/Interface
    ${CMAKE_CURRENT_LIST_DIR}/Device/Manager
    ${CMAKE_CURRENT_LIST_DIR}/Device/USB
    ${CMAKE_CURRENT_LIST_DIR}/Device/View
    ${CMAKE_CURRENT_LIST_DIR}/Filter
    ${CMAKE_CURRENT_LIST_DIR}/MorpheusHMD
    ${CMAKE_CURRENT_LIST_DIR}/VirtualHMD
    ${CMAKE_CURRENT_LIST_DIR}/Platform
    ${CMAKE_CURRENT_LIST_DIR}/PSMoveConfig
    ${CMAKE_CURRENT_LIST_DIR}/PSDualShock4
    ${CMAKE_CURRENT_LIST_DIR}/PSMoveController
    ${CMAKE_CURRENT_LIST_DIR}/PSNaviController
    ${CMAKE_CURRENT_LIST_DIR}/PSMoveTracker
    ${CMAKE_CURRENT_LIST_DIR}/PSMoveTracker/PSEye
    ${CMAKE_CURRENT_LIST_DIR}/Server
    ${CMAKE_CURRENT_LIST_DIR}/VirtualController
)

# Eigen math library
list(APPEND PSMOVE_SERVICE_INCL_DIRS ${EIGEN3_INCLUDE_DIR})

# mherb/Kalman library
list(APPEND PSMOVE_SERVICE_INCL_DIRS ${ROOT_DIR}/thirdparty/kalman/include)

# Boost.Application and type_index are header only (?)
list(APPEND PSMOVE_SERVICE_INCL_DIRS
    ${ROOT_DIR}/thirdparty/Boost.Application/include/
    ${ROOT_DIR}/thirdparty/Boost.Application/example/
    ${ROOT_DIR}/thirdparty/type_index/include/)

# Protobuf (already found in top-level CMakeLists)
list(APPEND PSMOVE_SERVICE_INCL_DIRS ${PROTOBUF_INCLUDE_DIRS})
list(APPEND PSMOVE_SERVICE_REQ_LIBS ${PROTOBUF_LIBRARIES})

# Boost. TODO: Trim this list.
find_package(Boost REQUIRED QUIET COMPONENTS atomic chrono filesystem program_options system thread)
list(APPEND PSMOVE_SERVICE_INCL_DIRS ${Boost_INCLUDE_DIRS})
list(APPEND PSMOVE_SERVICE_REQ_LIBS ${Boost_LIBRARIES})

# hidapi
list(APPEND PSMOVE_SERVICE_INCL_DIRS ${HIDAPI_INCLUDE_DIRS})
list(APPEND PSMOVESERVICE_SRC ${HIDAPI_SRC})
list(APPEND PSMOVE_SERVICE_REQ_LIBS ${HIDAPI_LIBS})

# bluetooth
list(APPEND PSMOVESERVICE_SRC
    ${CMAKE_CURRENT_LIST_DIR}/Platform/BluetoothRequests.h
    ${CMAKE_CURRENT_LIST_DIR}/Platform/BluetoothQueries.h)
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    list(APPEND PSMOVESERVICE_SRC
        ${CMAKE_CURRENT_LIST_DIR}/Platform/BluetoothRequestsWin32.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Platform/BluetoothQueriesWin32.cpp)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    list(APPEND PSMOVESERVICE_SRC
        ${CMAKE_CURRENT_LIST_DIR}/Platform/BluetoothRequestsOSX.mm
        ${CMAKE_CURRENT_LIST_DIR}/Platform/BluetoothQueriesOSX.mm)
ELSE()
    list(APPEND PSMOVESERVICE_SRC
        ${CMAKE_CURRENT_LIST_DIR}/Platform/BluetoothRequestsLinux.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Platform/BluetoothQueriesLinux.cpp)
ENDIF()

# LibUSB for device management
find_package(USB1 REQUIRED)
list(APPEND PSMOVE_SERVICE_INCL_DIRS ${LIBUSB_INCLUDE_DIR})
list(APPEND PSMOVE_SERVICE_REQ_LIBS ${LIBUSB_LIBRARIES})

# libstem_gamepad
list(APPEND PSMOVE_SERVICE_INCL_DIRS ${LIBSTEM_GAMEPAD_INCLUDE_DIRS})
list(APPEND PSMOVESERVICE_SRC ${LIBSTEM_GAMEPAD_SRC})

# Platform Specific Device Management
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    # Windows utilities for querying driver infomation (provider name)
    list(APPEND PSMOVE_SERVICE_INCL_DIRS ${ROOT_DIR}/src/psmoveservice/Platform)
    list(APPEND PSMOVESERVICE_SRC
        ${CMAKE_CURRENT_LIST_DIR}/Platform/PlatformDeviceAPIWin32.h
        ${CMAKE_CURRENT_LIST_DIR}/Platform/PlatformDeviceAPIWin32.cpp)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
ELSE()
ENDIF()

# PSMoveDataFrame
list(APPEND PSMOVE_SERVICE_INCL_DIRS ${ROOT_DIR}/src/psmoveprotocol/)
list(APPEND PSMOVE_SERVICE_REQ_LIBS PSMoveProtocol)

# PSMoveMath
list(APPEND PSMOVE_SERVICE_INCL_DIRS ${ROOT_DIR}/src/psmovemath/)
list(APPEND PSMOVE_SERVICE_REQ_LIBS PSMoveMath)

# Tracker
# Requires OpenCV, PS3EYEDriver (Mac/Win64), CLEye (Win32)

# OpenCV - empty on Windows
IF(MSVC) # not necessary for OpenCV > 2.8 on other build systems
    list(APPEND PSMOVE_SERVICE_INCL_DIRS ${OpenCV_INCLUDE_DIRS}) 
ENDIF()
list(APPEND PSMOVE_SERVICE_REQ_LIBS ${OpenCV_LIBS})

# PS Eye - This brings in LIBUSB on Windows and Mac, but not Linux
list(APPEND PSMOVESERVICE_SRC ${PSEYE_SRC})
list(APPEND PSMOVE_SERVICE_INCL_DIRS ${PSEYE_INCLUDE_DIRS})
list(APPEND PSMOVE_SERVICE_REQ_LIBS ${PSEYE_LIBRARIES})

add_executable(PSMoveService ${PSMOVESERVICE_SRC})
target_include_directories(PSMoveService PUBLIC ${PSMOVE_SERVICE_INCL_DIRS})
target_link_libraries(PSMoveService ${PSMOVE_SERVICE_REQ_LIBS})

IF(MSVC)
	# set the UAC level in the property sheet
    set_target_properties(PSMoveService PROPERTIES LINK_FLAGS "/level='requireAdministrator' /uiAccess='false'")
ENDIF()

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    add_dependencies(PSMoveService opencv)
ENDIF()

# Install
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS PSMoveService
        CONFIGURATIONS Debug
        RUNTIME DESTINATION ${PSM_DEBUG_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib)
    install(TARGETS PSMoveService
        CONFIGURATIONS Release
        RUNTIME DESTINATION ${PSM_RELEASE_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib)        
    IF(${ISWIN32})
        install(DIRECTORY "${ROOT_DIR}/thirdparty/CLEYE/x86/bin/"
            CONFIGURATIONS Debug
            DESTINATION ${PSM_DEBUG_INSTALL_PATH}/bin
            FILES_MATCHING PATTERN "*.dll")
        install(DIRECTORY "${ROOT_DIR}/thirdparty/CLEYE/x86/bin/"
            CONFIGURATIONS Release
            DESTINATION ${PSM_RELEASE_INSTALL_PATH}/bin
            FILES_MATCHING PATTERN "*.dll")            
    ENDIF()#ISWIN32
ELSE() #Linux/Darwin
ENDIF()

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    IF(NOT(${CMAKE_C_SIZEOF_DATA_PTR} EQUAL 8))
        IF(${CL_EYE_SDK_PATH} STREQUAL "CL_EYE_SDK_PATH-NOTFOUND")
            #If the developer does not have CLEyeMulticam.dll on their system,
            #copy it to the correct directory to prevent crashes.
            #Windows service binaries should be distributed with this DLL.
            #It will be up to CLEYE SDK users to delete this version of the DLL
            #to use their system version.
            add_custom_command(TARGET PSMoveService POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${ROOT_DIR}/thirdparty/CLEYE/x86/bin/CLEyeMulticam.dll"
                    $<TARGET_FILE_DIR:PSMoveService>)
        ENDIF()
    ENDIF()
ENDIF()#ISWIN32 and CL_EYE_SDK_PATH-NOTFOUND
