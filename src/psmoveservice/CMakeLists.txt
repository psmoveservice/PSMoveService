cmake_minimum_required(VERSION 3.0)

set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/../..)

# Fix Error C1128 - number of sections exceeded object file format limit: compile with /bigobj
if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()

# Dependencies
set(PSMOVE_SERVICE_INCL_DIRS)
set(PSMOVE_SERVICE_REQ_LIBS)

# Platform specific libraries
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    find_library(IOKIT_FRAMEWORK IOKit)
    find_library(COREFOUNDATION_FRAMEWORK CoreFoundation)
    #find_library(QUARTZCORE QuartzCore)
    find_library(APPKIT_FRAMEWORK AppKit)
    #find_library(QTKIT QTKit)
    find_library(AVFOUNDATION AVFoundation)
    find_library(IOBLUETOOTH IOBluetooth)
    #stdc++ ${QUARTZCORE} ${APPKIT_FRAMEWORK} ${QTKIT} ${AVFOUNDATION}
    list(APPEND PSMOVE_SERVICE_REQ_LIBS
        ${COREFOUNDATION_FRAMEWORK}
        ${IOKIT_FRAMEWORK}
        ${APPKIT_FRAMEWORK}
        ${AVFOUNDATION}
        ${IOBLUETOOTH})
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    #OpenCV extra dependencies: comctl32 gdi32 ole32 setupapi ws2_32 vfw32
    #setupapi required by hidapi
    #hid required for HidD_SetOutputReport() in DualShock4 controller
    #winmm required by libstem_gamepad
    list(APPEND PSMOVE_SERVICE_REQ_LIBS bthprops setupapi hid winmm)
    IF(MINGW)
        #list(APPEND PSMOVE_SERVICE_REQ_LIBS stdc++)
    ENDIF(MINGW)
ELSE() #Linux
ENDIF()

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
    ${CMAKE_CURRENT_LIST_DIR}/Platform
    ${CMAKE_CURRENT_LIST_DIR}/PSMoveConfig
    ${CMAKE_CURRENT_LIST_DIR}/PSDualShock4
    ${CMAKE_CURRENT_LIST_DIR}/PSMoveController
    ${CMAKE_CURRENT_LIST_DIR}/PSNaviController
    ${CMAKE_CURRENT_LIST_DIR}/PSMoveTracker
    ${CMAKE_CURRENT_LIST_DIR}/PSMoveTracker/PSEye
    ${CMAKE_CURRENT_LIST_DIR}/Server
)

# Eigen math library
list(APPEND PSMOVE_SERVICE_INCL_DIRS ${ROOT_DIR}/thirdparty/eigen/)

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
find_package(Boost 1.61.0 REQUIRED QUIET COMPONENTS atomic chrono filesystem program_options system thread)
list(APPEND PSMOVE_SERVICE_INCL_DIRS ${Boost_INCLUDE_DIRS})
list(APPEND PSMOVE_SERVICE_REQ_LIBS ${Boost_LIBRARIES})
IF(MSVC) # Disable asio auto linking in date-time and regex
add_definitions(-DBOOST_DATE_TIME_NO_LIB)
add_definitions(-DBOOST_REGEX_NO_LIB)
ENDIF()

# hidapi
include_directories(${ROOT_DIR}/thirdparty/hidapi/hidapi)
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    list(APPEND PSMOVESERVICE_SRC ${ROOT_DIR}/thirdparty/hidapi/windows/hid.c)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    list(APPEND PSMOVESERVICE_SRC ${ROOT_DIR}/thirdparty/hidapi/mac/hid.c)
ELSE()
    list(APPEND PSMOVESERVICE_SRC ${ROOT_DIR}/thirdparty/hidapi/linux/hid.c)
ENDIF()

# bluetooth
list(APPEND PSMOVESERVICE_SRC
    ${CMAKE_CURRENT_LIST_DIR}/Platform/BluetoothRequests.h
    ${CMAKE_CURRENT_LIST_DIR}/Platform/BluetoothQueries.h)
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    list(APPEND PSMOVESERVICE_SRC
        ${CMAKE_CURRENT_LIST_DIR}/Platform/BluetoothRequestsWin32.cpp)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    list(APPEND PSMOVESERVICE_SRC
        ${CMAKE_CURRENT_LIST_DIR}/Platform/BluetoothRequestsOSX.mm
        ${CMAKE_CURRENT_LIST_DIR}/Platform/BluetoothQueriesOSX.mm)
ELSE()
ENDIF()

# libstem_gamepad
include_directories(${ROOT_DIR}/thirdparty/libstem_gamepad/source)
list(APPEND PSMOVESERVICE_SRC ${ROOT_DIR}/thirdparty/libstem_gamepad/source/gamepad/Gamepad_private.c)
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    list(APPEND PSMOVESERVICE_SRC ${ROOT_DIR}/thirdparty/libstem_gamepad/source/gamepad/Gamepad_windows_mm.c)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    list(APPEND PSMOVESERVICE_SRC ${ROOT_DIR}/thirdparty/libstem_gamepad/source/gamepad/Gamepad_macosx.c)
ELSE()
    list(APPEND PSMOVESERVICE_SRC ${ROOT_DIR}/thirdparty/libstem_gamepad/source/gamepad/Gamepad_linux.c)
ENDIF()

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

# OpenCV
set(OpenCV_STATIC ON)

IF(NOT(${CMAKE_SYSTEM_NAME} MATCHES "Windows"))
FIND_PACKAGE(OpenCV REQUIRED)
ENDIF()
list(APPEND PSMOVE_SERVICE_INCL_DIRS ${OpenCV_INCLUDE_DIRS})
list(APPEND PSMOVE_SERVICE_REQ_LIBS ${OpenCV_LIBS})

# PS3EYEDriver - only necessary on Mac and Win64, but can be used in Win32 (I think)
IF (${CMAKE_SYSTEM_NAME} MATCHES "Darwin"
    OR (${CMAKE_SYSTEM_NAME} MATCHES "Windows"))
    #PS3EYEDriver
    list(APPEND PSMOVE_SERVICE_INCL_DIRS ${ROOT_DIR}/thirdparty/PS3EYEDriver/src)
    list(APPEND PSMOVESERVICE_SRC
        ${ROOT_DIR}/thirdparty/PS3EYEDriver/src/ps3eye.h
        ${ROOT_DIR}/thirdparty/PS3EYEDriver/src/ps3eye.cpp)
    #Requires libusb
    find_package(USB1 REQUIRED)
    list(APPEND PSMOVE_SERVICE_INCL_DIRS ${LIBUSB_INCLUDE_DIR})
    list(APPEND PSMOVE_SERVICE_REQ_LIBS ${LIBUSB_LIBRARIES})
    add_definitions(-DHAVE_PS3EYE)
    IF (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-braced-scalar-init")
    ENDIF()
ENDIF()

# CL EYE - only on Win32
SET(ISWIN32 FALSE)
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows"
    AND NOT(${CMAKE_C_SIZEOF_DATA_PTR} EQUAL 8))
    SET(ISWIN32 TRUE)
    add_definitions(-DHAVE_CLEYE)
    list(APPEND PSMOVE_SERVICE_INCL_DIRS ${ROOT_DIR}/thirdparty/CLEYE)
    list(APPEND PSMOVE_SERVICE_REQ_LIBS ${ROOT_DIR}/thirdparty/CLEYE/x86/lib/CLEyeMulticam.lib)
    find_path(CL_EYE_SDK_PATH CLEyeMulticam.dll
        HINTS C:/Windows/SysWOW64)
    #The non-Multicam version does not require any libs/dlls/includes
    #Uses OpenCV for video. Uses the registry for settings.
    #But libusb is required for enumerating the devices and checking for the CL Eye Driver.
    find_package(USB1 REQUIRED)
    list(APPEND PSMOVE_SERVICE_INCL_DIRS ${LIBUSB_INCLUDE_DIR})
    list(APPEND PSMOVE_SERVICE_REQ_LIBS ${LIBUSB_LIBRARIES})
ENDIF()

add_executable(PSMoveService ${PSMOVESERVICE_SRC})
target_include_directories(PSMoveService PUBLIC ${PSMOVE_SERVICE_INCL_DIRS})
target_link_libraries(PSMoveService ${PSMOVE_SERVICE_REQ_LIBS})

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
add_dependencies(PSMoveService opencv)
ENDIF()

# Only set the admin privilege escalation on MSVC builds (for service operations)
IF(MSVC)
set_target_properties(PSMoveService PROPERTIES LINK_FLAGS "/level='requireAdministrator' /uiAccess='false'")
ENDIF()

# Install    
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS PSMoveService
        RUNTIME DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/bin
        LIBRARY DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/lib
        ARCHIVE DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/lib)
    IF(${ISWIN32})
        install(DIRECTORY "${ROOT_DIR}/thirdparty/CLEYE/x86/bin/"
            DESTINATION ${ROOT_DIR}/Win32/bin
            FILES_MATCHING PATTERN "*.dll")    
    ENDIF()#ISWIN32
ELSE() #Linux/Darwin
ENDIF()

IF(${ISWIN32})
    IF(${CL_EYE_SDK_PATH} STREQUAL "CL_EYE_SDK_PATH-NOTFOUND")
        #If the developer does not have CLEyeMulticam.dll on their system,
        #copy it to the correct directory to prevent crashes.
        #If we distribute binaries (e.g., a server to use alongside a UE4 plugin)
        #then we will distribute it with this DLL with the server exe.
        #It will be up to CLEYE SDK users to delete this version of the DLL
        #to use their system version.
        add_custom_command(TARGET PSMoveService POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${ROOT_DIR}/thirdparty/CLEYE/x86/bin/CLEyeMulticam.dll"
                $<TARGET_FILE_DIR:PSMoveService>)
    ENDIF()#CL_EYE not found
ENDIF()#ISWIN32
