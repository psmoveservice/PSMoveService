set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/../..)

# Common dependencies
SET(PLATFORM_LIBS)

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
    list(APPEND PLATFORM_LIBS
        ${COREFOUNDATION_FRAMEWORK}
        ${IOKIT_FRAMEWORK}
        ${APPKIT_FRAMEWORK}
        ${AVFOUNDATION}
        ${IOBLUETOOTH})
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    #OpenCV extra dependencies: comctl32 gdi32 ole32 setupapi ws2_32 vfw32
    #setupapi required by hidapi
    list(APPEND PLATFORM_LIBS setupapi)
    IF(MINGW)
        #list(APPEND PLATFORM_LIBS stdc++)
    ENDIF(MINGW)
ELSE() #Linux
ENDIF()

IF(MSVC) # Disable asio auto linking in date-time and regex
add_definitions(-DBOOST_DATE_TIME_NO_LIB)
add_definitions(-DBOOST_REGEX_NO_LIB)
ENDIF()

#
# TEST_CONFIGMANAGER
#

SET(TEST_CONFIG_SRC)
SET(TEST_CONFIG_INCL_DIRS)
SET(TEST_CONFIG_REQ_LIBS)

# Dependencies

# Boost
# TODO: Eliminate boost::filesystem with C++14
FIND_PACKAGE(Boost REQUIRED QUIET COMPONENTS filesystem system)
list(APPEND TEST_CONFIG_INCL_DIRS ${Boost_INCLUDE_DIRS})
list(APPEND TEST_CONFIG_REQ_LIBS ${Boost_LIBRARIES})

# Our custom ConfigManager classes
# We are not including the PSMoveService project on purpose.
list(APPEND TEST_CONFIG_INCL_DIRS 
    ${ROOT_DIR}/src/psmoveservice/PSMoveConfig
    ${ROOT_DIR}/src/psmoveservice/Device/Interface
    ${ROOT_DIR}/src/psmoveservice/Server)
list(APPEND TEST_CONFIG_SRC
    ${ROOT_DIR}/src/psmoveservice/PSMoveConfig/PSMoveConfig.h
    ${ROOT_DIR}/src/psmoveservice/PSMoveConfig/PSMoveConfig.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/Interface/DeviceInterface.h
    ${ROOT_DIR}/src/psmoveservice/Server/ServerUtility.h
    ${ROOT_DIR}/src/psmoveservice/Server/ServerUtility.cpp)

add_executable(test_config ${CMAKE_CURRENT_LIST_DIR}/test_config.cpp ${TEST_CONFIG_SRC})
target_include_directories(test_config PUBLIC ${TEST_CONFIG_INCL_DIRS})
target_link_libraries(test_config ${PLATFORM_LIBS} ${TEST_CONFIG_REQ_LIBS})
SET_TARGET_PROPERTIES(test_config PROPERTIES FOLDER Test)

# Install    
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS test_config
        RUNTIME DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/bin
        LIBRARY DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/lib
        ARCHIVE DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/lib)
ELSE() #Linux/Darwin
ENDIF()

#
# TEST_CAMERA
#

SET(TEST_CAMERA_SRC)
SET(TEST_CAMERA_INCL_DIRS)
SET(TEST_CAMERA_REQ_LIBS)

# OpenCV
set(OpenCV_STATIC ON)
FIND_PACKAGE(OpenCV REQUIRED)
IF ("${OpenCV_LIBS}" STREQUAL "")
    message("OpenCV Libs was empty! Manually setting.")
    list(APPEND OpenCV_LIBS opencv_core opencv_imgproc opencv_calib3d opencv_video opencv_features2d opencv_ml opencv_highgui opencv_objdetect)
ENDIF()
IF ("${OpenCV_INCLUDE_DIRS}" STREQUAL "" AND ${OpenCV_DIR})
    message("OpenCV include dirs was empty! Manually setting.")
    set (OpenCV_INCLUDE_DIRS ${OpenCV_DIR}\include)
ENDIF()
list(APPEND TEST_CAMERA_INCL_DIRS ${OpenCV_INCLUDE_DIRS})
list(APPEND TEST_CAMERA_REQ_LIBS ${OpenCV_LIBS})

# PS3EYEDriver - only necessary on Mac and Win64, but can be used in Win32 (I think)
IF (${CMAKE_SYSTEM_NAME} MATCHES "Darwin"
    OR (${CMAKE_SYSTEM_NAME} MATCHES "Windows"))
    #PS3EYEDriver
    list(APPEND TEST_CAMERA_INCL_DIRS ${ROOT_DIR}/thirdparty/PS3EYEDriver/src)
    list(APPEND TEST_CAMERA_SRC
        ${ROOT_DIR}/thirdparty/PS3EYEDriver/src/ps3eye.h
        ${ROOT_DIR}/thirdparty/PS3EYEDriver/src/ps3eye.cpp)
    #Requires libusb
    find_package(USB1 REQUIRED)
    list(APPEND TEST_CAMERA_INCL_DIRS ${LIBUSB_INCLUDE_DIR})
    list(APPEND TEST_CAMERA_REQ_LIBS ${LIBUSB_LIBRARIES})
    add_definitions(-DHAVE_PS3EYE)
ENDIF()

# CL EYE - only on Win32
SET(ISWIN32 FALSE)
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows"
    AND NOT(${CMAKE_C_SIZEOF_DATA_PTR} EQUAL 8))
    SET(ISWIN32 TRUE)
    add_definitions(-DHAVE_CLEYE)
    list(APPEND TEST_CAMERA_INCL_DIRS ${ROOT_DIR}/thirdparty/CLEYE)
    list(APPEND TEST_CAMERA_REQ_LIBS ${ROOT_DIR}/thirdparty/CLEYE/x86/lib/CLEyeMulticam.lib)
    find_path(CL_EYE_SDK_PATH CLEyeMulticam.dll
        HINTS C:/Windows/SysWOW64)
    #The non-Multicam version does not require any libs/dlls/includes
    #Uses OpenCV for video. Uses the registry for settings (maybe OpenCV for settings?)
    #But libusb is required for enumerating the devices and checking for the CL Eye Driver.
    find_package(USB1 REQUIRED)
    list(APPEND TEST_CAMERA_INCL_DIRS ${LIBUSB_INCLUDE_DIR})
    list(APPEND TEST_CAMERA_REQ_LIBS ${LIBUSB_LIBRARIES})

    # Windows utilities for querying driver infomation (provider name)
    list(APPEND TEST_CAMERA_INCL_DIRS ${ROOT_DIR}/src/psmoveservice/Platform)
    list(APPEND TEST_CAMERA_SRC ${ROOT_DIR}/src/psmoveservice/Platform/USBDeviceInterfaceWin32.cpp)
ENDIF()

# Our custom OpenCV VideoCapture classes
# We are not including the PSMoveService project on purpose.
list(APPEND TEST_CAMERA_INCL_DIRS 
    ${ROOT_DIR}/src/psmoveclient/
    ${ROOT_DIR}/src/psmoveservice/PSMoveTracker/PSEye)
list(APPEND TEST_CAMERA_SRC
    ${ROOT_DIR}/src/psmoveclient/ClientConstants.h
    ${ROOT_DIR}/src/psmoveservice/PSMoveTracker/PSEye/PSEyeVideoCapture.h
    ${ROOT_DIR}/src/psmoveservice/PSMoveTracker/PSEye/PSEyeVideoCapture.cpp)

# The test_camera app
add_executable(test_camera ${CMAKE_CURRENT_LIST_DIR}/test_camera.cpp ${TEST_CAMERA_SRC})
target_include_directories(test_camera PUBLIC ${TEST_CAMERA_INCL_DIRS})
target_link_libraries(test_camera ${PLATFORM_LIBS} ${TEST_CAMERA_REQ_LIBS})
SET_TARGET_PROPERTIES(test_camera PROPERTIES FOLDER Test)

IF(${ISWIN32})
    IF(${CL_EYE_SDK_PATH} STREQUAL "CL_EYE_SDK_PATH-NOTFOUND")
        #If the developer does not have CLEyeMulticam.dll on their system,
        #copy it to the correct directory to prevent crashes.
        #If we distribute binaries (e.g., a server to use alongside a UE4 plugin)
        #then we will distribute it with this DLL with the server exe.
        #It will be up to CLEYE SDK users to delete this version of the DLL
        #to use their system version.
        add_custom_command(TARGET test_camera POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${ROOT_DIR}/thirdparty/CLEYE/x86/bin/CLEyeMulticam.dll"
                $<TARGET_FILE_DIR:test_camera>)
    ENDIF()#CL_EYE not found
ENDIF()#ISWIN32

# Install    
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS test_camera
        RUNTIME DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/bin
        LIBRARY DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/lib
        ARCHIVE DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/lib)
ELSE() #Linux/Darwin
ENDIF()

#
# Test Controller
#

SET(TEST_CTRLR_SRC)
SET(TEST_CTRLR_INCL_DIRS)
SET(TEST_CTRLR_REQ_LIBS)

# Dependencies

# hidapi
include_directories(${ROOT_DIR}/thirdparty/hidapi/hidapi)
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(HIDAPI_SRC ${ROOT_DIR}/thirdparty/hidapi/windows/hid.c)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(HIDAPI_SRC ${ROOT_DIR}/thirdparty/hidapi/mac/hid.c)      
ELSE()
    set(HIDAPI_SRC ${ROOT_DIR}/thirdparty/hidapi/linux/hid.c)
ENDIF()
list(APPEND TEST_CTRLR_SRC ${HIDAPI_SRC})

#Bluetooth
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    list(APPEND TEST_CTRLR_SRC ${ROOT_DIR}/src/psmoveservice/Platform/BluetoothQueriesOSX.mm)
ELSE()
ENDIF()

# Boost
# TODO: Eliminate boost::filesystem with C++14
FIND_PACKAGE(Boost REQUIRED QUIET COMPONENTS chrono filesystem program_options system thread)
list(APPEND TEST_CTRLR_INCL_DIRS ${Boost_INCLUDE_DIRS})
list(APPEND TEST_CTRLR_REQ_LIBS ${Boost_LIBRARIES})

# Eigen math library
list(APPEND TEST_CTRLR_INCL_DIRS ${ROOT_DIR}/thirdparty/eigen/)

# PSMoveController
# We are not including the PSMoveService target on purpose, because this only tests
# a small part of the service and should not depend on the whole thing building.
list(APPEND TEST_CTRLR_INCL_DIRS
    ${ROOT_DIR}/src/psmovemath/
    ${ROOT_DIR}/src/psmoveservice/
    ${ROOT_DIR}/src/psmoveservice/Server
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator
    ${ROOT_DIR}/src/psmoveservice/Device/Interface
    ${ROOT_DIR}/src/psmoveservice/Platform
    ${ROOT_DIR}/src/psmoveservice/PSMoveConfig
    ${ROOT_DIR}/src/psmoveservice/PSMoveController)
list(APPEND TEST_CTRLR_SRC    
    ${ROOT_DIR}/src/psmovemath/MathAlignment.h
    ${ROOT_DIR}/src/psmoveservice/Server/ServerLog.h
    ${ROOT_DIR}/src/psmoveservice/Server/ServerLog.cpp
    ${ROOT_DIR}/src/psmoveservice/Server/ServerUtility.h
    ${ROOT_DIR}/src/psmoveservice/Server/ServerUtility.cpp
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/ControllerDeviceEnumerator.h    
    ${ROOT_DIR}/src/psmoveservice/Device/Enumerator/ControllerDeviceEnumerator.cpp
    ${ROOT_DIR}/src/psmoveservice/Platform/BluetoothQueries.h
    ${ROOT_DIR}/src/psmoveservice/PSMoveConfig/PSMoveConfig.h
    ${ROOT_DIR}/src/psmoveservice/PSMoveConfig/PSMoveConfig.cpp
    ${ROOT_DIR}/src/psmoveservice/PSMoveController/PSMoveController.h
    ${ROOT_DIR}/src/psmoveservice/PSMoveController/PSMoveController.cpp)

# psmoveprotocol
list(APPEND TEST_CTRLR_INCL_DIRS ${ROOT_DIR}/src/psmoveprotocol)
list(APPEND TEST_CTRLR_REQ_LIBS PSMoveProtocol)

add_executable(test_controller ${CMAKE_CURRENT_LIST_DIR}/test_controller.cpp ${TEST_CTRLR_SRC})
target_include_directories(test_controller PUBLIC ${TEST_CTRLR_INCL_DIRS})
target_link_libraries(test_controller ${PLATFORM_LIBS} ${TEST_CTRLR_REQ_LIBS})
SET_TARGET_PROPERTIES(test_controller PROPERTIES FOLDER Test)

# Install    
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS test_controller
        RUNTIME DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/bin
        LIBRARY DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/lib
        ARCHIVE DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/lib)
ELSE() #Linux/Darwin
ENDIF()

#
# TEST_CONSOLE_CLIENT
#
add_executable(test_console_client test_console_client.cpp)
target_include_directories(test_console_client PUBLIC ${ROOT_DIR}/src/psmoveclient/)
target_link_libraries(test_console_client PSMoveClient)
SET_TARGET_PROPERTIES(test_console_client PROPERTIES FOLDER Test)

# Install    
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS test_console_client
        RUNTIME DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/bin
        LIBRARY DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/lib
        ARCHIVE DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/lib)
ELSE() #Linux/Darwin
ENDIF()

#
# TEST_UDP_SERVER
#

SET(TEST_UDP_SERVER_INCL_DIRS)
SET(TEST_UDP_SERVER_REQ_LIBS)

# Dependencies

# Boost
find_package(Boost 1.59.0 REQUIRED QUIET COMPONENTS atomic chrono filesystem program_options system thread)
list(APPEND TEST_UDP_SERVER_INCL_DIRS ${Boost_INCLUDE_DIRS})
list(APPEND TEST_UDP_SERVER_REQ_LIBS ${Boost_LIBRARIES})

# Boost.Application and type_index are header only (?)
list(APPEND TEST_UDP_SERVER_INCL_DIRS
    ${ROOT_DIR}/thirdparty/Boost.Application/include/
    ${ROOT_DIR}/thirdparty/Boost.Application/example/
    ${ROOT_DIR}/thirdparty/type_index/include/)

add_executable(test_udp_server ${CMAKE_CURRENT_LIST_DIR}/test_udp_server.cpp)
target_include_directories(test_udp_server PUBLIC ${TEST_UDP_SERVER_INCL_DIRS})
target_link_libraries(test_udp_server ${PLATFORM_LIBS} ${TEST_UDP_SERVER_REQ_LIBS})
SET_TARGET_PROPERTIES(test_udp_server PROPERTIES FOLDER Test)

# Only set the admin privilege escalation on MSVC builds (for service operations)
IF(MSVC)
set_target_properties(test_udp_server PROPERTIES LINK_FLAGS "/level='requireAdministrator' /uiAccess='false'")
ENDIF()

# Install    
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
install(TARGETS test_udp_server
    RUNTIME DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/bin
    LIBRARY DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/lib
    ARCHIVE DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/lib)
ELSE() #Linux/Darwin
ENDIF()

#
# TEST_UDP_CLIENT
#

SET(TEST_UDP_CLIENT_INCL_DIRS)
SET(TEST_UDP_CLIENT_REQ_LIBS)

# Dependencies

# Boost
find_package(Boost 1.59.0 REQUIRED QUIET COMPONENTS system)
list(APPEND TEST_UDP_CLIENT_INCL_DIRS ${Boost_INCLUDE_DIRS})
list(APPEND TEST_UDP_CLIENT_REQ_LIBS ${Boost_LIBRARIES})

add_executable(test_udp_client ${CMAKE_CURRENT_LIST_DIR}/test_udp_client.cpp)
target_include_directories(test_udp_client PUBLIC ${TEST_UDP_CLIENT_INCL_DIRS})
target_link_libraries(test_udp_client ${PLATFORM_LIBS} ${TEST_UDP_CLIENT_REQ_LIBS})
SET_TARGET_PROPERTIES(test_udp_client PROPERTIES FOLDER Test)

# Install    
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
install(TARGETS test_udp_client
    RUNTIME DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/bin
    LIBRARY DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/lib
    ARCHIVE DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/lib)
ELSE() #Linux/Darwin
ENDIF()

#
# TEST_OPTICAL_TRACKER
#

add_executable(test_optical_tracker ${CMAKE_CURRENT_LIST_DIR}/test_optical_tracker.cpp ${TEST_CAMERA_SRC} ${TEST_CTRLR_SRC})
target_include_directories(test_optical_tracker PUBLIC ${TEST_CAMERA_INCL_DIRS} ${TEST_CTRLR_INCL_DIRS})
target_link_libraries(test_optical_tracker ${PLATFORM_LIBS} ${TEST_CAMERA_REQ_LIBS} ${TEST_CTRLR_REQ_LIBS})
SET_TARGET_PROPERTIES(test_optical_tracker PROPERTIES FOLDER Test)

# Install
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
install(TARGETS test_optical_tracker
RUNTIME DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/bin
LIBRARY DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/lib
ARCHIVE DESTINATION ${ROOT_DIR}/${ARCH_LABEL}/lib)
ELSE() #Linux/Darwin
ENDIF()