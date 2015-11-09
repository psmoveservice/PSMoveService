cmake_minimum_required(VERSION 3.0)

set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/..)

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
    #stdc++ ${QUARTZCORE} ${APPKIT_FRAMEWORK} ${QTKIT} ${AVFOUNDATION}
    list(APPEND PLATFORM_LIBS
        ${COREFOUNDATION_FRAMEWORK}
        ${IOKIT_FRAMEWORK}
        ${APPKIT_FRAMEWORK}
        ${AVFOUNDATION})
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    #OpenCV extra dependencies: comctl32 gdi32 ole32 setupapi ws2_32 vfw32
    #setupapi required by hidapi
    list(APPEND PLATFORM_LIBS setupapi)
    IF(MINGW)
        #list(APPEND PLATFORM_LIBS stdc++)
    ENDIF(MINGW)
ELSE() #Linux
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
list(APPEND TEST_CONFIG_INCL_DIRS ${ROOT_DIR}/psmoveservice/PSMoveConfig)
list(APPEND TEST_CONFIG_SRC
    ${ROOT_DIR}/psmoveservice/PSMoveConfig/PSMoveConfig.h
    ${ROOT_DIR}/psmoveservice/PSMoveConfig/PSMoveConfig.cpp)

add_executable(test_config ${CMAKE_CURRENT_LIST_DIR}/test_config.cpp ${TEST_CONFIG_SRC})
target_include_directories(test_config PUBLIC ${TEST_CONFIG_INCL_DIRS})
target_link_libraries(test_config ${PLATFORM_LIBS} ${TEST_CONFIG_REQ_LIBS})


#
# TEST_CAMERA
#

SET(TEST_CAMERA_SRC)
SET(TEST_CAMERA_INCL_DIRS)
SET(TEST_CAMERA_REQ_LIBS)

# OpenCV
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(OpenCV_DIR "C:/OpenCV-3.0.0/build")# CACHE PATH "Path to OpenCV" FORCE)
ENDIF()
set(OpenCV_STATIC ON)
FIND_PACKAGE(OpenCV REQUIRED)
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
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows"
    AND NOT(${CMAKE_C_SIZEOF_DATA_PTR} EQUAL 8))
    #include_directories(${ROOT_DIR}/external/CLEYE)
    #find_path(CL_EYE_SDK_PATH bin/CLEyeMulticam.dll
    #    HINTS ${ROOT_DIR}/thirdparty/CLEYE/)
    #IF(CL_EYE_SDK_PATH)
    #    list(APPEND TEST_CAMERA_REQ_LIBS CLEyeMulticam)
    #    file(COPY ${CL_EYE_SDK_PATH}/bin/CLEyeMulticam.dll
    #        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
    #    # XXX: If this crashes, disable compiler optimizations
    #ENDIF()

    #TODO: The non multicam version. Maybe it does not require any includes/libs?
    add_definitions(-DHAVE_CLEYE)
ENDIF()

# Our custom OpenCV VideoCapture classes
# We are not including the PSMoveService project on purpose.
list(APPEND TEST_CAMERA_INCL_DIRS ${ROOT_DIR}/psmoveservice/PSEye)
list(APPEND TEST_CAMERA_SRC
    ${ROOT_DIR}/psmoveservice/PSEye/PSEyeVideoCapture.h
    ${ROOT_DIR}/psmoveservice/PSEye/PSEyeVideoCapture.cpp)

# The test_camera app
add_executable(test_camera ${CMAKE_CURRENT_LIST_DIR}/test_camera.cpp ${TEST_CAMERA_SRC})
target_include_directories(test_camera PUBLIC ${TEST_CAMERA_INCL_DIRS})
target_link_libraries(test_camera ${PLATFORM_LIBS} ${TEST_CAMERA_REQ_LIBS})


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

# PSMoveController
# We are not including the PSMoveService target on purpose, because this only tests
# a small part of the service and should not depend on the whole thing building.
list(APPEND TEST_CTRLR_INCL_DIRS
    ${ROOT_DIR}/psmoveservice/PSMoveConfig
    ${ROOT_DIR}/psmoveservice/PSMoveController)
list(APPEND TEST_CTRLR_SRC
    ${ROOT_DIR}/psmoveservice/PSMoveConfig/PSMoveConfig.h
    ${ROOT_DIR}/psmoveservice/PSMoveConfig/PSMoveConfig.cpp
    ${ROOT_DIR}/psmoveservice/PSMoveController/PSMoveController.h
    ${ROOT_DIR}/psmoveservice/PSMoveController/PSMoveController.cpp)

# PSMoveDataFrame
list(APPEND TEST_CTRLR_INCL_DIRS ${ROOT_DIR}/psmovedataframe)
list(APPEND TEST_CTRLR_REQ_LIBS PSMoveDataFrame ${Boost_LIBRARIES})

add_executable(test_controller ${CMAKE_CURRENT_LIST_DIR}/test_controller.cpp ${TEST_CTRLR_SRC})
target_include_directories(test_controller PUBLIC ${TEST_CTRLR_INCL_DIRS})
target_link_libraries(test_controller ${PLATFORM_LIBS} ${TEST_CTRLR_REQ_LIBS})

#
# TEST_CONSOLE_CLIENT
#

set(TEST_CONSOLE_CLIENT_INCL_DIRS)
set(TEST_CONSOLE_CLIENT_REQ_LIBS)

# Boost.Application and type_index are header only (?)
list(APPEND TEST_CONSOLE_CLIENT_INCL_DIRS
    ${ROOT_DIR}/thirdparty/Boost.Application/include/
    ${ROOT_DIR}/thirdparty/type_index/include/)

# Protobuf (already found in top-level CMakeLists)
list(APPEND TEST_CONSOLE_CLIENT_INCL_DIRS ${PROTOBUF_INCLUDE_DIRS})
list(APPEND TEST_CONSOLE_CLIENT_REQ_LIBS ${PROTOBUF_LIBRARIES})

# Boost - TODO: Trim this list
find_package(Boost 1.59.0 REQUIRED QUIET COMPONENTS atomic chrono date_time filesystem program_options regex system thread)
list(APPEND TEST_CONSOLE_CLIENT_INCL_DIRS ${Boost_INCLUDE_DIRS})
list(APPEND TEST_CONSOLE_CLIENT_REQ_LIBS ${Boost_LIBRARIES})

# Parent projects
list(APPEND TEST_CONSOLE_CLIENT_INCL_DIRS 
    ${ROOT_DIR}/psmovedataframe/
    ${ROOT_DIR}/psmoveclient/)
list(APPEND TEST_CONSOLE_CLIENT_REQ_LIBS PSMoveDataFrame PSMoveClient)

add_executable(test_console_client test_console_client.cpp)
target_include_directories(test_console_client PUBLIC ${TEST_CONSOLE_CLIENT_INCL_DIRS})
target_link_libraries(test_console_client ${TEST_CONSOLE_CLIENT_REQ_LIBS})
