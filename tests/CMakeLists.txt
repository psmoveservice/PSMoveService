cmake_minimum_required(VERSION 3.0)

set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/..)

IF(MSVC)
    set(CompilerFlags
        CMAKE_CXX_FLAGS
        CMAKE_CXX_FLAGS_DEBUG
        CMAKE_CXX_FLAGS_RELEASE
        CMAKE_C_FLAGS
        CMAKE_C_FLAGS_DEBUG
        CMAKE_C_FLAGS_RELEASE
        )
    foreach(CompilerFlag ${CompilerFlags})
      string(REPLACE "/MD" "/MT" ${CompilerFlag} "${${CompilerFlag}}")
    endforeach()
ENDIF(MSVC)

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
# TEST_CAMERA
#

SET(TEST_CAMERA_SRC)
SET(TEST_CAMERA_REQ_LIBS)

# OpenCV
#set(OpenCV_DIR ${ROOT_DIR}/thirdparty/opencv/build CACHE PATH "Path to local OpenCV source tree" FORCE)
set(OpenCV_STATIC ON)
FIND_PACKAGE(OpenCV REQUIRED)
message(${OpenCV_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND TEST_CAMERA_REQ_LIBS ${OpenCV_LIBS})

# PS3EYEDriver - only necessary on Mac and Win64, but can be used in Win32 (I think)
IF (${CMAKE_SYSTEM_NAME} MATCHES "Darwin"
    OR (${CMAKE_SYSTEM_NAME} MATCHES "Windows"))
    #PS3EYEDriver
    include_directories(${ROOT_DIR}/thirdparty/PS3EYEDriver/src)
    list(APPEND TEST_CAMERA_SRC
        ${ROOT_DIR}/thirdparty/PS3EYEDriver/src/ps3eye.h
        ${ROOT_DIR}/thirdparty/PS3EYEDriver/src/ps3eye.cpp)
    #Requires libusb
    find_package(USB1 REQUIRED)
    include_directories(${LIBUSB_INCLUDE_DIR})
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
include_directories(${ROOT_DIR}/psmoveservice/PSEye)
list(APPEND TEST_CAMERA_SRC
    ${ROOT_DIR}/psmoveservice/PSEye/PSEyeVideoCapture.h
    ${ROOT_DIR}/psmoveservice/PSEye/PSEyeVideoCapture.cpp)

# The test_camera app
add_executable(test_camera ${CMAKE_CURRENT_LIST_DIR}/test_camera.cpp ${TEST_CAMERA_SRC})
target_link_libraries(test_camera ${PLATFORM_LIBS} ${TEST_CAMERA_REQ_LIBS})

#
# Test Controller
#

SET(TEST_CTRLR_SRC)
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
include_directories(${ROOT_DIR}/psmoveservice/PSMoveController)
list(APPEND TEST_CTRLR_SRC
    ${CMAKE_CURRENT_LIST_DIR}/../psmoveservice/PSMoveController/PSMoveController.h
    ${CMAKE_CURRENT_LIST_DIR}/../psmoveservice/PSMoveController/PSMoveController.cpp)

# PSMoveDataFrame
include_directories(${ROOT_DIR}/psmovedataframe)
list(APPEND TEST_CTRLR_REQ_LIBS PSMoveDataFrame)

add_executable(test_controller ${CMAKE_CURRENT_LIST_DIR}/test_controller.cpp ${TEST_CTRLR_SRC})
target_link_libraries(test_controller ${PLATFORM_LIBS} ${TEST_CTRLR_REQ_LIBS})