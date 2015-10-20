cmake_minimum_required(VERSION 3.0)

set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/..)

#
# TEST_CAMERA
#

SET(TEST_CAMERA_SRC)
SET(TEST_CAMERA_REQ_LIBS)

# Platform specific libraries
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    find_library(IOKIT_FRAMEWORK IOKit)
    find_library(COREFOUNDATION_FRAMEWORK CoreFoundation)
    #find_library(QUARTZCORE QuartzCore)
    find_library(APPKIT_FRAMEWORK AppKit)
    #find_library(QTKIT QTKit)
    find_library(AVFOUNDATION AVFoundation)
    #stdc++ ${QUARTZCORE} ${APPKIT_FRAMEWORK} ${QTKIT} ${AVFOUNDATION}
    list(APPEND TEST_CAMERA_REQ_LIBS
        ${COREFOUNDATION_FRAMEWORK}
        ${IOKIT_FRAMEWORK}
        ${APPKIT_FRAMEWORK}
        ${AVFOUNDATION})
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    #OpenCV extra dependencies: comctl32 gdi32 ole32 setupapi ws2_32 vfw32
    #list(APPEND TEST_CAMERA_REQ_LIBS vfw32 comctl32)
    IF(MINGW)
        #list(APPEND TEST_CAMERA_REQ_LIBS stdc++)
    ENDIF(MINGW)
ELSE() #Linux
ENDIF()

# OpenCV
#set(OpenCV_DIR ${ROOT_DIR}/thirdparty/opencv/build CACHE PATH "Path to local OpenCV source tree" FORCE)
set(OpenCV_STATIC ON)
FIND_PACKAGE(OpenCV REQUIRED)
message(${OpenCV_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND TEST_CAMERA_REQ_LIBS ${OpenCV_LIBS})

# PS3EYEDriver - only on Mac and Win64
IF (${CMAKE_SYSTEM_NAME} MATCHES "Darwin"
    OR (${CMAKE_SYSTEM_NAME} MATCHES "Windows" AND ${CMAKE_C_SIZEOF_DATA_PTR} EQUAL 8))
    #PS3EYEDriver
    include_directories(${ROOT_DIR}/thirdparty/PS3EYEDriver/src)
    list(APPEND TEST_CAMERA_SRC ${ROOT_DIR}/thirdparty/PS3EYEDriver/src/ps3eye.cpp)
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
include_directories(${CMAKE_CURRENT_LIST_DIR}/../psmoveservice/PSEye)
list(APPEND TEST_CAMERA_SRC ${CMAKE_CURRENT_LIST_DIR}/../psmoveservice/PSEye/PSEyeVideoCapture.cpp)

IF(MSVC)
	SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_DEBUG} /MT")
	SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
ENDIF(MSVC)

add_executable(test_camera ${CMAKE_CURRENT_LIST_DIR}/test_camera.cpp ${TEST_CAMERA_SRC})
target_link_libraries(test_camera ${TEST_CAMERA_REQ_LIBS})
