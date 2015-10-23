# - Try to find the freetype library
# Once done this defines
#
#  LIBUSB_FOUND - system has libusb
#  LIBUSB_INCLUDE_DIR - the libusb include directory
#  LIBUSB_LIBRARIES - Link these to use libusb

# Copyright (c) 2006, 2008  Laurent Montel, <montel@kde.org>
#
#
# Modified on 2015/07/14 by Chadwick Boulay <chadwick.boulay@gmail.com>
# to use static local libraries only.
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.


IF (LIBUSB_INCLUDE_DIR AND LIBUSB_LIBRARIES)

    # in cache already
    set(LIBUSB_FOUND TRUE)

ELSE (LIBUSB_INCLUDE_DIR AND LIBUSB_LIBRARIES)
    set(LIBUSB_ROOT ${CMAKE_CURRENT_LIST_DIR}/../thirdparty/libusb)
    # Because we want to use the static library,
    # look locally only.
    find_path(LIBUSB_INCLUDE_DIR
        NAMES
            libusb.h
        PATHS 
            ${LIBUSB_ROOT}/libusb
            ${LIBUSB_ROOT}/libusb-1.0
            /usr/local/include
    )
    # There are 4 platform-specific ways we might get the libraries.
    # 1 - Windows MSVC, download the source, compile with MSVC
    # 2 - Windows MSVC, download pre-compiled binaries <- Do not use; wrong CRT library
    # 3 - Windows MinGW, download pre-compiled binaries
    # 4 - OSX, download the source, build, but do not install
    # 5 - OSX, homebrew OR download the source, build, and install
    # Each of these puts the compiled library into a different folder
    # and that is also architecture-specific.

    set(LIBUSB_LIB_SEARCH_PATH_RELEASE ${LIBUSB_ROOT})
    set(LIBUSB_LIB_SEARCH_PATH_DEBUG ${LIBUSB_ROOT})
    IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
        IF(MINGW)
            set(LIBUSB_PLATFORM_PREFIX MinGW)  # Does this get used?
            #TODO: Add self-compiled folder for MinGW
            IF (${CMAKE_C_SIZEOF_DATA_PTR} EQUAL 8)
                list(APPEND LIBUSB_LIB_SEARCH_PATH_RELEASE
                    ${LIBUSB_ROOT}/MinGW64/static)
                list(APPEND LIBUSB_LIB_SEARCH_PATH_DEBUG
                    ${LIBUSB_ROOT}/MinGW64/static)
            ELSE()
                list(APPEND LIBUSB_LIB_SEARCH_PATH_RELEASE
                    ${LIBUSB_ROOT}/MinGW32/static)
                list(APPEND LIBUSB_LIB_SEARCH_PATH_DEBUG
                    ${LIBUSB_ROOT}/MinGW32/static)
            ENDIF()
        ELSE() # MSVC?
            set(LIBUSB_PLATFORM_PREFIX MS)  # Does this get used?
            IF (${CMAKE_C_SIZEOF_DATA_PTR} EQUAL 8)
                list(APPEND LIBUSB_LIB_SEARCH_PATH_RELEASE
                    ${LIBUSB_ROOT}/x64/Release/lib)
                list(APPEND LIBUSB_LIB_SEARCH_PATH_DEBUG
                    ${LIBUSB_ROOT}/x64/Debug/lib)
            ELSE()
                list(APPEND LIBUSB_LIB_SEARCH_PATH_RELEASE
                    ${LIBUSB_ROOT}/Win32/Release/lib)
                list(APPEND LIBUSB_LIB_SEARCH_PATH_DEBUG
                    ${LIBUSB_ROOT}/Win32/Debug/lib)
            ENDIF()
        ENDIF()
    ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        list(APPEND LIBUSB_LIB_SEARCH_PATH_RELEASE
                    ${LIBUSB_ROOT}/libusb/.libs
                    /usr/local/lib)
        list(APPEND LIBUSB_LIB_SEARCH_PATH_DEBUG
                    ${LIBUSB_ROOT}/libusb/.libs
                    /usr/local/lib)
    ENDIF()
    
    FIND_LIBRARY(LIBUSB_LIBRARY_RELEASE
            NAMES libusb-1.0.a libusb-1.0.lib libusb-1.0 usb-1.0 usb
            PATHS ${LIBUSB_LIB_SEARCH_PATH_RELEASE})
    FIND_LIBRARY(LIBUSB_LIBRARY_DEBUG
            NAMES libusb-1.0.a libusb-1.0.lib libusb-1.0 usb-1.0 usb
            PATHS ${LIBUSB_LIB_SEARCH_PATH_DEBUG})
    SET(LIBUSB_LIBRARIES
        debug ${LIBUSB_LIBRARY_DEBUG}
        optimized ${LIBUSB_LIBRARY_RELEASE})

  include(FindPackageHandleStandardArgs)
  FIND_PACKAGE_HANDLE_STANDARD_ARGS(LIBUSB DEFAULT_MSG LIBUSB_LIBRARIES LIBUSB_INCLUDE_DIR)

  MARK_AS_ADVANCED(LIBUSB_INCLUDE_DIR LIBUSB_LIBRARIES)

ENDIF (LIBUSB_INCLUDE_DIR AND LIBUSB_LIBRARIES)