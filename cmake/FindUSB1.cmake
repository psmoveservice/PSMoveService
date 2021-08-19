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
    
    find_path(LIBUSB_INCLUDE_DIR
        NAMES
            libusb.h
        HINTS 
            /usr/local/include
		PATH_SUFFIXES
			libusb-1.0
    )
    
    FIND_LIBRARY(LIBUSB_LIBRARIES
            NAMES libusb-1.0.a libusb-1.0.lib libusb-1.0 usb-1.0 usb
            PATHS /usr/local/lib)

    include(FindPackageHandleStandardArgs)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(LIBUSB DEFAULT_MSG LIBUSB_LIBRARIES LIBUSB_INCLUDE_DIR)

    MARK_AS_ADVANCED(LIBUSB_INCLUDE_DIR LIBUSB_LIBRARIES)

ENDIF (LIBUSB_INCLUDE_DIR AND LIBUSB_LIBRARIES)