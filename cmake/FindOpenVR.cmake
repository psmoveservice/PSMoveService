# - try to find the OpenVR SDK - currently designed for the version on GitHub.
#
# Cache Variables: (probably not for direct use in your scripts)
#  OPENVR_INCLUDE_DIR
#
# Non-cache variables you might use in your CMakeLists.txt:
#  OPENVR_FOUND
#  OPENVR_INCLUDE_DIR
#  OPENVR_BINARIES_DIR
#  OPENVR_LIBRARIES
#
# Requires these CMake modules:
#  FindPackageHandleStandardArgs (known included with CMake >=2.6.2)
#
# Adapted from the LibUSB cmake script
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

IF (OPENVR_INCLUDE_DIR AND OPENVR_LIBRARIES AND OPENVR_BINARIES_DIR)
    # in cache already
    set(OPENVR_FOUND TRUE)

ELSE (OPENVR_INCLUDE_DIR AND OPENVR_LIBRARIES AND OPENVR_BINARIES_DIR)
    set(OPENVR_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/../thirdparty/openvr)

    # Find the include path
    find_path(OPENVR_INCLUDE_DIR
        NAMES
            openvr.h
        PATHS 
            ${OPENVR_ROOT_DIR}/headers
            /usr/local/include)

    # Find the libraries to include
    set(OPENVR_LIB_SEARCH_PATH ${OPENVR_ROOT_DIR}/lib)
    IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
        IF (${CMAKE_C_SIZEOF_DATA_PTR} EQUAL 8)
            list(APPEND OPENVR_LIB_SEARCH_PATH
                ${OPENVR_ROOT_DIR}/lib/win64)
        ELSE()
            list(APPEND OPENVR_LIB_SEARCH_PATH
                ${OPENVR_ROOT_DIR}/lib/win32)
        ENDIF()
    ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
        IF (${CMAKE_C_SIZEOF_DATA_PTR} EQUAL 8)
            list(APPEND OPENVR_LIB_SEARCH_PATH
                ${OPENVR_ROOT_DIR}/lib/linux64
                /usr/local/lib)
        ELSE()
            list(APPEND OPENVR_LIB_SEARCH_PATH
                ${OPENVR_ROOT_DIR}/lib/linux32
                /usr/local/lib)  
        ENDIF()      
    ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        list(APPEND OPENVR_LIB_SEARCH_PATH
            ${OPENVR_ROOT_DIR}/lib/osx32
            /usr/local/lib)
    ENDIF()
    
    FIND_LIBRARY(OPENVR_LIBRARY
            NAMES libopenvr_api.dylib libopenvr_api.so openvr_api.lib libopenvr_api openvr_api
            PATHS ${OPENVR_LIB_SEARCH_PATH})
    SET(OPENVR_LIBRARIES ${OPENVR_LIBRARY})
    
    # Find the path to copy DLLs from
    set(OPENVR_BIN_SEARCH_PATH ${OPENVR_ROOT_DIR}/bin)
    IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
        IF (${CMAKE_C_SIZEOF_DATA_PTR} EQUAL 8)
            list(APPEND OPENVR_BIN_SEARCH_PATH
                ${OPENVR_ROOT_DIR}/bin/win64)
        ELSE()
            list(APPEND OPENVR_BIN_SEARCH_PATH
                ${OPENVR_ROOT_DIR}/bin/win32)
        ENDIF()
    ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
        IF (${CMAKE_C_SIZEOF_DATA_PTR} EQUAL 8)
            list(APPEND OPENVR_BIN_SEARCH_PATH
                ${OPENVR_ROOT_DIR}/bin/linux64
                /usr/local/bin)
        ELSE()
            list(APPEND OPENVR_BIN_SEARCH_PATH
                ${OPENVR_ROOT_DIR}/bin/linux32
                /usr/local/bin)  
        ENDIF()      
    ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        list(APPEND OPENVR_BIN_SEARCH_PATH
            ${OPENVR_ROOT_DIR}/bin/osx32
            /usr/local/bin)
    ENDIF()

    find_path(OPENVR_BINARIES_DIR
        NAMES
            libopenvr_api.so libopenvr_api.dylib openvr_api.dll
        PATHS 
            ${OPENVR_BIN_SEARCH_PATH})    

    # Register OPENVR_LIBRARIES, OPENVR_INCLUDE_DIR, OPENVR_BINARIES_DIR
    include(FindPackageHandleStandardArgs)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(OPENVR DEFAULT_MSG OPENVR_LIBRARIES OPENVR_INCLUDE_DIR OPENVR_BINARIES_DIR)

    MARK_AS_ADVANCED(OPENVR_INCLUDE_DIR OPENVR_LIBRARIES)           
     
ENDIF (OPENVR_INCLUDE_DIR AND OPENVR_LIBRARIES AND OPENVR_BINARIES_DIR)