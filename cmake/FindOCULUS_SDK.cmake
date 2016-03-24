# - Try to find Oculus Rift SDK
#
# Originally from
# https://github.com/bjornblissing/osgoculusviewer/blob/master/cmake/FindOculusSDK.cmake
# Modified to work with Mac/Linux using
# https://github.com/stevenlovegrove/Pangolin/blob/master/CMakeModules/FindOculus.cmake
#
#  OCULUS_SDK_INCLUDE_DIRS - where to find OVR.h, etc.
#  OCULUS_SDK_LIBRARIES    - List of libraries when using OculusSDK.
#  OCULUS_SDK_FOUND        - True if OculusSDK found.
#  OCULUS_SDK_VERSION      - Version of the OculusSDK if found

IF (DEFINED ENV{OCULUS_SDK_ROOT_DIR})
    SET(OCULUS_SDK_ROOT_DIR "$ENV{OCULUS_SDK_ROOT_DIR}")
ENDIF()
SET(OCULUS_SDK_ROOT_DIR
    "${OCULUS_SDK_ROOT_DIR}"
    CACHE
    PATH
    "Root directory to search for OculusSDK")

SET(OCULUS_SDK_SEARCH_PATHS
  ${OCULUS_SDK_ROOT_DIR}  
  ${CMAKE_CURRENT_LIST_DIR}/../thirdparty/OculusSDK-0.8.0.0/LibOVR
  ${CMAKE_CURRENT_LIST_DIR}/../thirdparty/OculusSDK-0.5.0.1/LibOVR
  ${CMAKE_CURRENT_LIST_DIR}/../thirdparty/OculusSDK/LibOVR
  ${CMAKE_CURRENT_LIST_DIR}/../thirdparty/LibOVR
  ~/Library/Frameworks
  /Library/Frameworks
  /usr/include/LibOVR
  /usr/local/include/LibOVR
  /opt/local/include/LibOVR
  /usr
  /usr/local
  /opt/local
)

FIND_PATH(
  OCULUS_SDK_INCLUDE_DIRS
  NAMES
    OVR.h
  PATH_SUFFIXES
    Include
    include
  PATHS
    ${OCULUS_SDK_SEARCH_PATHS}
)

# Try to ascertain the version of the SDK
IF(OCULUS_SDK_INCLUDE_DIRS)
    SET(_OCULUS_VERSION_FILE "${OCULUS_SDK_INCLUDE_DIRS}/OVR_Version.h")
    
    IF(EXISTS "${_OCULUS_VERSION_FILE}") 
        FILE(STRINGS "${_OCULUS_VERSION_FILE}" _OCULUS_VERSION_FILE_CONTENTS REGEX "#define OVR_[A-Z]+_VERSION[ \t]+[0-9]+")

        STRING(REGEX REPLACE ".*#define OVR_PRODUCT_VERSION[ \t]+([0-9]+).*" "\\1" OCULUS_SDK_VERSION_PRODUCT ${_OCULUS_VERSION_FILE_CONTENTS})
        STRING(REGEX REPLACE ".*#define OVR_MAJOR_VERSION[ \t]+([0-9]+).*" "\\1" OCULUS_SDK_VERSION_MAJOR ${_OCULUS_VERSION_FILE_CONTENTS})
        STRING(REGEX REPLACE ".*#define OVR_MINOR_VERSION[ \t]+([0-9]+).*" "\\1" OCULUS_SDK_VERSION_MINOR ${_OCULUS_VERSION_FILE_CONTENTS})
        STRING(REGEX REPLACE ".*#define OVR_PATCH_VERSION[ \t]+([0-9]+).*" "\\1" OCULUS_SDK_VERSION_PATCH ${_OCULUS_VERSION_FILE_CONTENTS})
        
        SET(OCULUS_SDK_VERSION "${OCULUS_SDK_VERSION_PRODUCT}.${OCULUS_SDK_VERSION_MAJOR}.${OCULUS_SDK_VERSION_MINOR}.${OCULUS_SDK_VERSION_PATCH}" CACHE INTERNAL "The version of Oculus SDK which was detected")
    ENDIF()
ENDIF()

SET(OCULUS_SDK_PLATFORM_SUFFIX "")
SET(OCULUS_SDK_PLATFORM_SUFFIX_DEBUG "")
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  # Determine architecture
  IF (MSVC)
    IF(CMAKE_SIZEOF_VOID_P MATCHES "8")
      SET(_OCULUS_SDK_LIB_ARCH "x64")
    ELSE()
      SET(_OCULUS_SDK_LIB_ARCH "Win32")
    ENDIF()
    MARK_AS_ADVANCED(_OCULUS_SDK_LIB_ARCH)

    # Determine the compiler version for Visual Studio
    # Visual Studio 2010
    IF(MSVC10)
        SET(_OCULUS_MSVC_DIR "VS2010")
    ENDIF()
    # Visual Studio 2012
    IF(MSVC11)
        SET(_OCULUS_MSVC_DIR "VS2012")
    ENDIF()
    # Visual Studio 2013
    IF(MSVC12)
        SET(_OCULUS_MSVC_DIR "VS2013")
    ENDIF()
  ENDIF()

  # Append "d" to debug libs on windows platform
  SET(CMAKE_DEBUG_POSTFIX d)
  
  SET(OCULUS_SDK_PLATFORM_SUFFIX Lib/Windows/${_OCULUS_SDK_LIB_ARCH}/Release/${_OCULUS_MSVC_DIR})
  SET(OCULUS_SDK_PLATFORM_SUFFIX_DEBUG Lib/Windows/${_OCULUS_SDK_LIB_ARCH}/Debug/${_OCULUS_MSVC_DIR})
  
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

  SET(OCULUS_SDK_PLATFORM_SUFFIX Lib/Mac/Release)
  SET(OCULUS_SDK_PLATFORM_SUFFIX_DEBUG Lib/Mac/Debug)

ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")


ENDIF()

# Look for the library.
FIND_LIBRARY(OCULUS_SDK_LIBRARY
  NAMES
    ovr
    libovr
    libovr64
    OVR
  PATH_SUFFIXES
    ${OCULUS_SDK_PLATFORM_SUFFIX}
  PATHS
    ${OCULUS_SDK_SEARCH_PATHS} 
    ${OCULUS_SDK_INCLUDE_DIRS}/..
)

# Look for the debug library
FIND_LIBRARY(OCULUS_SDK_LIBRARY_DEBUG
  NAMES
    libovr${CMAKE_DEBUG_POSTFIX}
    libovr64${CMAKE_DEBUG_POSTFIX}
    ovr${CMAKE_DEBUG_POSTFIX}
    ovr
    libovr
  PATH_SUFFIXES
    ${OCULUS_SDK_PLATFORM_SUFFIX_DEBUG}
  PATHS
    ${OCULUS_SDK_SEARCH_PATHS} 
    ${OCULUS_SDK_INCLUDE_DIRS}/..
)

MARK_AS_ADVANCED(OCULUS_SDK_LIBRARY)
MARK_AS_ADVANCED(OCULUS_SDK_LIBRARY_DEBUG)

IF(${OCULUS_SDK_LIBRARY_DEBUG})
SET(OCULUS_SDK_LIBRARIES optimized ${OCULUS_SDK_LIBRARY} debug ${OCULUS_SDK_LIBRARY_DEBUG})
ELSE()
SET(OCULUS_SDK_LIBRARIES ${OCULUS_SDK_LIBRARY})
ENDIF()

# Additional required libraries for Mac & Linux
IF(OCULUS_SDK_INCLUDE_DIRS AND OCULUS_SDK_LIBRARIES)
    IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        find_library(CARBON_LIBRARIES NAMES Carbon)
        find_library(IOKIT_LIBRARIES NAMES IOKit)
        list(APPEND OCULUS_SDK_LIBRARIES ${CARBON_LIBRARIES})
        list(APPEND OCULUS_SDK_LIBRARIES ${IOKIT_LIBRARIES})
    ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
        FIND_PACKAGE(Xrandr)
        IF( Xrandr_FOUND )
            list(APPEND OCULUS_SDK_LIBRARIES ${Xrandr_LIBRARIES} -ludev -lXrandr)
        ENDIF()
    ENDIF()
ENDIF(OCULUS_SDK_INCLUDE_DIRS AND OCULUS_SDK_LIBRARIES)

# handle the QUIETLY and REQUIRED arguments and set OCULUS_SDK_FOUND to TRUE if
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(OCULUS_SDK DEFAULT_MSG OCULUS_SDK_LIBRARIES OCULUS_SDK_INCLUDE_DIRS )

MARK_AS_ADVANCED(OCULUS_SDK_LIBRARIES OCULUS_SDK_INCLUDE_DIRS)