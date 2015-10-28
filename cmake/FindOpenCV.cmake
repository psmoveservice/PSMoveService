######################################################################
## Find OpenCV - variables set:
##  OpenCV_FOUND
##  OpenCV_LIBRARIES
##  OpenCV_INCLUDE_DIRS
##
## This script is a combination from multiple sources that use
## different variable names; the names are reconciled at the end
## of the script.

###########################################################
#                  Find OpenCV Library
# See http://sourceforge.net/projects/opencvlibrary/
#----------------------------------------------------------
#
## 1: Setup:
# The following variables are optionally searched for defaults
#  OpenCV_DIR:            Base directory of OpenCv tree to use.
#
## 2: Variable
# The following are set after configuration is done:
#
#  OpenCV_FOUND
#  OpenCV_LIBS
#  OpenCV_INCLUDE_DIR
#  OpenCV_VERSION (OpenCV_VERSION_MAJOR, OpenCV_VERSION_MINOR, OpenCV_VERSION_PATCH)
#
#
# Deprecated variable are used to maintain backward compatibility with
# the script of Jan Woetzel (2006/09): www.mip.informatik.uni-kiel.de/~jw
#  OpenCV_INCLUDE_DIRS
#  OpenCV_LIBRARIES
#  OpenCV_LINK_DIRECTORIES
#
## 3: Version
#
# 2010/04/07 Benoit Rat, Correct a bug when OpenCVConfig.cmake is not found.
# 2010/03/24 Benoit Rat, Add compatibility for when OpenCVConfig.cmake is not found.
# 2010/03/22 Benoit Rat, Creation of the script.
#
#
# tested with:
# - OpenCV 2.1:  MinGW, MSVC2008
# - OpenCV 2.0:  MinGW, MSVC2008, GCC4
#
#
## 4: Licence:
#
# Copyright: 2009 Benoit Rat
# CopyPolicy: LGPLv2.1
# 
#----------------------------------------------------------

# Lorenzo Natale -- Feb 2011
# Improve compatibility with OpenCV package in Ubuntu 10.10
# Lorenzo Natale -- March 2011
# Removing OpenCV_INCLUDE_DIRS from required arguments
# Since not all version of OpenCV set OpenCV_INCLUDE_DIRS. Problem detected with OpenCV 2.0 OpenCVConfig.cmake
# directly calls INCLUDE_DIRECTORIES() and does not propagate any OpenCV_INCLUDE_ variable

# let's skip module mode, and see if a OpenCVConfig.cmake file is around
# this searches in system directories and ${OpenCV_DIR}

include(FindPackageHandleStandardArgs)

set(_OpenCV_FIND_QUIETLY ${OpenCV_FIND_QUIETLY})
find_package(OpenCV QUIET NO_MODULE)
set(OpenCV_FIND_QUIETLY ${_OpenCV_FIND_QUIETLY})

if(OpenCV_FOUND)
    set(OpenCV_CONFIG_MODE true)
    find_package_handle_standard_args(OpenCV DEFAULT_MSG OpenCV_CONFIG)
    ## OpenCVConfig.cmake sets OpenCV_LIBS OpenCV_INCLUDE_DIRS
    ## but we need OpenCV_LIBRARIES
    if(NOT DEFINED OpenCV_LIBRARIES)
        set(OpenCV_LIBRARIES ${OpenCV_LIBS})
    endif()
endif()


### If the above failed continues with traditional search method
## To keep backward compatibility we keep the whole script
## intact, however there is probably a lot of redundancy now
if(NOT OpenCV_CONFIG_MODE)
    find_path(OpenCV_DIR "OpenCVConfig.cmake" DOC "Root directory of OpenCV")

##====================================================
## Find OpenCV libraries
##----------------------------------------------------
if(EXISTS "${OpenCV_DIR}")

    #When its possible to use the Config script use it.
    if(EXISTS "${OpenCV_DIR}/OpenCVConfig.cmake")

        ## Include the standard CMake script
        include("${OpenCV_DIR}/OpenCVConfig.cmake")

        ## Search for a specific version
        set(CVLIB_SUFFIX "${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}")

    #Otherwise it try to guess it.
    else(EXISTS "${OpenCV_DIR}/OpenCVConfig.cmake")

        set(OPENCV_LIB_COMPONENTS cxcore cv ml highgui cvaux)
        find_path(OpenCV_INCLUDE_DIR "cv.h" PATHS "${OpenCV_DIR}" PATH_SUFFIXES "include" "include/opencv" DOC "")
        if(EXISTS  ${OpenCV_INCLUDE_DIR})
            include_directories(${OpenCV_INCLUDE_DIR})
        endif(EXISTS  ${OpenCV_INCLUDE_DIR})

        #Find OpenCV version by looking at cvver.h
        file(STRINGS ${OpenCV_INCLUDE_DIR}/cvver.h OpenCV_VERSIONS_TMP REGEX "^#define CV_[A-Z]+_VERSION[ \t]+[0-9]+$")
        string(REGEX REPLACE ".*#define CV_MAJOR_VERSION[ \t]+([0-9]+).*" "\\1" OpenCV_VERSION_MAJOR ${OpenCV_VERSIONS_TMP})
        string(REGEX REPLACE ".*#define CV_MINOR_VERSION[ \t]+([0-9]+).*" "\\1" OpenCV_VERSION_MINOR ${OpenCV_VERSIONS_TMP})
        string(REGEX REPLACE ".*#define CV_SUBMINOR_VERSION[ \t]+([0-9]+).*" "\\1" OpenCV_VERSION_PATCH ${OpenCV_VERSIONS_TMP})
        set(OpenCV_VERSION ${OpenCV_VERSION_MAJOR}.${OpenCV_VERSION_MINOR}.${OpenCV_VERSION_PATCH} CACHE STRING "" FORCE)
        set(CVLIB_SUFFIX "${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}")

    endif(EXISTS "${OpenCV_DIR}/OpenCVConfig.cmake")

    ## Initiate the variable before the loop
    set(OpenCV_LIBS "")
    set(OpenCV_FOUND_TMP true)

    ## Loop over each components
    foreach(__CVLIB ${OPENCV_LIB_COMPONENTS})

        find_library(OpenCV_${__CVLIB}_LIBRARY_DEBUG NAMES "${__CVLIB}${CVLIB_SUFFIX}d" "lib${__CVLIB}${CVLIB_SUFFIX}d" PATHS "${OpenCV_DIR}/lib" NO_DEFAULT_PATH)
        find_library(OpenCV_${__CVLIB}_LIBRARY_RELEASE NAMES "${__CVLIB}${CVLIB_SUFFIX}" "lib${__CVLIB}${CVLIB_SUFFIX}" PATHS "${OpenCV_DIR}/lib" NO_DEFAULT_PATH)
        #Remove the cache value
        set(OpenCV_${__CVLIB}_LIBRARY "" CACHE STRING "" FORCE)

        #both debug/release
        if(OpenCV_${__CVLIB}_LIBRARY_DEBUG AND OpenCV_${__CVLIB}_LIBRARY_RELEASE)
            set(OpenCV_${__CVLIB}_LIBRARY debug ${OpenCV_${__CVLIB}_LIBRARY_DEBUG} optimized ${OpenCV_${__CVLIB}_LIBRARY_RELEASE}  CACHE STRING "" FORCE)
        #only debug
        elseif(OpenCV_${__CVLIB}_LIBRARY_DEBUG)
            set(OpenCV_${__CVLIB}_LIBRARY ${OpenCV_${__CVLIB}_LIBRARY_DEBUG}  CACHE STRING "" FORCE)
        #only release
        elseif(OpenCV_${__CVLIB}_LIBRARY_RELEASE)
            set(OpenCV_${__CVLIB}_LIBRARY ${OpenCV_${__CVLIB}_LIBRARY_RELEASE}  CACHE STRING "" FORCE)
        #no library found
        else()
            set(OpenCV_FOUND_TMP false)
        endif()

        #Add to the general list
        if(OpenCV_${__CVLIB}_LIBRARY)
            set(OpenCV_LIBS ${OpenCV_LIBS} ${OpenCV_${__CVLIB}_LIBRARY})
        endif(OpenCV_${__CVLIB}_LIBRARY)

    endforeach(__CVLIB)


    set(OpenCV_FOUND ${OpenCV_FOUND_TMP} CACHE BOOL "" FORCE)


else(EXISTS "${OpenCV_DIR}")
    set(ERR_MSG "Please specify OpenCV directory using OpenCV_DIR env. variable")
endif(EXISTS "${OpenCV_DIR}")

##====================================================
if(NOT OpenCV_FOUND)
#
# Try another method to find OpenCV library
# Once run this will define:
#
# OPENCV_FOUND
# OPENCV_INCLUDE_DIR
# OPENCV_LIBRARIES
# OPENCV_LINK_DIRECTORIES
##
# deprecated:
# (JW)-- OPENCV_EXE_LINKER_FLAGS
#
# 2004/05 Jan Woetzel, Friso, Daniel Grest 
# 2006 complete rewrite by Jan Woetzel
##
# www.mip.informatik.uni-kiel.de/
# --------------------------------
#
# Modified by nat for YARP
# - nat modified to work on win: added "$ENV{OPENCV_DIR}/otherlibs/highgui/include"
# - nat, 19-oct 06: cxcore does not seem required by the opencv pakage that is
# distributed with Debian stable. Still a requirement in windows
# - nat, 20-oct 06: not smart enough to handle possible clashes between
# different versions of opencv if installed in the system. Be careful.
# - nat, 23-oct 06: First check OPENCV_DIR, OPENCV_ROOT or OPENCV_HOME
# - paulfitz, 5-nov 06: made link with libhighgui be default true
#   so that the opencv driver can be compiled. Nat, can you explain again 
#   which systems don't have this?
# - nat, 13-nov 06: differentiated LINK_LIBHIGHGUI default on windows and linux
#   still both true by default. I got a problem linking libhighgui on linux (did not
#   have time to debug it) and found that in any case this lib is no longer required
#   by opencv 0.9.9 (as a dependency). If older versions of opencv require libhighgui 
#   we can leave the default to TRUE. The libhighgui problem needs to be solved 
#   anyway...

    if(WIN32)
        set(LINK_LIB_HIGHGUI TRUE CACHE BOOL "Do you want to link against libhighgui?")
    else(WIN32)
        set(LINK_LIB_HIGHGUI TRUE CACHE BOOL "Do you want to link against libhighgui?")
    endif(WIN32)

    if(LINK_LIB_HIGHGUI AND WIN32)
        set(LINK_LIB_CVCAM)
    endif(LINK_LIB_HIGHGUI AND WIN32)


    set(IS_GNUCXX3 FALSE)
    set(IS_GNUCXX4 FALSE)
    if(${CMAKE_COMPILER_IS_GNUCXX})
        message(STATUS "Checking GNUCXX version 3/4 to determine  OpenCV /opt/net/ path")
        exec_program(${CMAKE_CXX_COMPILER} ARGS --version OUTPUT_VARIABLE CXX_COMPILER_VERSION)

        if(CXX_COMPILER_VERSION MATCHES ".*3\\.[0-9].*")
        #     message("DBG OpenCV for 3.x")
            set(IS_GNUCXX3 TRUE)
        # else(CXX_COMPILER_VERSION MATCHES ".*3\\.[0-9].*")
        #     message("DBG not 3.x")
        endif(CXX_COMPILER_VERSION MATCHES ".*3\\.[0-9].*")

        if(CXX_COMPILER_VERSION MATCHES ".*4\\.[0-9].*")
        #     message("DBG OpenCV for 4.x")
            set(IS_GNUCXX4 TRUE)
        # else(CXX_COMPILER_VERSION MATCHES ".*4\\.[0-9].*")
        #     message("DBG not 4.x")
        endif(CXX_COMPILER_VERSION MATCHES ".*4\\.[0-9].*")

    endif(${CMAKE_COMPILER_IS_GNUCXX})

    # CHECK OPENCV_ROOT
    if(EXISTS "$ENV{OPENCV_ROOT}")
        set(OPENCV_POSSIBLE_INCDIRS "$ENV{OPENCV_ROOT}"
                                    "$ENV{OPENCV_ROOT}/include"
                                    "$ENV{OPENCV_ROOT}/include/cv"
                                    "$ENV{OPENCV_ROOT}/include/opencv"
                                    "$ENV{OPENCV_ROOT}/cxcore/include"
                                    "$ENV{OPENCV_ROOT}/cv/include"
                                    "$ENV{OPENCV_ROOT}/cvaux/include"
                                    "$ENV{OPENCV_ROOT}/otherlibs/cvcam/include"
                                    "$ENV{OPENCV_ROOT}/otherlibs/highgui/include"
                                    "$ENV{OPENCV_ROOT}/otherlibs/highgui/")

        set(OPENCV_POSSIBLE_LIBRARY_PATHS "$ENV{OPENCV_ROOT}/lib"
                                          "$ENV{OPENCV_ROOT}")
    endif(EXISTS "$ENV{OPENCV_ROOT}")

    # CHECK OPENCV_DIR
    if(EXISTS "$ENV{OPENCV_DIR}")
        set(OPENCV_POSSIBLE_INCDIRS "$ENV{OPENCV_DIR}"
                                    "$ENV{OPENCV_DIR}/include"
                                    "$ENV{OPENCV_DIR}/include/cv"
                                    "$ENV{OPENCV_DIR}/include/opencv"
                                    "$ENV{OPENCV_DIR}/cxcore/include"
                                    "$ENV{OPENCV_DIR}/cv/include"
                                    "$ENV{OPENCV_DIR}/cvaux/include"
                                    "$ENV{OPENCV_DIR}/otherlibs/cvcam/include"
                                    "$ENV{OPENCV_DIR}/otherlibs/highgui/include"
                                    "$ENV{OPENCV_DIR}/otherlibs/highgui/")

        set(OPENCV_POSSIBLE_LIBRARY_PATHS "$ENV{OPENCV_DIR}"
                                          "$ENV{OPENCV_DIR}/lib")

    endif(EXISTS "$ENV{OPENCV_DIR}")

    # CHECK OPENCV_HOME
    if(EXISTS "$ENV{OPENCV_HOME}")
        set(OPENCV_POSSIBLE_INCDIRS "$ENV{OPENCV_HOME}"
                                    "$ENV{OPENCV_HOME}/include"
                                    "$ENV{OPENCV_HOME}/include/cv"
                                    "$ENV{OPENCV_HOME}/include/opencv")
    endif(EXISTS "$ENV{OPENCV_HOME}")

    if(NOT OPENCV_POSSIBLE_INCDIRS)
        set(OPENCV_POSSIBLE_INCDIRS "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\Intel(R) Open Source Computer Vision Library_is1;Inno Setup: App Path]"  
                                    "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\Intel(R) Open Source Computer Vision Library_is1;Inno Setup: App Path]/include"
                                    "$ENV{ProgramFiles}/OpenCV"
                                    "$ENV{ProgramFiles}/OpenCV/include"
                                    "$ENV{ProgramFiles}/OpenCV/cxcore/include"
                                    "$ENV{ProgramFiles}/OpenCV/cv/include"
                                    "$ENV{ProgramFiles}/OpenCV/cvaux/include"
                                    "$ENV{ProgramFiles}/OpenCV/otherlibs/cvcam/include"
                                    "$ENV{ProgramFiles}/OpenCV/otherlibs/highgui/include"
                                    "$ENV{ProgramFiles}/OpenCV/otherlibs/highgui"
                                    /usr/include/opencv
                                    /usr/local/include/opencv)
    endif(NOT OPENCV_POSSIBLE_INCDIRS)

    if(NOT OPENCV_POSSIBLE_LIBRARY_PATHS)
        set(OPENCV_POSSIBLE_LIBRARY_PATHS "$ENV{ProgramFiles}/OpenCV/lib"
                                          "/usr/local/lib"
                                          "/usr/lib"
                                          "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\Intel(R) Open Source Computer Vision Library_is1;Inno Setup: App Path]/lib")
    endif(NOT OPENCV_POSSIBLE_LIBRARY_PATHS)

    if(IS_GNUCXX3)
        set(OPENCV_POSSIBLE_INCDIRS ${OPENCV_POSSIBLE_INCDIRS}
                                    /opt/net/gcc33/OpenCV/
                                    /opt/net/gcc33/OpenCV/include
                                    /opt/net/gcc33/OpenCV/include/opencv)
    endif(IS_GNUCXX3)
    if(IS_GNUCXX4)
        set(OPENCV_POSSIBLE_INCDIRS ${OPENCV_POSSIBLE_INCDIRS}
                                    /opt/net/gcc41/OpenCV/
                                    /opt/net/gcc41/OpenCV/include
                                    /opt/net/gcc41/OpenCV/include/opencv)
    endif(IS_GNUCXX4)
    #message("DBG (OPENCV_POSSIBLE_INCDIRS=${OPENCV_POSSIBLE_INCDIRS}")

    # candidates for OpenCV library directories:
    if(IS_GNUCXX3)
        set(OPENCV_POSSIBLE_LIBRARY_PATHS ${OPENCV_POSSIBLE_LIBRARY_PATHS}
                                          /opt/net/gcc33/OpenCV
                                          /opt/net/gcc33/OpenCV/lib)
    endif(IS_GNUCXX3)

    if(IS_GNUCXX4)
        set(OPENCV_POSSIBLE_LIBRARY_PATHS ${OPENCV_POSSIBLE_LIBRARY_PATHS}
                                          /opt/net/gcc41/OpenCV
                                          /opt/net/gcc41/OpenCV/lib)
    endif(IS_GNUCXX4)
    #message("DBG (OPENCV_POSSIBLE_LIBRARY_PATHS=${OPENCV_POSSIBLE_LIBRARY_PATHS}")

    # find (all) header files for include directories:
    find_path(OPENCV_INCLUDE_DIR_CXCORE   cxcore.h  ${OPENCV_POSSIBLE_INCDIRS} )
    find_path(OPENCV_INCLUDE_DIR_CV       cv.h      ${OPENCV_POSSIBLE_INCDIRS} )
    find_path(OPENCV_INCLUDE_DIR_CVAUX    cvaux.h   ${OPENCV_POSSIBLE_INCDIRS} )
    find_path(OPENCV_INCLUDE_DIR_HIGHGUI  highgui.h ${OPENCV_POSSIBLE_INCDIRS} )
    find_path(OPENCV_INCLUDE_DIR_CVCAM    cvcam.h   ${OPENCV_POSSIBLE_INCDIRS} )

    #message("DBG OPENCV_INCLUDE_DIR_CV=${OPENCV_INCLUDE_DIR_CV}")

    # find (all) libraries - some dont exist on Linux
    find_library(OPENCV_LIBRARY
                 NAMES opencv cv cv0.9 cv0.9.5 cv0.9.6 cv0.9.7 cv0.9.8 cv0.9.9
                 PATHS ${OPENCV_POSSIBLE_LIBRARY_PATHS}
                 DOC "Location of the opencv lib")

    find_library(OPENCV_CVAUX_LIBRARY
                 NAMES cvaux cvaux0.9 cvaux0.9.5 cvaux0.9.6 cvaux0.9.7 cvaux0.9.8 cvaux0.9.9
                 PATHS ${OPENCV_POSSIBLE_LIBRARY_PATHS}
                 DOC "Location of the cvaux lib")

    find_library(OPENCV_CXCORE_LIBRARY
                 NAMES cxcore cxcore0.9  cxcore0.9.5 cxcore0.9.6 cxcore0.9.7 cxcore0.9.8 cxcore0.9.9
                 PATHS ${OPENCV_POSSIBLE_LIBRARY_PATHS}
                 DOC "Location of the cvcore lib")

    find_library(OPENCV_HIGHGUI_LIBRARY
                 NAMES highgui highgui0.9 highgui0.9.5 highgui0.9.6 highgui0.9.7 highgui0.9.8 highgui0.9.9
                 PATHS ${OPENCV_POSSIBLE_LIBRARY_PATHS}
                 DOC "Location of the highgui library")

    # optional CVCAM libs (WIN32 only)
    find_library(OPENCV_CVCAM_LIBRARY
                 NAMES cvcam
                 PATHS ${OPENCV_POSSIBLE_LIBRARY_PATHS}
                 DOC "Location of the cvcam lib")

    set(OPENCV_FOUND ON)

    ###########################################
    ## CHECK HEADER FILES

    # REQUIRED LIBS
    foreach(INCDIR OPENCV_INCLUDE_DIR_CV OPENCV_INCLUDE_DIR_CVAUX)
        if(${INCDIR})
            set(OPENCV_INCLUDE_DIR ${OPENCV_INCLUDE_DIR} ${${INCDIR}} )
        else(${INCDIR})
            message(STATUS "- DBG OPENCV_INCLUDE_DIR_CVCAM=${OPENCV_INCLUDE_DIR_CVCAM}")
            set(OPENCV_FOUND OFF)
        endif(${INCDIR})
    endforeach(INCDIR)

    # some people write opencv/foo.h, some write foo.h
    if(OPENCV_INCLUDE_DIR_CV)
        set(OPENCV_INCLUDE_DIR ${OPENCV_INCLUDE_DIR} "${OPENCV_INCLUDE_DIR_CV}/..")
    endif(OPENCV_INCLUDE_DIR_CV)


    # CVCAM exists only on Windows (check this -- nat)
    if(LINK_LIB_CVCAM)
        if(OPENCV_INCLUDE_DIR_CVCAM)
            set(OPENCV_INCLUDE_DIR ${OPENCV_INCLUDE_DIR} ${OPENCV_INCLUDE_DIR_CVCAM} )
        else(OPENCV_INCLUDE_DIR_CVCAM)
            # exists only on Windows, thus only there required
            if(WIN32)
                set(OPENCV_FOUND OFF)
                message(STATUS "- DBG OPENCV_INCLUDE_DIR_CVCAM=${OPENCV_INCLUDE_DIR_CVCAM} ")
            endif(WIN32)
        endif(OPENCV_INCLUDE_DIR_CVCAM)
    endif(LINK_LIB_CVCAM)

    # message("DBG OPENCV_INCLUDE_DIR=${OPENCV_INCLUDE_DIR}")

    # libcxcore does not seem to be always required (different distribution behave
    # differently)
    if(OPENCV_INCLUDE_DIR_CXCORE)
        set(OPENCV_INCLUDE_DIR ${OPENCV_INCLUDE_DIR} ${OPENCV_INCLUDE_DIR_CXCORE})
    else(OPENCV_INCLUDE_DIR_CXCORE)
        if(WIN32) #required in win
            set(OPENCV_FOUND OFF)
            message(STATUS "- DBG OPENCV_INCLUDE_DIR_CXCORE=${OPENCV_INCLUDE_DIR_CXCORE} ")
        endif(WIN32)
    endif(OPENCV_INCLUDE_DIR_CXCORE)

    if(LINK_LIB_HIGHGUI)
        if(OPENCV_INCLUDE_DIR_HIGHGUI)
            set(OPENCV_INCLUDE_DIR ${OPENCV_INCLUDE_DIR} ${OPENCV_INCLUDE_DIR_HIGHGUI})
        else(OPENCV_INCLUDE_DIR_HIGHGUI)
            if(WIN32)
                set(OPENCV_FOUND OFF)
                message(STATUS "- DBG OPENCV_INCLUDE_DIR_HIGHGUI=${OPENCV_INCLUDE_DIR_HIGHGUI} ")
            endif(WIN32)
        endif(OPENCV_INCLUDE_DIR_HIGHGUI)
    endif(LINK_LIB_HIGHGUI)

    #################################
    ## LIBRARIES

    # REQUIRED LIBRARIES
    foreach(LIBNAME OPENCV_LIBRARY OPENCV_CVAUX_LIBRARY)
        if(${LIBNAME})
            set(OPENCV_LIBRARIES ${OPENCV_LIBRARIES} ${${LIBNAME}})
        else(${LIBNAME})
            message(STATUS "${LIBNAME} not found turning off OPENCV_FOUND")
            set(OPENCV_FOUND OFF)
        endif(${LIBNAME})
    endforeach(LIBNAME)

    if(OPENCV_CXCORE_LIBRARY)
        set(OPENCV_LIBRARIES ${OPENCV_LIBRARIES} ${OPENCV_CXCORE_LIBRARY})
    else(OPENCV_CXCORE_LIBRARY)
        if(WIN32) #this is required on windows
            set(OPENCV_FOUND OFF)
            message(STATUS "OPENCV_CXCORE_LIBRARY not found turning off OPENCV_FOUND")
        endif(WIN32)
    endif(OPENCV_CXCORE_LIBRARY)

    # CVCAM exists only on Windows (check this -- nat)
    if(LINK_LIB_CVCAM)
        if(OPENCV_CVCAM_LIBRARY)
            set(OPENCV_LIBRARIES ${OPENCV_LIBRARIES} ${OPENCV_CVCAM_LIBRARY} )
        else(OPENCV_CVCAM_LIBRARY)
            if(WIN32)
                set(OPENCV_FOUND OFF)
                message(STATUS "OPENCV_CVCAM_LIBRARY not found turning off OPENCV_FOUND")
            endif(WIN32)
        endif(OPENCV_CVCAM_LIBRARY)
    endif(LINK_LIB_CVCAM)
    # message("DBG OPENCV_LIBRARIES=${OPENCV_LIBRARIES}")

    if(LINK_LIB_HIGHGUI)
        if(OPENCV_HIGHGUI_LIBRARY)
            set(OPENCV_LIBRARIES ${OPENCV_LIBRARIES} ${OPENCV_HIGHGUI_LIBRARY})
        else(OPENCV_HIGHGUI_LIBRARY)
            if(WIN32) #this is required on windows
                set(OPENCV_FOUND OFF)
                message(STATUS "OPENCV_HIGHGUI_LIBRARY not found turning off OPENCV_FOUND")
            endif(WIN32)
        endif(OPENCV_HIGHGUI_LIBRARY)
    endif(LINK_LIB_HIGHGUI)

    # get the link directory for rpath to be used with LINK_DIRECTORIES:
    if(OPENCV_LIBRARY)
        get_filename_component(OPENCV_LINK_DIRECTORIES ${OPENCV_LIBRARY} PATH)
    endif(OPENCV_LIBRARY)

    # display help message
    if(NOT OPENCV_FOUND)
        message(STATUS "OPENCV library or headers not found. Please search manually or set env. variable OPENCV_ROOT to guide search." )
    endif(NOT OPENCV_FOUND)

    mark_as_advanced(OPENCV_INCLUDE_DIR
                     OPENCV_INCLUDE_DIR_CXCORE
                     OPENCV_INCLUDE_DIR_CV
                     OPENCV_INCLUDE_DIR_CVAUX
                     OPENCV_INCLUDE_DIR_CVCAM
                     OPENCV_INCLUDE_DIR_HIGHGUI
                     OPENCV_LIBRARIES
                     OPENCV_LIBRARY
                     OPENCV_HIGHGUI_LIBRARY
                     OPENCV_CVAUX_LIBRARY
                     OPENCV_CXCORE_LIBRARY
                     OPENCV_CVCAM_LIBRARY
                     OPENCV_DIR)

    set(OpenCV_FOUND ${OPENCV_FOUND})
    set(OpenCV_INCLUDE_DIR ${OPENCV_INCLUDE_DIR})
    set(OpenCV_LIBS ${OPENCV_LIBRARIES})
endif(NOT OpenCV_FOUND)


##====================================================
## Print message
##----------------------------------------------------
if(NOT OpenCV_FOUND)
    # make FIND_PACKAGE friendly
    if(NOT OpenCV_FIND_QUIETLY)
        if(OpenCV_FIND_REQUIRED)
            message(FATAL_ERROR "OpenCV required but some headers or libs not found. ${ERR_MSG}")
        else(OpenCV_FIND_REQUIRED)
            message(STATUS "WARNING: OpenCV was not found. ${ERR_MSG}")
        endif(OpenCV_FIND_REQUIRED)
    endif(NOT OpenCV_FIND_QUIETLY)
endif(NOT OpenCV_FOUND)
##====================================================


##====================================================
## Backward compatibility
##----------------------------------------------------
if(OpenCV_FOUND)
    option(OpenCV_BACKWARD_COMPA "Add some variable to make this script compatible with the other version of FindOpenCV.cmake" false)
    if(OpenCV_BACKWARD_COMPA)
        find_path(OpenCV_INCLUDE_DIRS "cv.h" PATHS "${OpenCV_DIR}" PATH_SUFFIXES "include" "include/opencv" DOC "Include directory") 
        find_path(OpenCV_INCLUDE_DIR "cv.h" PATHS "${OpenCV_DIR}" PATH_SUFFIXES "include" "include/opencv" DOC "Include directory")
        set(OpenCV_LIBRARIES "${OpenCV_LIBS}" CACHE STRING "" FORCE)
    endif(OpenCV_BACKWARD_COMPA)
endif(OpenCV_FOUND)
##====================================================

set(OpenCV_LIBRARIES ${OpenCV_LIBS})
set(OpenCV_INCLUDE_DIRS ${OpenCV_INCLUDE_DIR})
# support old variable names
set(OPENCV_LIBRARIES ${OpenCV_LIBS})
set(OPENCV_INCLUDE_DIR ${OpenCV_INCLUDE_DIR})

# Not all version of OpenCV set OpenCV_INCLUDE_DIRS, removing it from required arguments
# Lorenzo Natale March 2011. See notes on top of file.
# find_package_handle_standard_args(OpenCV DEFAULT_MSG OpenCV_LIBRARIES OpenCV_INCLUDE_DIRS)
find_package_handle_standard_args(OpenCV DEFAULT_MSG OpenCV_LIBRARIES)

endif(NOT OpenCV_CONFIG_MODE)

# Set package properties if FeatureSummary was included
if(COMMAND set_package_properties)
    set_package_properties(OpenCV PROPERTIES DESCRIPTION "Open source computer vision library"
                                             URL "http://opencv.org/")
endif()