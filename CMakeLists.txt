cmake_minimum_required(VERSION 3.0)
project(PSMoveService VERSION 0.0.1)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
IF (NOT MSVC)
    set(CMAKE_CXX_FLAGS "-std=c++11 -stdlib=libc++")
ENDIF()

# When not using MSVC, we recommend using system-wide libraries
# (installed via homebrew on Mac or apt-get in Linux/Ubuntu)
# In MSVC, we recommend downloading pre-compiled binaries where
# possible, and building Protobuf yourself (see README.md)

# OpenCV
# Override by adding "-DOpenCV_DIR=C:\path\to\opencv\build" to your cmake command
IF(NOT OpenCV_DIR)
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    # Default location of pre-compiled OpenCV for Windows
    set(OpenCV_DIR "C:/OpenCV-3.0.0/build")# CACHE PATH "Path to OpenCV" FORCE)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    # Location of homebrew opencv3
    # Alternatively, can do `brew ln opencv3 --force`
    message("-- Using homebrew opencv3")
    set(OpenCV_DIR "/usr/local/Cellar/opencv3/3.0.0/share/OpenCV")
ENDIF()#Windows or Darwin
ENDIF(NOT OpenCV_DIR)

# Boost & Protobuf
IF(MSVC)
    # Default location of pre-compiled Boost for Windows
    # Override by adding "-DBOOST_ROOT=C:\path\to\boost\ -DBOOST_LIBRARYDIR=C:\path\to\boost\lib32-msvc-12.0\" to your cmake command
    IF (NOT BOOST_ROOT)
        SET(BOOST_ROOT "C:/boost_1_59_0/")
        SET(BOOST_LIBRARYDIR "C:/boost_1_59_0/lib32-msvc-12.0/")
    ENDIF()
    
    # Default location of protobuf for Windows
    #SET(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} "${CMAKE_CURRENT_LIST_DIR}/thirdparty/protobuf")
ENDIF()

IF(MSVC)
#I cannot remember which one, but one of our dependencies
#links against static runtime, so we need our apps to link
#against the static runtime too.
#https://cmake.org/Wiki/CMake_FAQ#How_can_I_build_my_MSVC_application_with_a_static_runtime.3F

    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")

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

# Boost Settings
SET(Boost_DEBUG                OFF) #Switch this and next to ON for help debugging Boost problems.
SET(Boost_DETAILED_FAILURE_MSG OFF)
set(Boost_USE_STATIC_LIBS      ON) # only find static libs
set(Boost_USE_MULTITHREADED    ON)
set(Boost_USE_STATIC_RUNTIME   ON) #Not default. Because our app is linking against static runtime (see above).
find_package(Boost REQUIRED)
# Future targets will do their own find so they can specify components. Here we just want a print to screen.

# Find Protobuf
IF(MSVC)
    set(PROTOBUF_SRC_ROOT_FOLDER ${CMAKE_CURRENT_LIST_DIR}/thirdparty/protobuf)
    #PROTOBUF_IMPORT_DIRS ?
ENDIF(MSVC)
set(PROTOBUF_ORIG_FIND_LIBRARY_SUFFIXES "${CMAKE_FIND_LIBRARY_SUFFIXES}") # Store original
set(CMAKE_FIND_LIBRARY_SUFFIXES .a .lib .so .dylib .dll)  # Prefer static libs
find_package(Protobuf REQUIRED)
set(CMAKE_FIND_LIBRARY_SUFFIXES "${PROTOBUF_ORIG_FIND_LIBRARY_SUFFIXES}")  # Restore original
include_directories(${CMAKE_BINARY_DIR}/psmoveprotocol)  # This is where the .proto files are compiled to.

# Step into the subdirectories
add_subdirectory(src)