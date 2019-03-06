# This is where we find thirdparty libraries and packages that are common to two or more targets.

# Platform specific libraries
SET(PLATFORM_LIBS)
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
    #hid required for HidD_SetOutputReport() in DualShock4 controller
    #setupapi required by hidapi
    #dinput8 required by libstem_gamepad
    list(APPEND PLATFORM_LIBS bthprops setupapi hid dinput8)
    IF(MINGW)
        #list(APPEND PLATFORM_LIBS stdc++)
    ENDIF(MINGW)
ELSE() #Linux
    list(APPEND PLATFORM_LIBS rt)
ENDIF()

IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    find_package(PkgConfig REQUIRED)
ENDIF()


# Eigen3
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    find_package(Eigen3 CONFIG REQUIRED)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    MESSAGE(STATUS "Using homebrew Eigen3")
    find_package(Eigen3 REQUIRED CONFIG PATHS /usr/local/opt/eigen/lib/cmake/eigen3)
ELSE()
    find_package(Eigen3 REQUIRED)  # TODO: Test as CONFIG
ENDIF()
# Use with:
# target_link_libraries(main PRIVATE Eigen3::Eigen)


# OpenCV
# Override by adding "-DOpenCV_DIR=C:\path\to\opencv\build" to your cmake command
IF(NOT OpenCV_DIR)
    IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        # Location of homebrew opencv3's OpenCVConfig.cmake
        # Alternatively, can do `brew ln opencv3 --force`
        MESSAGE(STATUS "Using homebrew opencv3")
        set(OpenCV_DIR "/usr/local/opt/opencv3/share/OpenCV")
    ELSE()
        set(OpenCV_DIR “/usr/share/OpenCV”)
    ENDIF()#Windows or Darwin
    LIST(APPEND CMAKE_MODULE_PATH ${OpenCV_DIR})
ENDIF(NOT OpenCV_DIR)
set(OpenCV_STATIC ON)
FIND_PACKAGE(OpenCV REQUIRED)
# Use with:
#target_include_directories(main PRIVATE ${OpenCV_INCLUDE_DIRS})
#target_link_libraries(main PRIVATE ${OpenCV_LIBS})

# Boost
IF(MSVC)
    # Disable asio auto linking in date-time and regex
    add_definitions(-DBOOST_DATE_TIME_NO_LIB)
    add_definitions(-DBOOST_REGEX_NO_LIB)
    # fix: fatal error C1128: number of sections exceeded object file format limit
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
ENDIF()
SET(Boost_DEBUG                OFF) #Switch this and next to ON for help debugging Boost problems.
SET(Boost_DETAILED_FAILURE_MSG OFF)
set(Boost_USE_STATIC_LIBS      OFF) # only find static libs
set(Boost_USE_MULTITHREADED    ON)
set(Boost_USE_STATIC_RUNTIME   OFF) # Default = OFF
find_package(Boost REQUIRED)
# Because the version of Boost supported depends on cmake and can often lag behind,
# we cannot use (e.g.) target_link_libraries(${target} Boost::filesystem)
# So, use:
# find_package(Boost REQUIRED COMPONENTS <list of components>)
# target_include_directories(${target} ${Boost_INCLUDE_DIRS})
# target_link_libraries(${target} ${Boost_LIBRARIES})


# Protobuf
set(Protobuf_DEBUG OFF)  # Turn on to debug protobuf issues.
find_package(Protobuf REQUIRED)
# Use with:
# target_include_directories(${target} ${Protobuf_INCLUDE_DIRS})
# target_link_libraries(${target} ${Protobuf_LIBRARIES})
# Also, .proto files can be added to a target with:
# protobuf_generate(TARGET ${target} PROTOS ${my.proto})

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    # protobuf current generates many warnings in MacOS:
    #'OSMemoryBarrier' is deprecated: first deprecated in macOS 10.12 - Use std::atomic_thread_fence() from <atomic> instead
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
ENDIF()


# SDL and GL
#set(SDL_GL_INCLUDE_DIRS)
#set(SDL_GL_LIBS)
message(STATUS "ThirdParty.cmake finding SDL")
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    find_library(OPENGL_FRAMEWORK OpenGL)
    find_package(SDL2)
    set(APPEND SDL_GL_INCLUDE_DIRS ${SDL2_INCLUDE_DIR})
    list(APPEND SDL_GL_LIBS
        ${SDL2_LIBRARY} ${OPENGL_FRAMEWORK} ${GLUT_FRAMEWORK})
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    find_package(SDL2 CONFIG REQUIRED)
#    list(APPEND SDL_GL_INCLUDE_DIRS ${SDL2_INCLUDE_DIR})
#    list(APPEND SDL_GL_LIBS ${SDL2_LIBRARY} GL)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    find_package(SDL2 CONFIG REQUIRED)
ENDIF()
# Use with:
# target_include_directories(main ${SDL2_INCLUDE_DIRS})
# target_link_libraries(main PRIVATE ${SDL2_LIBRARIES})


# For PSEye camera
set(PSEYE_SRC)
set(PSEYE_INCLUDE_DIRS)
set(PSEYE_LIBRARIES)
# PS3EYEDriver - only necessary on Mac and Win64, but can be used in Win32 (I think)
IF (${CMAKE_SYSTEM_NAME} MATCHES "Darwin"
    OR (${CMAKE_SYSTEM_NAME} MATCHES "Windows"))
    #PS3EYEDriver
    list(APPEND PSEYE_INCLUDE_DIRS ${ROOT_DIR}/thirdparty/PS3EYEDriver/src)
    list(APPEND PSEYE_SRC
        ${ROOT_DIR}/thirdparty/PS3EYEDriver/src/ps3eye.h
        ${ROOT_DIR}/thirdparty/PS3EYEDriver/src/ps3eye.cpp)
    #Requires libusb
    find_package(USB1 REQUIRED)
    list(APPEND PSEYE_INCLUDE_DIRS ${LIBUSB_INCLUDE_DIR})
    list(APPEND PSEYE_LIBRARIES ${LIBUSB_LIBRARIES})
    add_definitions(-DHAVE_PS3EYE)
    IF (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-braced-scalar-init")
    ENDIF()
ENDIF()
# CL EYE - only on Win32
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows"
    AND NOT(${CMAKE_C_SIZEOF_DATA_PTR} EQUAL 8))
    add_definitions(-DHAVE_CLEYE)
    list(APPEND PSEYE_INCLUDE_DIRS ${ROOT_DIR}/thirdparty/CLEYE)
    list(APPEND PSEYE_LIBRARIES ${ROOT_DIR}/thirdparty/CLEYE/x86/lib/CLEyeMulticam.lib)
    find_path(CL_EYE_SDK_PATH CLEyeMulticam.dll
        HINTS C:/Windows/SysWOW64)
    #The non-Multicam version does not require any libs/dlls/includes
    #Uses OpenCV for video. Uses the registry for settings.
    #But libusb is required for enumerating the devices and checking for the CL Eye Driver.
    # LIBUSB already added above
    #find_package(USB1 REQUIRED)
    #list(APPEND PSEYE_INCLUDE_DIRS ${LIBUSB_INCLUDE_DIR})
    #list(APPEND PSEYE_LIBRARIES ${LIBUSB_LIBRARIES})
ENDIF()


# hidapi
set(HIDAPI_INCLUDE_DIRS ${ROOT_DIR}/thirdparty/hidapi/hidapi)
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(HIDAPI_SRC ${ROOT_DIR}/thirdparty/hidapi/windows/hid.c)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(HIDAPI_SRC ${ROOT_DIR}/thirdparty/hidapi/mac/hid.c)
ELSE()
    set(HIDAPI_SRC ${ROOT_DIR}/thirdparty/hidapi/linux/hid.c)
#   I tried using hidapi from apt-get but it didn't work.
#    pkg_check_modules(HIDAPI REQUIRED hidapi-libusb)
#    pkg_check_modules(HIDAPI REQUIRED hidapi-hidraw)
    pkg_check_modules(UDEV REQUIRED libudev)
    list(APPEND HIDAPI_INCLUDE_DIRS ${UDEV_INCLUDEDIR})
    list(APPEND HIDAPI_LIBRARIES ${UDEV_LIBRARIES})
    pkg_check_modules(BLUEZ REQUIRED bluez)
    list(APPEND HIDAPI_INCLUDE_DIRS ${BLUEZ_INCLUDEDIR})
    list(APPEND HIDAPI_LIBRARIES ${BLUEZ_LIBRARIES})
ENDIF()


# libstem_gamepad
set(LIBSTEM_GAMEPAD_INCLUDE_DIRS ${ROOT_DIR}/thirdparty/libstem_gamepad/source)
set(LIBSTEM_GAMEPAD_SRC ${ROOT_DIR}/thirdparty/libstem_gamepad/source/gamepad/Gamepad_private.c)
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    list(APPEND LIBSTEM_GAMEPAD_SRC ${ROOT_DIR}/thirdparty/libstem_gamepad/source/gamepad/Gamepad_windows_dinput.c)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    list(APPEND LIBSTEM_GAMEPAD_SRC ${ROOT_DIR}/thirdparty/libstem_gamepad/source/gamepad/Gamepad_macosx.c)
ELSE()
    list(APPEND LIBSTEM_GAMEPAD_SRC ${ROOT_DIR}/thirdparty/libstem_gamepad/source/gamepad/Gamepad_linux.c)
ENDIF()
