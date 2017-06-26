LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
IF (NOT MSVC)
    set(CMAKE_CXX_FLAGS "-std=c++11")
    IF (NOT CMAKE_COMPILER_IS_GNUCXX)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
    ENDIF()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-switch")
ENDIF()

# Disable GLM warnings in Darwin
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    #warning: 'register' storage class specifier is deprecated and incompatible with C++1z
    #warning: operator '<<' has lower precedence than '-'; '-' will be evaluated first
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-register -Wno-shift-op-parentheses")
ENDIF()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Shared architecture label used for install folder locations
if (${CMAKE_C_SIZEOF_DATA_PTR} EQUAL 8)
    set(BITNESS "64")
    if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
        set(ARCH_LABEL "Win64")
    elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        set(ARCH_LABEL "OSX64")
    else()
        set(ARCH_LABEL "Linux64")
    endif()
else()
    set(BITNESS "32")
    if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
        set(ARCH_LABEL "Win32")
    elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        set(ARCH_LABEL "OSX32")
    else()
        set(ARCH_LABEL "Linux32")
    endif()
endif()

#I cannot remember which one, but one of our dependencies
#links against static runtime, so we need our apps to link
#against the static runtime too.
#https://cmake.org/Wiki/CMake_FAQ#How_can_I_build_my_MSVC_application_with_a_static_runtime.3F
IF(MSVC)
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
ELSE()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-switch")
ENDIF(MSVC)
