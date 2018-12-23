cmake_minimum_required(VERSION 2.8.12)
set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/..)

include(${ROOT_DIR}/cmake/Environment.cmake)
include(${ROOT_DIR}/cmake/ThirdParty.cmake)

# Language bindings

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/bindings)

option(PSMSERVICE_BUILD_CSHARP_BINDINGS "Build the C# bindings" ON)
option(PSMSERVICE_BUILD_PYTHON_BINDINGS "Build the Python bindings" ON)

# useful if multiple versions of the Python lib are installed
set(PSMSERVICE_PYTHON_VERSION "" CACHE STRING "Use this version of the Python lib for building Python bindings")

# Language bindings (Python and C#)
if(SWIG_FOUND)
    include(${SWIG_USE_FILE})

    if(PSMSERVICE_BUILD_PYTHON_BINDINGS)
        find_package(PythonLibs ${PSMSERVICE_PYTHON_VERSION} QUIET)
        if(PYTHONLIBS_FOUND)
            unset(CMAKE_SWIG_FLAGS)
            include_directories(${PYTHON_INCLUDE_PATH})
            include_directories(${ROOT_DIR}/src/psmoveclient)
            include_directories(${ROOT_DIR}/src/psmoveprotocol)
            swig_add_library(PSMoveClient_python LANGUAGE python SOURCES ${CMAKE_CURRENT_LIST_DIR}/PSMoveClient.i)
            swig_link_libraries(PSMoveClient_python PSMoveClient_CAPI ${PYTHON_LIBRARIES})
            set_target_properties(_PSMoveClient_python PROPERTIES FOLDER Client)
            set(INFO_BUILD_PYTHON_BINDINGS "Yes (version ${PYTHONLIBS_VERSION_STRING})")
            
            install(TARGETS _PSMoveClient_python CONFIGURATIONS Debug DESTINATION ${PSM_DEBUG_INSTALL_PATH}/bin)
            install(TARGETS _PSMoveClient_python CONFIGURATIONS Debug DESTINATION ${PSM_RELEASE_INSTALL_PATH}/bin)
        else()
            set(INFO_BUILD_PYTHON_BINDINGS "No (libpython not found)")
        endif()
    else()
        set(INFO_BUILD_PYTHON_BINDINGS "No (disabled)")
    endif()

    if(PSMSERVICE_BUILD_CSHARP_BINDINGS)
        set(PSMSERVICE_CSHARP_NS "PSMoveService")
        set(CMAKE_SWIG_FLAGS -namespace ${PSMSERVICE_CSHARP_NS})
        include_directories(${ROOT_DIR}/src/psmoveclient)
        include_directories(${ROOT_DIR}/src/psmoveprotocol)
        swig_add_library(PSMoveClient_csharp LANGUAGE csharp SOURCES ${CMAKE_CURRENT_LIST_DIR}/PSMoveClient.i)
        swig_link_libraries(PSMoveClient_csharp PSMoveClient_CAPI)
        set_target_properties(PSMoveClient_csharp PROPERTIES FOLDER Client)
        set(INFO_BUILD_CSHARP_BINDINGS "Yes")
        
        install(TARGETS PSMoveClient_csharp CONFIGURATIONS Debug DESTINATION ${PSM_DEBUG_INSTALL_PATH}/bin)
        install(TARGETS PSMoveClient_csharp CONFIGURATIONS Debug DESTINATION ${PSM_RELEASE_INSTALL_PATH}/bin)
    else()
        set(INFO_BUILD_CSHARP_BINDINGS "No (disabled)")
    endif()
else()
    set(INFO_BUILD_PYTHON_BINDINGS "No (SWIG not found)")
    set(INFO_BUILD_CSHARP_BINDINGS "No (SWIG not found)")
endif()

message("")
message("  Language bindings")
message("    Python:           " ${INFO_BUILD_PYTHON_BINDINGS})
message("    C#:               " ${INFO_BUILD_CSHARP_BINDINGS})
message("")