cmake_minimum_required(VERSION 3.0)
include (GenerateExportHeader)

#
# PSMoveClient Shared library
#

set(CMAKE_INSTALL_PREFIX ${ROOT_DIR}/dist)

set(PSMOVE_CLIENT_INCL_DIRS)
set(PSMOVE_CLIENT_REQ_LIBS)

list(APPEND PSMOVE_CLIENT_INCL_DIRS
    ${ROOT_DIR}/thirdparty/Boost.Application/include/
    ${ROOT_DIR}/thirdparty/type_index/include/)

# Protobuf
list(APPEND PSMOVE_CLIENT_INCL_DIRS ${PROTOBUF_INCLUDE_DIRS})
list(APPEND PSMOVE_CLIENT_REQ_LIBS ${PROTOBUF_LIBRARIES})

# Boost
find_package(Boost REQUIRED QUIET COMPONENTS system)
list(APPEND PSMOVE_CLIENT_INCL_DIRS ${Boost_INCLUDE_DIRS})
list(APPEND PSMOVE_CLIENT_REQ_LIBS ${Boost_LIBRARIES})

# PSMoveProtocol
include_directories(${ROOT_DIR}/src/psmoveprotocol/)
list(APPEND PSMOVE_CLIENT_REQ_LIBS PSMoveProtocol)

# PSMoveMath
include_directories(${ROOT_DIR}/src/psmovemath/)
list(APPEND PSMOVE_CLIENT_REQ_LIBS PSMoveMath)

# Source files that are needed for the shared library
file(GLOB PSMOVECLIENT_LIBRARY_SRC
    "${CMAKE_CURRENT_LIST_DIR}/*.h"
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
)

# TODO: Build PSMoveClient as a STATIC or OBJECT w/ $<TARGET_OBJECTS:objlib>
add_library(PSMoveClient_static STATIC ${PSMOVECLIENT_LIBRARY_SRC})
target_include_directories(PSMoveClient_static PUBLIC ${PSMOVE_CLIENT_INCL_DIRS})
target_link_libraries(PSMoveClient_static PUBLIC ${PLATFORM_LIBS} ${PSMOVE_CLIENT_REQ_LIBS})
target_compile_definitions(PSMoveClient_static PRIVATE PSMOVECLIENT_CPP_API)
target_compile_definitions(PSMoveClient_static PRIVATE PSMoveClient_STATIC)

#
# PSMoveClient_CAPI Shared library
#
set(PSMOVE_CLIENT_CAPI_REQ_LIBS)

# PSMoveClient_static
list(APPEND PSMOVE_CLIENT_CAPI_REQ_LIBS PSMoveClient_static)
#Via PSMoveClient_static, transitively inherits PSMoveProtocol < Protobuf, Boost, PSMoveMath

# Source files to develop the shared library.
list(APPEND PSMOVECLIENT_CAPI_LIBRARY_SRC
    "${CMAKE_CURRENT_LIST_DIR}/PSMoveClient_export.h"
    "${CMAKE_CURRENT_LIST_DIR}/PSMoveClient_CAPI.h"
    "${CMAKE_CURRENT_LIST_DIR}/PSMoveClient_CAPI.cpp"
)

# Shared library
add_library(PSMoveClient_CAPI SHARED ${PSMOVECLIENT_LIBRARY_SRC})
target_include_directories(PSMoveClient_CAPI PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(PSMoveClient_CAPI PRIVATE ${PSMOVE_CLIENT_CAPI_REQ_LIBS})
set_target_properties(PSMoveClient_CAPI PROPERTIES PUBLIC_HEADER "ClientConstants.h;ClientGeometry_CAPI.h;PSMoveClient_CAPI.h;PSMoveClient_export.h")
set_target_properties(PSMoveClient_CAPI PROPERTIES CXX_VISIBILITY_PRESET hidden)
set_target_properties(PSMoveClient_CAPI PROPERTIES C_VISIBILITY_PRESET hidden)

# Install
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS PSMoveClient_CAPI
        CONFIGURATIONS Debug
        RUNTIME DESTINATION ${PSM_DEBUG_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib
        PUBLIC_HEADER DESTINATION ${PSM_DEBUG_INSTALL_PATH}/include)
    install(TARGETS PSMoveClient_CAPI
        CONFIGURATIONS Release
        RUNTIME DESTINATION ${PSM_RELEASE_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib
        PUBLIC_HEADER DESTINATION ${PSM_RELEASE_INSTALL_PATH}/include)                
ELSE() #Linux/Darwin
    install(TARGETS PSMoveClient_CAPI
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        PUBLIC_HEADER DESTINATION include
    )
ENDIF()
