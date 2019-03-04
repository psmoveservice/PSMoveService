cmake_minimum_required(VERSION 3.0)
include (GenerateExportHeader)

#
# PSMoveClient Shared library
#

set(CMAKE_INSTALL_PREFIX ${ROOT_DIR}/dist)

set(PSMOVECLIENT_LIBRARY_SRC)
set(PSMOVECLIENT_INCL_DIRS)
set(PSMOVECLIENT_REQ_LIBS)

list(APPEND PSMOVECLIENT_INCL_DIRS
	${ROOT_DIR}/thirdparty/type_index/include/
    ${ROOT_DIR}/thirdparty/Boost.Application/include/)
# https://github.com/cboulay/PSMoveService/issues/39 ?

# Protobuf
list(APPEND PSMOVECLIENT_REQ_LIBS protobuf::libprotoc protobuf::libprotobuf)

# Boost
find_package(Boost REQUIRED QUIET COMPONENTS system)
list(APPEND PSMOVECLIENT_INCL_DIRS ${Boost_INCLUDE_DIRS})
list(APPEND PSMOVECLIENT_REQ_LIBS ${Boost_LIBRARIES})

# PSMoveProtocol
# Alternatively, could do: list(APPEND PSMOVECLIENT_LIBRARY_SRC) $<TARGET_OBJECTS:PSMoveProtocol>?
list(APPEND PSMOVECLIENT_INCL_DIRS ${ROOT_DIR}/src/psmoveprotocol/)
list(APPEND PSMOVECLIENT_REQ_LIBS PSMoveProtocol)

# PSMoveMath
list(APPEND PSMOVECLIENT_INCL_DIRS ${ROOT_DIR}/src/psmovemath/)
list(APPEND PSMOVECLIENT_REQ_LIBS PSMoveMath)

# Source files that are needed for the shared library
file(GLOB PSMOVECLIENT_LIBRARY_SRC
    "${CMAKE_CURRENT_LIST_DIR}/*.h"
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
)

# Build PSMoveClient CPP SHARED library.
add_library(PSMoveClient SHARED ${PSMOVECLIENT_LIBRARY_SRC})
target_include_directories(PSMoveClient PRIVATE ${PSMOVECLIENT_INCL_DIRS})
target_link_libraries(PSMoveClient PRIVATE ${PLATFORM_LIBS} ${PSMOVECLIENT_REQ_LIBS})
set_target_properties(PSMoveClient PROPERTIES PUBLIC_HEADER "ClientConstants.h;PSMoveClient_export.h; PSMoveClient.h")
target_compile_definitions(PSMoveClient PRIVATE PSMoveClient_EXPORTS)
target_compile_definitions(PSMoveClient PRIVATE PSMOVECLIENT_CPP_API)

# Build PSMoveClient_CAPI SHARED library.
add_library(PSMoveClient_CAPI SHARED ${PSMOVECLIENT_LIBRARY_SRC})
target_include_directories(PSMoveClient_CAPI PRIVATE ${PSMOVECLIENT_INCL_DIRS})
target_link_libraries(PSMoveClient_CAPI PRIVATE ${PLATFORM_LIBS} ${PSMOVECLIENT_REQ_LIBS})
set_target_properties(PSMoveClient_CAPI PROPERTIES PUBLIC_HEADER "ClientConstants.h;ClientGeometry_CAPI.h;PSMoveClient_CAPI.h;PSMoveClient_export.h")
set_target_properties(PSMoveClient_CAPI PROPERTIES CXX_VISIBILITY_PRESET hidden)
set_target_properties(PSMoveClient_CAPI PROPERTIES C_VISIBILITY_PRESET hidden)
target_compile_definitions(PSMoveClient_CAPI PRIVATE PSMoveClient_CAPI_EXPORTS)


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
