cmake_minimum_required(VERSION 3.12)
set(PSMOVEPROTOCOL_LIBRARY_SRC)    # Container for source files

# Source files and headers
file(GLOB PSMOVEPROTOCOL_LIBRARY_SRC
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/*.h"
)

# PSMoveProtocol library
add_library(PSMoveProtocol OBJECT ${PSMOVEPROTOCOL_LIBRARY_SRC})
set_property(TARGET PSMoveProtocol PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(PSMoveProtocol PRIVATE ${Protobuf_LIBRARIES})
protobuf_generate(
    TARGET PSMoveProtocol
    PROTOS ${CMAKE_CURRENT_LIST_DIR}/PSMoveProtocol.proto)
target_include_directories(PSMoveProtocol
    PUBLIC
        ${CMAKE_CURRENT_BINARY_DIR}
    PRIVATE
        ${Protobuf_INCLUDE_DIRS}
)

# Install
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(FILES ProtocolVersion.h SharedConstants.h
        CONFIGURATIONS Debug
        DESTINATION ${PSM_DEBUG_INSTALL_PATH}/include)
    install(FILES ProtocolVersion.h SharedConstants.h
        CONFIGURATIONS Release
        DESTINATION ${PSM_RELEASE_INSTALL_PATH}/include)
ELSE() #Linux/Darwin
    install(FILES ProtocolVersion.h SharedConstants.h
	DESTINATION include)
ENDIF()
