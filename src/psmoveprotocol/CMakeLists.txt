set(PSMOVEPROTOCOL_LIBRARY_SRC)    # Container for source files

# Source files and headers
file(GLOB PSMOVEPROTOCOL_LIBRARY_SRC
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/*.h"
)

# Static library
add_library(PSMoveProtocol OBJECT ${PSMOVEPROTOCOL_LIBRARY_SRC})
target_link_libraries(PSMoveProtocol
    PRIVATE
        protobuf::libprotoc
        protobuf::libprotobuf
)
protobuf_generate(
    TARGET PSMoveProtocol
    PROTOS ${CMAKE_CURRENT_LIST_DIR}/PSMoveProtocol.proto)
# Anything that links PSMoveProtocol needs access to the generated pb.h file.
target_include_directories(PSMoveProtocol PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

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