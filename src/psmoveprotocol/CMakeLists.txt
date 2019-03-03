set(PSMOVEPROTOCOL_LIBRARY_SRC)    # Container for source files
set(PSMOVEPROTOCOL_INCLUDE_DIRS)   # Container for include directories
set(PSMOVEPROTOCOL_REQUIRED_LIBS)  # Container for library dependencies

# Dependencies

# Protobuf
#list(APPEND PSMOVEPROTOCOL_INCLUDE_DIRS ${PROTOBUF_INCLUDE_DIRS})
list(APPEND PSMOVEPROTOCOL_REQUIRED_LIBS protobuf::libprotoc protobuf::libprotobuf)
# protobuf_generate(PROTO_SRCS PROTO_HDRS ${CMAKE_CURRENT_LIST_DIR}/PSMoveProtocol.proto)
# See http://stackoverflow.com/questions/20824194/cmake-with-google-protocol-buffers
# https://stackoverflow.com/questions/19343018/cmake-and-findprotobuf

# Source files and headers
file(GLOB PSMOVEPROTOCOL_LIBRARY_SRC
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/*.h"
)

list(APPEND PSMOVEPROTOCOL_LIBRARY_SRC ${CMAKE_CURRENT_LIST_DIR}/PSMoveProtocol.proto)

# Static library
add_library(PSMoveProtocol STATIC ${PSMOVEPROTOCOL_LIBRARY_SRC})
protobuf_generate(TARGET PSMoveProtocol PROTOS ${CMAKE_CURRENT_LIST_DIR}/PSMoveProtocol.proto)
target_include_directories(PSMoveProtocol PUBLIC ${PSMOVEPROTOCOL_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(PSMoveProtocol PRIVATE ${PSMOVEPROTOCOL_REQUIRED_LIBS})
set_target_properties(PSMoveProtocol PROPERTIES
    COMPILE_FLAGS "-DBUILDING_STATIC_LIBRARY -fPIC")
    
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